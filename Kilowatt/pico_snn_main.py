"""
SNN-driven two-motor walk for PicoGo.
1. Obstacle avoidance (ultrasonic + IR)
2. Line tracking (IR TRSensor) - optional
3. Pick-up detection - stop when lifted

Copy to Pico as main.py. Requires Motor.py. For line tracking, also copy TRSensor.py.
"""
import time
import utime
from machine import Pin
from Motor import PicoGo

# --- Feature flags ---
ENABLE_LINE_TRACKING = False  # set True for line follow; needs good calibration
ENABLE_PICKUP_DETECT = True

# --- Obstacle sensors (PicoGo V2) ---
DSR = Pin(2, Pin.IN)   # IR right: 0=obstacle
DSL = Pin(3, Pin.IN)   # IR left:  0=obstacle
Trig = Pin(14, Pin.OUT)
Echo = Pin(15, Pin.IN)
Trig.value(0)

OBSTACLE_CM = 20
STEER_STRENGTH = 50
OBSTACLE_POWER_BOOST = 1.3  # extra power when avoiding (helps over bumps)
STUCK_STEPS = 50            # obstacle mode this long -> assume stuck (sooner = faster recovery)
STUCK_REVERSE_MS = 250      # reverse duration (ms)
STUCK_TURN_MS = 300         # turn duration after reverse

# Pick-up: ultrasonic sees "far" for many steps in a row = likely lifted
PICKUP_FAR_CM = 120       # distance above this = "far"
PICKUP_FAR_COUNT = 10    # this many consecutive far readings -> stop
PICKUP_RESUME_CM = 50    # distance below this = back on ground
PICKUP_RESUME_COUNT = 5  # this many consecutive near -> drive again

# --- Line tracking (when enabled) ---
LINE_KP = 1.0 / 30.0
LINE_KD = 2.0
LINE_MAX = 100
last_proportional = 0

# --- SNN parameters ---
V_TH = 1.0
TAU = 0.02
DT = 0.01
W_12 = -1.2
W_21 = -1.2
W_11 = 0.0
W_22 = 0.0
I_BIAS = 1.15
V_RESET = 0.0
ALPHA = 0.9
REFRAC_STEPS = max(1, int(0.08 / DT))

v1, v2 = 0.92, 0.0
s1, s2 = 0.0, 0.0
refrac1, refrac2 = 0, 0

# Motor balance (no encoders; tune by veering)
LEFT_SCALE = 82
RIGHT_SCALE = 86   # slight bump to correct remaining left veer


def dist_cm():
    """Ultrasonic distance in cm. ~300 max. Returns 999 on timeout."""
    Trig.value(1)
    utime.sleep_us(10)
    Trig.value(0)
    t0 = utime.ticks_us()
    while Echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), t0) > 50000:
            return 999.0
        pass
    ts = utime.ticks_us()
    while Echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), ts) > 50000:
            return 999.0
        pass
    te = utime.ticks_us()
    return (utime.ticks_diff(te, ts) * 0.034) / 2


def snn_step():
    """Returns (motor1, motor2) in 0..1."""
    global v1, v2, s1, s2, refrac1, refrac2
    spike1 = 1.0 if v1 >= V_TH else 0.0
    spike2 = 1.0 if v2 >= V_TH else 0.0

    I1 = I_BIAS + W_11 * spike1 + W_21 * spike2
    I2 = I_BIAS + W_22 * spike2 + W_12 * spike1

    if refrac1 <= 0:
        v1 = v1 + (I1 - v1) / TAU * DT
    else:
        v1 = V_RESET
        refrac1 -= 1
    if refrac2 <= 0:
        v2 = v2 + (I2 - v2) / TAU * DT
    else:
        v2 = V_RESET
        refrac2 -= 1

    if v1 >= V_TH:
        v1 = V_RESET
        refrac1 = REFRAC_STEPS
    if v2 >= V_TH:
        v2 = V_RESET
        refrac2 = REFRAC_STEPS

    v1 = max(-0.5, min(V_TH + 0.2, v1))
    v2 = max(-0.5, min(V_TH + 0.2, v2))

    rate1 = min(1.0, max(0.0, v1) / V_TH)
    rate2 = min(1.0, max(0.0, v2) / V_TH)
    s1 = ALPHA * s1 + (1 - ALPHA) * rate1
    s2 = ALPHA * s2 + (1 - ALPHA) * rate2

    return (s1, s2)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    global last_proportional

    M = PicoGo()
    M.stop()

    TRS = None
    if ENABLE_LINE_TRACKING:
        from TRSensor import TRSensor
        TRS = TRSensor()
        # Calibration: sweep left/right on the line
        for i in range(100):
            if i < 25 or i >= 75:
                M.setMotor(30, -30)
            else:
                M.setMotor(-30, 30)
            TRS.calibrate()
            time.sleep(0.02)
        M.stop()
        time.sleep(0.5)

    far_count = 0
    near_count = 0
    lifted = False
    obstacle_steps = 0
    turn_right_next = True   # alternate turn direction to avoid getting stuck

    while True:
        m1, m2 = snn_step()
        left = int(m1 * LEFT_SCALE)
        right = int(m2 * RIGHT_SCALE)

        d = dist_cm()
        dr = DSR.value()
        dl = DSL.value()

        # 3. Pick-up detection: ultrasonic "far" for many steps = lifted
        if ENABLE_PICKUP_DETECT:
            if lifted:
                if d < PICKUP_RESUME_CM:
                    near_count += 1
                    if near_count >= PICKUP_RESUME_COUNT:
                        lifted = False
                        near_count = 0
                else:
                    near_count = 0
                if lifted:
                    M.stop()
                    time.sleep(DT)
                    continue
            else:
                if d > PICKUP_FAR_CM or d >= 999.0:
                    far_count += 1
                    if far_count >= PICKUP_FAR_COUNT:
                        lifted = True
                        far_count = 0
                else:
                    far_count = 0
                if lifted:
                    M.stop()
                    time.sleep(DT)
                    continue

        # 2. Line tracking: steer to follow line (blended with SNN)
        if ENABLE_LINE_TRACKING and TRS is not None:
            position, sensors = TRS.readLine()
            ssum = sum(sensors)
            if ssum < 4000:  # on line
                proportional = position - 2000
                derivative = proportional - last_proportional
                last_proportional = proportional
                power_diff = proportional * LINE_KP + derivative * LINE_KD
                power_diff = clamp(power_diff, -LINE_MAX, LINE_MAX)
                # Apply line correction on top of SNN
                if power_diff < 0:
                    left = clamp(left + power_diff, -100, 100)
                    right = clamp(right, -100, 100)
                else:
                    left = clamp(left, -100, 100)
                    right = clamp(right - power_diff, -100, 100)
            elif ssum > 4000:
                last_proportional = 2000

        # 1. Obstacle avoidance
        obstacle_mode = d <= OBSTACLE_CM or dl == 0 or dr == 0
        if obstacle_mode:
            obstacle_steps += 1
            # Stuck recovery: reverse then turn
            if obstacle_steps >= STUCK_STEPS:
                M.setMotor(-60, -60)
                time.sleep_ms(STUCK_REVERSE_MS)
                turn_right_next = not turn_right_next
                if turn_right_next:
                    M.setMotor(-50, 50)
                else:
                    M.setMotor(50, -50)
                time.sleep_ms(STUCK_TURN_MS)
                obstacle_steps = 0
                continue
            # When clear left/right: follow sensors. When ahead/both: alternate
            if dl == 0 and dr == 1:
                left = clamp(left - STEER_STRENGTH, -100, 100)
                right = clamp(right + STEER_STRENGTH, -100, 100)
            elif dl == 1 and dr == 0:
                left = clamp(left + STEER_STRENGTH, -100, 100)
                right = clamp(right - STEER_STRENGTH, -100, 100)
            else:
                turn_right_next = not turn_right_next
                if turn_right_next:
                    left = clamp(left - STEER_STRENGTH, -100, 100)
                    right = clamp(right + STEER_STRENGTH, -100, 100)
                else:
                    left = clamp(left + STEER_STRENGTH, -100, 100)
                    right = clamp(right - STEER_STRENGTH, -100, 100)
            # Power boost when avoiding (helps over bumps)
            left = clamp(int(left * OBSTACLE_POWER_BOOST), -100, 100)
            right = clamp(int(right * OBSTACLE_POWER_BOOST), -100, 100)
        else:
            obstacle_steps = 0

        M.setMotor(left, right)
        time.sleep(DT)


if __name__ == "__main__":
    main()
