"""
Minimal SNN for Raspberry Pi Pico / MicroPython (no numpy).

Copy the logic below into your Pico project. Map motor1, motor2 to PWM duty
for your two motors (e.g. 0–65535 or 0–100%).
"""

# --- Parameters (match make_default_snn() for alternation) ---
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
REFRAC_STEPS = max(1, int(0.08 / DT))  # ~80 ms so the other neuron can fire

# --- State ---
v1, v2 = 0.92, 0.0   # N1 head start so it fires first
s1, s2 = 0.0, 0.0
refrac1, refrac2 = 0, 0


def snn_step():
    """One time step. Returns (motor1, motor2) in 0..1. Uses refractory for alternation."""
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


# --- Example loop (adapt to your Pico main loop and PWM) ---
if __name__ == "__main__":
    import time
    for _ in range(500):
        m1, m2 = snn_step()
        # On Pico: set PWM duty from m1, m2 (e.g. motor_l.duty_u16(int(65535 * m1)))
        print(f"Motor1={m1:.3f} Motor2={m2:.3f}")
        time.sleep(DT)
