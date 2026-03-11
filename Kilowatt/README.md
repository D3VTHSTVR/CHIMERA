# Two-Motor SNN Walk (PicoGo)

A **spiking neural network** drives a two-wheeled robot with alternating motor signals. Built for the **Waveshare PicoGo V2** with obstacle avoidance, pick-up detection, and optional line tracking.

## Features

- **SNN gait**: Two reciprocally coupled neurons alternate to produce a walking rhythm.
- **Obstacle avoidance**: Ultrasonic (HC-SR04) + IR sensors steer around obstacles; alternates left/right turns to avoid getting stuck.
- **Pick-up detection**: Stops motors when lifted (ultrasonic sees "far" persistently).
- **Stuck recovery**: Reverse + turn when blocked for ~0.5 s.
- **Line tracking** (optional): Follow a black line using the 5 IR sensors.

## Project layout

| File | Description |
|------|-------------|
| `pico_snn_main.py` | Main robot firmware. Copy to Pico as `main.py`. |
| `pico_example.py` | Minimal SNN snippet (no sensors). Reference only. |
| `snn_walk.py` | 2-neuron SNN class (Python, for simulation). |
| `simulate.py` | Run SNN on PC and plot motor outputs. |
| `PicoGo_Code V2/` | Waveshare demo files. Need `Motor.py` and `TRSensor.py` for the robot. |

## Hardware

- **Board**: Waveshare PicoGo V2 (Raspberry Pi Pico)
- **Sensors**: Ultrasonic (HC-SR04), IR obstacle (left/right), IR line (5 sensors, optional)
- **Actuation**: Two N20 motors via TB6612

## Deploy to robot

### First-time setup

1. Flash MicroPython on the Pico ([download](https://micropython.org/download/RPI_PICO/)).
2. Install `mpremote`: `pip install mpremote`
3. Connect Pico via USB (without BOOTSEL).

### Copy files

```bash
cd ~/Desktop/CS485
# Motor driver (required)
mpremote connect /dev/cu.usbmodem* fs cp "Kilowatt/PicoGo_Code V2/Motor.py" :Motor.py
# Main firmware
mpremote connect /dev/cu.usbmodem* fs cp Kilowatt/pico_snn_main.py :main.py
```

For line tracking, also copy TRSensor:

```bash
mpremote connect /dev/cu.usbmodem* fs cp "Kilowatt/PicoGo_Code V2/TRSensor.py" :TRSensor.py
```

### Run

1. Turn the power switch **ON**.
2. Disconnect USB or press reset. The robot starts automatically.

## Tuning (in `pico_snn_main.py`)

| Constant | Purpose |
|----------|---------|
| `LEFT_SCALE`, `RIGHT_SCALE` | Motor balance. Veering right → increase LEFT or decrease RIGHT. |
| `OBSTACLE_CM` | Distance threshold (cm) for obstacle detection. |
| `STEER_STRENGTH` | How hard to turn when avoiding. |
| `STUCK_STEPS` | Steps before reverse+turn recovery (default 50). |
| `ENABLE_LINE_TRACKING` | Set `True` for line following; needs `TRSensor.py`. |
| `ENABLE_PICKUP_DETECT` | Set `False` to disable lift-to-stop. |

## Run simulation (PC)

```bash
cd Kilowatt
pip install numpy
python simulate.py
```

## SNN idea

Two neurons N1, N2 with reciprocal inhibition. When N1 fires, N2 is suppressed (and vice versa), producing alternation. Each neuron drives one motor. Parameters `V_th`, `I_BIAS`, `W_12`, `W_21` control rhythm and balance.
