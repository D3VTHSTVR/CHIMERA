#!/usr/bin/env python3
"""
Run the two-motor SNN and print/plot alternating motor signals.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from snn_walk import make_default_snn


def main():
    snn = make_default_snn()
    dt = snn.dt
    duration = 5.0   # seconds
    n_steps = int(duration / dt)

    motor1_log = []
    motor2_log = []
    time_log = []

    print("Two-motor SNN: alternating walk pattern")
    print("Step   t(s)    Motor1   Motor2   (N1 active -> M1 high, N2 suppressed -> M2 low; then switch)")
    print("-" * 70)

    for k in range(n_steps):
        t = k * dt
        m1, m2 = snn.step()
        motor1_log.append(m1)
        motor2_log.append(m2)
        time_log.append(t)
        if k % 50 == 0 or k < 10:
            print(f"{k:5d}  {t:5.2f}   {m1:7.3f}   {m2:7.3f}")

    print("-" * 70)
    print("Done. Motor outputs alternate: one high while the other is low, then they switch.")

    # Optional: plot if matplotlib available
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(8, 4))
        ax[0].plot(time_log, motor1_log, label="Motor 1")
        ax[0].set_ylabel("Motor 1")
        ax[0].legend(loc="right")
        ax[0].set_ylim(-0.05, 1.05)
        ax[1].plot(time_log, motor2_log, color="C1", label="Motor 2")
        ax[1].set_ylabel("Motor 2")
        ax[1].set_xlabel("Time (s)")
        ax[1].legend(loc="right")
        ax[1].set_ylim(-0.05, 1.05)
        plt.suptitle("SNN alternating motor outputs (walk pattern)")
        plt.tight_layout()
        plt.savefig(os.path.join(os.path.dirname(__file__), "motor_signals.png"), dpi=120)
        print("Plot saved to motor_signals.png")
    except ImportError:
        print("Install matplotlib to generate motor_signals.png")


if __name__ == "__main__":
    main()
