"""
Two-neuron SNN for alternating two motors (walk pattern).

Architecture (matches N1–N2 whiteboard):
- N1 and N2: leaky integrate-and-fire neurons with reciprocal inhibition.
- N1 -> Motor 1,  N2 -> Motor 2.
- When N1 is active (high output), N2 is inhibited (low); then they switch.
- Parameters: V_th (threshold), dt (time step), W_ij (weights).
"""

import numpy as np


class TwoMotorSNN:
    """
    Two neurons N1, N2. Each drives one motor.
    Reciprocal coupling: N1 inhibits N2, N2 inhibits N1 → alternating activity.
    Optional self-feedback (recurrent) for persistence.
    """

    def __init__(
        self,
        V_th=1.0,
        tau=0.05,
        dt=0.01,
        W_12=-1.2,   # N1 -> N2 (inhibitory)
        W_21=-1.2,   # N2 -> N1 (inhibitory)
        W_11=0.15,   # N1 self-feedback (excitatory)
        W_22=0.15,   # N2 self-feedback
        I_bias=0.6,  # constant drive to both (so they can alternate)
        V_reset=0.0,
    ):
        self.V_th = float(V_th)
        self.tau = float(tau)
        self.dt = float(dt)
        self.W_12 = float(W_12)
        self.W_21 = float(W_21)
        self.W_11 = float(W_11)
        self.W_22 = float(W_22)
        self.I_bias = float(I_bias)
        self.V_reset = float(V_reset)
        self.refrac_steps = max(1, int(0.08 / dt))  # ~80 ms refractory so the other neuron can reach threshold
        # State (N1 near threshold so it fires first and breaks symmetry)
        self.V1 = 0.92
        self.V2 = 0.0
        self.s1 = 0.0  # smoothed spike output (for motor)
        self.s2 = 0.0
        self._alpha = 0.9   # smoothing for s1, s2
        self.refrac1 = 0
        self.refrac2 = 0

    def reset(self):
        self.V1 = 0.92
        self.V2 = 0.0
        self.s1 = 0.0
        self.s2 = 0.0
        self.refrac1 = 0
        self.refrac2 = 0

    def step(self):
        """
        One time step: update membrane potentials, apply threshold, then motor outputs.
        Coupling uses smoothed activity (s1, s2) so inhibition persists and neurons alternate.
        Returns (motor1, motor2) in [0, 1] for duty cycle / PWM.
        """
        dt = self.dt
        tau = max(self.tau, dt * 2)

        # Binary spike for coupling (with refractory, the other neuron gets a clear window to integrate)
        spike1 = 1.0 if self.V1 >= self.V_th else 0.0
        spike2 = 1.0 if self.V2 >= self.V_th else 0.0
        I1 = self.I_bias + self.W_11 * spike1 + self.W_21 * spike2  # N2 spike inhibits N1
        I2 = self.I_bias + self.W_22 * spike2 + self.W_12 * spike1  # N1 spike inhibits N2

        # Integrate (skip when in refractory period so the other neuron can rise)
        if self.refrac1 <= 0:
            self.V1 = self.V1 + (I1 - self.V1) / tau * dt
        else:
            self.V1 = self.V_reset
            self.refrac1 -= 1
        if self.refrac2 <= 0:
            self.V2 = self.V2 + (I2 - self.V2) / tau * dt
        else:
            self.V2 = self.V_reset
            self.refrac2 -= 1

        # Spike and refractory
        if self.V1 >= self.V_th:
            self.V1 = self.V_reset
            self.refrac1 = self.refrac_steps
        if self.V2 >= self.V_th:
            self.V2 = self.V_reset
            self.refrac2 = self.refrac_steps

        # Clamp to avoid blow-up
        self.V1 = np.clip(self.V1, -0.5, self.V_th + 0.2)
        self.V2 = np.clip(self.V2, -0.5, self.V_th + 0.2)

        # Motor output: smoothed rate (0–1) for PWM
        rate1 = min(1.0, max(0.0, self.V1) / self.V_th)
        rate2 = min(1.0, max(0.0, self.V2) / self.V_th)
        self.s1 = self._alpha * self.s1 + (1 - self._alpha) * rate1
        self.s2 = self._alpha * self.s2 + (1 - self._alpha) * rate2

        return (float(self.s1), float(self.s2))

    def get_motor_commands(self):
        """Current motor outputs (0–1). Call after step()."""
        return (self.s1, self.s2)


def make_default_snn():
    """SNN that alternates cleanly for two motors (reciprocal inhibition + refractory)."""
    return TwoMotorSNN(
        V_th=1.0,
        tau=0.02,
        dt=0.01,
        W_12=-1.2,
        W_21=-1.2,
        W_11=0.0,    # no self-feedback so alternation is clear
        W_22=0.0,
        I_bias=1.15, # above V_th so when other is silent we can fire
    )
