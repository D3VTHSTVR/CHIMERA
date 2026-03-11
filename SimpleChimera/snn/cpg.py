"""
Central Pattern Generator (CPG) for alternating tripod gait.

Generates a rhythmic signal: Tripod A (L1, R2, L3) and Tripod B (R1, L2, R3)
are 180° out of phase. Produces 18 joint torques (3 per leg) for marching in place.
"""

import numpy as np

N_LEGS = 6
N_JOINTS = 18  # 3 per leg
# Tripod A: L1, R2, L3 (leg indices 0, 4, 2). Tripod B: R1, L2, R3 (legs 3, 1, 5)
TRIPOD_A_LEGS = [0, 2, 4]   # L1, L3, R2
TRIPOD_B_LEGS = [1, 3, 5]   # L2, R1, R3


class TripodCPG:
    """
    Phase-based CPG for alternating tripod march.
    Phase phi advances each step; Tripod A uses sin(phi), Tripod B uses sin(phi + pi).
    """

    def __init__(self, freq_hz=0.22, amplitude=0.08, dt=0.002):
        self.freq_hz = float(freq_hz)
        self.amplitude = float(amplitude)
        self.dt = float(dt)
        self.phase = 0.0

    def reset(self):
        self.phase = 0.0

    def step(self):
        """Advance phase by one timestep."""
        self.phase += 2.0 * np.pi * self.freq_hz * self.dt
        if self.phase >= 2.0 * np.pi:
            self.phase -= 2.0 * np.pi

    def get_torque_signal(self):
        """
        Returns (18,) array in [-1, 1] for joint torques.
        Tripod A: sin(phase) -> stance when > 0, swing when < 0
        Tripod B: -sin(phase) -> opposite phase
        """
        s = np.sin(self.phase)
        leg_signals = np.zeros(N_LEGS, dtype=np.float64)
        for i in TRIPOD_A_LEGS:
            leg_signals[i] = s
        for i in TRIPOD_B_LEGS:
            leg_signals[i] = -s
        # Expand to 18 joints (3 per leg)
        out = np.repeat(leg_signals * self.amplitude, 3)
        return np.clip(out, -1.0, 1.0).astype(np.float32)
