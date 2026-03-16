"""
Kilowatt-style 2-neuron SNN chain for SimpleChimera balance/march.

Uses 3 coupled TwoMotorSNN modules (front, middle, rear). Each module drives
one leg pair: (L1,R1), (L2,R2), (L3,R3). Outputs 18 joint torques in [-1, 1]
for direct torque control in BalanceEnv.

Joint order (matches balance_env): L1_coxa,femur,tibia, L2_..., L3_..., R1_..., R2_..., R3_....
"""

import numpy as np
import sys
from pathlib import Path

# TwoMotorSNN from Chimera Kilowatt package
_chimera_root = Path(__file__).resolve().parents[2]
if str(_chimera_root) not in sys.path:
    sys.path.insert(0, str(_chimera_root))
from Kilowatt.snn_walk import TwoMotorSNN

N_JOINTS = 18
# Leg order: L1=0, L2=1, L3=2, R1=3, R2=4, R3=5. Front=(L1,R1), Mid=(L2,R2), Rear=(L3,R3)
LEG_FROM_JOINT = np.repeat(np.arange(6), 3)  # joint i -> leg LEG_FROM_JOINT[i]
# NeuroMechFly-style tripod: A = L1, R2, L3 (0,4,2); B = R1, L2, R3 (3,1,5) — 180° out of phase
TRIPOD_A_LEGS = (0, 4, 2)   # L1, R2, L3
TRIPOD_B_LEGS = (3, 1, 5)   # R1, L2, R3
SNN_INDEX = [0, 1, 2, 0, 1, 2]
MOTOR_INDEX = [0, 0, 0, 1, 1, 1]


class KilowattBalanceController:
    """
    Chain of 3 TwoMotorSNN modules. Outputs 18 joint torques in [-1, 1].
    Optional intraleg phase: Coxa=0, Femur=+pi/2, Tibia=+pi for more natural swing.
    """

    def __init__(
        self,
        amplitude=0.5,
        dt_snn=0.01,
        time_step=0.002,
        k_couple=0.1,
        tau=0.05,
        I_bias=0.95,
        use_phase_offset=True,
    ):
        self.amplitude = float(amplitude)
        self.dt_snn = float(dt_snn)
        self.time_step = float(time_step)
        self.k_couple = float(k_couple)
        self.use_phase_offset = bool(use_phase_offset)
        # Run several SNN steps per env step so rhythm builds up (smoothing in SNN makes output tiny with only 1 step)
        self._steps_per_env_step = max(10, int(0.02 / time_step))  # at least 10 steps; or ~10 steps per 0.02s
        self._dt_snn_actual = time_step / self._steps_per_env_step

        self._snns = [
            TwoMotorSNN(tau=tau, dt=dt_snn, I_bias=I_bias),
            TwoMotorSNN(tau=tau, dt=dt_snn, I_bias=I_bias),
            TwoMotorSNN(tau=tau, dt=dt_snn, I_bias=I_bias),
        ]
        # Slight phase offset for wave: front -> mid -> rear
        self._snns[1].V1, self._snns[1].V2 = 0.6, 0.3
        self._snns[2].V1, self._snns[2].V2 = 0.3, 0.6

        # Phase offsets for stepping: coxa swings, femur lifts (pi/2), tibia extends (pi)
        # So during swing: femur leads (lift), then coxa (swing), then tibia (extend)
        self._joint_phase_offset = np.array(
            [0, np.pi/2, np.pi] * 6, dtype=np.float64
        )
        # Step-style: use sharper waveform so legs drive through range (lift/place) not slide
        self._step_style = True

    def reset(self):
        for s in self._snns:
            s.reset()
        self._snns[1].V1, self._snns[1].V2 = 0.6, 0.3
        self._snns[2].V1, self._snns[2].V2 = 0.3, 0.6

    def act(self, state):
        """
        state: (40,) ignored for pure Kilowatt rhythm; kept for API compatibility.
        Returns (18,) torques in [-1, 1].
        """
        # Step SNNs with coupling (multiple SNN steps per env step)
        for _ in range(self._steps_per_env_step):
            self._snns[0].step()
        m1_f, m2_f = self._snns[0].get_motor_commands()

        drive_mid = self.k_couple * (m1_f + m2_f)
        orig = self._snns[1].I_bias
        self._snns[1].I_bias = orig + drive_mid
        for _ in range(self._steps_per_env_step):
            self._snns[1].step()
        self._snns[1].I_bias = orig
        m1_m, m2_m = self._snns[1].get_motor_commands()

        drive_rear = self.k_couple * (m1_m + m2_m)
        orig_r = self._snns[2].I_bias
        self._snns[2].I_bias = orig_r + drive_rear
        for _ in range(self._steps_per_env_step):
            self._snns[2].step()
        self._snns[2].I_bias = orig_r
        m1_r, m2_r = self._snns[2].get_motor_commands()

        # Leg motor values: (m1,m2) per segment -> L1,L2,L3,R1,R2,R3
        leg_motors = np.array([m1_f, m1_m, m1_r, m2_f, m2_m, m2_r], dtype=np.float64)
        # Base phase from motor [0,1] -> [-pi, pi]
        base_phase = np.pi * (2.0 * leg_motors - 1.0)
        # NeuroMechFly-style: Tripod B (L2,R1,R3) 180° out of phase with Tripod A (L1,R2,L3)
        phase_leg_6 = base_phase.copy()
        phase_leg_6[list(TRIPOD_B_LEGS)] += np.pi
        # Expand to 18 joints (3 per leg)
        phase_leg = np.repeat(phase_leg_6, 3)
        if self.use_phase_offset:
            phase = phase_leg + self._joint_phase_offset
            if getattr(self, "_step_style", False):
                # Step-style: sharp but smooth waveform so legs drive through range (lift/place)
                out = self.amplitude * np.tanh(3.0 * np.sin(phase))
            else:
                out = self.amplitude * np.sin(phase)
        else:
            # Simple: same torque for all 3 joints of each leg
            out = self.amplitude * (2.0 * np.repeat(leg_motors, 3) - 1.0)
        out = np.clip(out, -1.0, 1.0).astype(np.float32)
        return out


def make_kilowatt_controller(amplitude=0.5, **kwargs):
    """Factory for run/train scripts."""
    return KilowattBalanceController(amplitude=amplitude, **kwargs)
