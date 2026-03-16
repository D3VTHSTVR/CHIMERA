"""
Balance environment for 6-legged insect using MuJoCo (same API as balance_env.py).
Used when PyBullet is not installed. State 40, action 18; reward: balance, forward velocity, foot clearance, straight walking.
"""

import os
import sys
import numpy as np
import mujoco
# Viewer is a submodule; some installs require explicit import for mujoco.viewer to exist
try:
    import mujoco.viewer
except ImportError:
    mujoco.viewer = None  # no viewer (e.g. headless or old mujoco)

N_JOINTS = 18

# MuJoCo free joint: qpos[0:3]=pos, qpos[3:7]=quat(w,x,y,z), qpos[7:25]=18 joints; qvel[0:6]=lin+ang, qvel[6:24]=18
def _q_to_state(q, v):
    qw, qx, qy, qz = q[3], q[4], q[5], q[6]
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    state = np.zeros(40, dtype=np.float32)
    state[0] = roll
    state[1] = pitch
    state[2] = v[3]
    state[3] = v[4]
    state[4:22] = q[7:25]
    state[22:40] = v[6:24]
    return state


def _find_model():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for rel in ["robot/six_leg_insect.xml", "SimpleChimera/robot/six_leg_insect.xml"]:
        path = os.path.join(base, rel)
        if os.path.isfile(path):
            return path
    path = os.path.join(base, "..", "robot", "six_leg_insect.xml")
    if os.path.isfile(path):
        return os.path.abspath(path)
    return None


class BalanceEnv:
    """MuJoCo-backed balance env: same state (40), action (18), and reward as PyBullet version."""

    def __init__(
        self,
        gui=False,
        time_step=0.002,
        gravity_scale=1.0,
        ground_friction=3.2,
        contact_stiffness=400,
        contact_damping=250,
        max_torque=0.1,
        max_steps=2500,
        seed=None,
        forward_vel_weight=0.0,
        foot_clearance_weight=0.0,
        **kwargs,
    ):
        self.time_step = time_step
        self.max_torque = max_torque
        self.forward_vel_weight = float(forward_vel_weight)
        self.foot_clearance_weight = float(foot_clearance_weight)
        self.max_steps = max_steps
        self._gui = bool(gui)
        self._viewer = None
        self._step_count = 0
        self._state = None
        if seed is not None:
            np.random.seed(seed)
        path = _find_model()
        if not path:
            raise FileNotFoundError("six_leg_insect.xml not found; run from SimpleChimera or set path")
        self._model = mujoco.MjModel.from_xml_path(path)
        self._model.opt.timestep = time_step
        self._model.opt.gravity[2] = -9.81 * gravity_scale
        self._data = mujoco.MjData(self._model)
        # Foot geom ids for clearance reward (reward lifting feet = stepping, not scooting)
        self._floor_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._foot_geom_ids = np.array([
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("L1_foot", "L2_foot", "L3_foot", "R1_foot", "R2_foot", "R3_foot")
        ])

    def reset(self):
        mujoco.mj_resetData(self._model, self._data)
        self._data.qpos[0] = 0
        self._data.qpos[1] = 0
        # Lower base so feet touch ground (0.102 left feet ~0.017 above ground; 0.085 puts feet on floor)
        self._data.qpos[2] = 0.085
        self._data.qpos[3] = 1
        self._data.qpos[4] = 0
        self._data.qpos[5] = 0
        self._data.qpos[6] = 0
        mujoco.mj_forward(self._model, self._data)
        self._step_count = 0
        if self._gui and self._viewer is None:
            viewer_mod = getattr(mujoco, "viewer", None)
            if viewer_mod is None or not hasattr(viewer_mod, "launch_passive"):
                raise RuntimeError(
                    "MuJoCo viewer not available (module 'mujoco' has no attribute 'viewer'). "
                    "Install with: pip install mujoco (3.x). On macOS you may need: mjpython run.py --policy best_policy.npz --gui"
                )
            try:
                self._viewer = viewer_mod.launch_passive(self._model, self._data)
            except RuntimeError as e:
                if "mjpython" in str(e).lower() or (sys.platform == "darwin" and "macOS" in str(e)):
                    raise RuntimeError(
                        "On macOS, MuJoCo's GUI must be run with mjpython. Use:\n  mjpython run.py --policy best_policy.npz --gui"
                    ) from e
                raise
        self._state = _q_to_state(self._data.qpos, self._data.qvel)
        return self._state.copy()

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64).ravel(), -1.0, 1.0)
        # Use same torque as training so policy is stable (no GUI-only scale)
        ctrl = np.asarray(action * self.max_torque, dtype=np.float64)
        # Mirror right legs (R1,R2,R3 = actuator indices 9..17): flip sign so policy output matches left
        ctrl[9:18] = -ctrl[9:18]
        nu = self._model.nu
        if nu != len(ctrl):
            ctrl = np.resize(ctrl, nu)
        np.copyto(self._data.ctrl, ctrl)
        mujoco.mj_step(self._model, self._data)
        self._step_count += 1
        if self._viewer is not None:
            self._viewer.sync()
        state = _q_to_state(self._data.qpos, self._data.qvel)
        roll, pitch = state[0], state[1]
        roll_dot, pitch_dot = state[2], state[3]
        tilt = np.sqrt(roll ** 2 + pitch ** 2)
        ang_speed = np.sqrt(roll_dot ** 2 + pitch_dot ** 2)
        reward = -0.6 * tilt - 0.06 * ang_speed
        lin_vel = self._data.qvel[0:3]
        # Forward velocity (world +X): main objective so policy learns to walk
        vx = lin_vel[0]
        if self.forward_vel_weight > 0:
            reward += self.forward_vel_weight * max(0.0, vx)
            # Penalize standing still so evolution quickly favors moving policies
            if abs(vx) < 0.02:
                reward -= 0.12
            # Slight bonus for leg motion so policy discovers stepping
            leg_vel = np.mean(np.abs(state[22:40]))
            reward += 0.05 * min(1.0, leg_vel * 5.0)
        # Foot clearance: reward feet in the air (stepping) to discourage scooting
        if self.foot_clearance_weight > 0:
            feet_in_contact = set()
            for i in range(self._data.ncon):
                con = self._data.contact[i]
                g1, g2 = con.geom1, con.geom2
                if g1 == self._floor_geom_id or g2 == self._floor_geom_id:
                    foot_id = g2 if g1 == self._floor_geom_id else g1
                    if foot_id in self._foot_geom_ids:
                        feet_in_contact.add(int(foot_id))
            n_feet_in_air = 6 - len(feet_in_contact)
            reward += self.foot_clearance_weight * (n_feet_in_air / 6.0)
        # Discourage turning (yaw) and lateral drift so bug walks straight
        reward -= 0.45 * abs(lin_vel[1])  # lateral drift (stronger so less left/right bias)
        reward -= 0.12 * abs(self._data.qvel[5])  # yaw rate (rotation around Z)
        reward -= 0.2 * max(0.0, -vx)  # backward
        done = tilt > 1.0 or self._step_count >= self.max_steps
        done_reason = "fell" if tilt > 1.0 else ("timeout" if self._step_count >= self.max_steps else None)
        self._state = state
        return state, float(reward), done, {"done_reason": done_reason}

    def close(self):
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None

    @property
    def state_dim(self):
        return 40

    @property
    def action_dim(self):
        return N_JOINTS
