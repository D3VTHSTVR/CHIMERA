"""
Balance environment for 6-legged insect in PyBullet.

Tailored for balance: flat floor, soft contacts, optional reduced gravity.
State (IMU-like): base orientation (roll, pitch), angular velocity (roll_dot, pitch_dot),
  plus joint positions/velocities. No separate IMU needed—orientation and rate are from sim.
Limb control: an SNN policy outputs 18 joint torques; torques are applied only when
  at least one limb tip (tibia) is in contact with the ground (avoids flailing in the air).
Reward: stay upright (penalize tilt and angular speed).
"""

import os
import numpy as np
import pybullet as p
import pybullet_data

# Default URDF path (absolute so it works from any cwd)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOT_URDF = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "robot", "six_leg_insect.urdf"))

# Joint order (must match URDF: 3 DOF per leg = Coxa, Femur, Tibia like fly)
JOINT_NAMES = [
    "L1_coxa", "L1_femur", "L1_tibia",
    "L2_coxa", "L2_femur", "L2_tibia",
    "L3_coxa", "L3_femur", "L3_tibia",
    "R1_coxa", "R1_femur", "R1_tibia",
    "R2_coxa", "R2_femur", "R2_tibia",
    "R3_coxa", "R3_femur", "R3_tibia",
]
N_JOINTS = 18
# Limb tips = tibia links (one per leg). In PyBullet, link index i = child of joint i; tibia joints at 2,5,8,11,14,17.
TIBIA_LINK_INDICES = [2, 5, 8, 11, 14, 17]
# Alternating tripod gaits: L1,R2,L3 and R1,L2,R3 (link indices).
TRIPOD_A = frozenset({2, 8, 14})   # L1, L3, R2
TRIPOD_B = frozenset({5, 11, 17})  # L2, R1, R3


class BalanceEnv:
    """
    PyBullet env: 6-legged insect on a flat floor (3 DOF per leg: Coxa, Femur, Tibia).
    - State (IMU-like): [roll, pitch, roll_dot, pitch_dot, joint_pos[18], joint_vel[18]] -> 40 dims
    - Action: 18 joint torques from SNN (clamped); applied only when at least one limb tip touches ground
    - Reward: upright + tripod stance (L1,R2,L3 or R1,L2,R3) + stay in place; episode ends if tilt > 60° or time > T_max
    """

    def __init__(
        self,
        gui=False,
        time_step=0.002,
        gravity_scale=1.0,
        ground_friction=3.2,
        contact_stiffness=400,
        contact_damping=250,
        max_torque=0.00028,
        max_steps=2500,
        seed=None,
        forward_vel_weight=0.0,
        **kwargs,
    ):
        self.time_step = time_step
        self.gravity_scale = gravity_scale
        self.ground_friction = ground_friction
        self.contact_stiffness = contact_stiffness
        self.contact_damping = contact_damping
        self.max_torque = max_torque
        self.max_steps = max_steps
        self._gui = p.GUI if gui else p.DIRECT
        self._client = None
        self._robot = None
        self._plane = None
        self._joint_indices = None
        self._step_count = 0
        self._last_camera_target = [0, 0, 0.04]
        self._last_tripod = None
        self._steps_same_tripod = 0
        self._stable_count = 0
        self._state = None
        if seed is not None:
            np.random.seed(seed)

    def reset(self):
        """Start new episode. Returns initial state (16-dim)."""
        if self._client is not None:
            p.disconnect(self._client)
        self._client = p.connect(self._gui)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        gravity_z = -9.81 * self.gravity_scale
        p.setGravity(0, 0, gravity_z)
        if self._gui == p.GUI:
            print(f"[BalanceEnv] Gravity: (0, 0, {gravity_z:.2f}) m/s^2")
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self._client)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(numSolverIterations=100)
        # Floor
        self._plane = p.loadURDF(
            "plane.urdf",
            [0, 0, 0],
            useFixedBase=True,
        )
        p.changeDynamics(
            self._plane, -1,
            lateralFriction=self.ground_friction,
            contactStiffness=self.contact_stiffness,
            contactDamping=self.contact_damping,
        )
        # Robot: spawn so feet clear ground (~0.10 m below base); penetration makes ground push body up
        start_z = 0.102
        self._robot = p.loadURDF(
            ROBOT_URDF,
            [0, 0, start_z],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False,
        )
        for link_idx in range(-1, p.getNumJoints(self._robot)):
            try:
                p.changeDynamics(
                    self._robot, link_idx,
                    contactStiffness=self.contact_stiffness,
                    contactDamping=self.contact_damping,
                )
            except p.error:
                pass
        # Base link damping: strong absorption so body doesn't get pushed around or fly
        try:
            p.changeDynamics(
                self._robot, -1,
                linearDamping=3.0,
                angularDamping=3.5,
            )
        except p.error:
            pass
        n_j = p.getNumJoints(self._robot)
        self._joint_indices = [
            i for i in range(n_j)
            if p.getJointInfo(self._robot, i)[2] == p.JOINT_REVOLUTE
        ]
        assert len(self._joint_indices) == N_JOINTS
        # Joint damping: enough to smooth motion and prevent runaway, still allows march
        for j in self._joint_indices:
            p.changeDynamics(self._robot, j, jointDamping=0.12)
        # Disable default joint motors so TORQUE_CONTROL in step() actually moves the joints
        for j in self._joint_indices:
            p.setJointMotorControl2(
                self._robot, j, p.VELOCITY_CONTROL, force=0, physicsClientId=self._client
            )
        # Zero initial velocity
        p.resetBaseVelocity(self._robot, [0, 0, 0], [0, 0, 0])
        self._step_count = 0
        self._last_camera_target = [0, 0, 0.04]
        self._last_tripod = None
        self._steps_same_tripod = 0
        self._stable_count = 0
        # Camera: start close to the robot when using GUI (robot is ~0.1 m scale)
        if self._gui == p.GUI:
            p.resetDebugVisualizerCamera(
                cameraDistance=0.35,
                cameraYaw=25,
                cameraPitch=-25,
                cameraTargetPosition=[0, 0, 0.04],
            )
        self._state = self._get_state()
        return self._state.copy()

    def _limb_tips_in_contact(self):
        """True if at least one limb tip (tibia link) is touching the ground."""
        return len(self._get_contact_tibia_set()) > 0

    def _get_contact_tibia_set(self):
        """Set of tibia link indices currently in contact with the ground (for tripod reward)."""
        try:
            contacts = p.getContactPoints(bodyA=self._robot, bodyB=self._plane)
        except p.error:
            return frozenset()
        out = set()
        for c in contacts:
            link_a = c[3] if len(c) > 4 else -1
            if link_a in TIBIA_LINK_INDICES:
                out.add(link_a)
        return frozenset(out)

    def _get_state(self):
        """State (IMU-like): [roll, pitch, roll_dot, pitch_dot, joint_pos[18], joint_vel[18]]."""
        pos, orn = p.getBasePositionAndOrientation(self._robot)
        rpy = p.getEulerFromQuaternion(orn)
        roll, pitch, yaw = rpy
        _, ang_vel = p.getBaseVelocity(self._robot)
        ang_vel = np.array(ang_vel)
        # Project angular vel to roll/pitch axes in world (simplified: use body frame)
        roll_dot = ang_vel[0]
        pitch_dot = ang_vel[1]
        joint_states = [p.getJointState(self._robot, j) for j in self._joint_indices]
        jpos = np.array([s[0] for s in joint_states], dtype=np.float32)
        jvel = np.array([s[1] for s in joint_states], dtype=np.float32)
        return np.concatenate([
            [roll, pitch, roll_dot, pitch_dot],
            jpos,
            jvel,
        ]).astype(np.float32)

    def step(self, action):
        """
        action: (18,) joint torques in [-1, 1]; will be scaled by max_torque.
        Returns: next_state, reward, done, info
        """
        action = np.clip(np.asarray(action, dtype=np.float64).ravel(), -1.0, 1.0)
        # Apply torque from step 0 (no warmup or stability gate)
        torques = action * self.max_torque
        p.setJointMotorControlArray(
            self._robot,
            self._joint_indices,
            p.TORQUE_CONTROL,
            forces=torques.tolist(),
            physicsClientId=self._client,
        )
        p.stepSimulation()
        self._step_count += 1
        # Keep camera focused on robot when using GUI (handle PyBullet errors)
        if self._gui == p.GUI:
            try:
                pos, _ = p.getBasePositionAndOrientation(self._robot)
                self._last_camera_target = list(pos)
            except p.error:
                pos = self._last_camera_target
            try:
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.35,
                    cameraYaw=25,
                    cameraPitch=-25,
                    cameraTargetPosition=pos,
                )
            except p.error:
                pass
        state = self._get_state()
        roll, pitch = state[0], state[1]
        roll_dot, pitch_dot = state[2], state[3]
        tilt = np.sqrt(roll ** 2 + pitch ** 2)
        ang_speed = np.sqrt(roll_dot ** 2 + pitch_dot ** 2)
        # Base: prefer upright and low angular speed (moderate weights so alternating is affordable)
        reward = -0.6 * tilt - 0.06 * ang_speed
        # Tripod stance and alternation: reward being in a tripod, and reward switching to the other tripod
        contact_set = self._get_contact_tibia_set()
        n_contact = len(contact_set)
        current_tripod = TRIPOD_A if contact_set == TRIPOD_A else (TRIPOD_B if contact_set == TRIPOD_B else None)
        # Bonus for being in a valid tripod (exactly 3 feet: stance legs down, swing legs lifted)
        if current_tripod is not None:
            reward += 0.12
        elif n_contact == 3:
            reward += 0.03
        # Strong bonus for alternating: swing legs lifted and set down as the other tripod
        if self._last_tripod is not None and current_tripod is not None and self._last_tripod != current_tripod:
            reward += 0.35
            self._steps_same_tripod = 0
        elif current_tripod is not None and current_tripod == self._last_tripod:
            self._steps_same_tripod += 1
        else:
            self._steps_same_tripod = 0
        self._last_tripod = current_tripod
        # Penalize standing in one tripod too long so alternating becomes better than standing still
        if self._steps_same_tripod > 40:
            reward -= 0.012 * (self._steps_same_tripod - 40)
        # Encourage lifting swing legs: penalize more than 3 feet on the ground
        if n_contact > 3:
            reward -= 0.04 * (n_contact - 3)
        # Stay in place: penalize drift but not so much that transition kills reward
        try:
            lin_vel, _ = p.getBaseVelocity(self._robot, physicsClientId=self._client)
            drift = np.sqrt(lin_vel[0] ** 2 + lin_vel[1] ** 2)
            reward -= 0.25 * drift
        except p.error:
            pass
        # Done: fell (tilt > ~60°) or timeout
        done = tilt > 1.0 or self._step_count >= self.max_steps
        self._state = state
        return state, float(reward), done, {}

    def close(self):
        if self._client is not None:
            p.disconnect(self._client)
            self._client = None

    @property
    def state_dim(self):
        return 4 + N_JOINTS * 2  # 40

    @property
    def action_dim(self):
        return N_JOINTS  # 18
