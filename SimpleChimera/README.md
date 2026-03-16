# SimpleChimera: 6-Legged Insect Balance & Walking

Minimal 6-legged insect in **MuJoCo** (or PyBullet): train an SNN policy to balance and walk, then run it with the saved policy. No preprogrammed gait—the policy learns to step (forward velocity + foot clearance rewards).

## Setup

```bash
cd Chimera/SimpleChimera
conda create -n neuromechfly python=3.10 -y
conda activate neuromechfly
pip install -r requirements.txt
```

For **MuJoCo** (recommended): `pip install mujoco`.  
For **PyBullet** fallback: `pip install pybullet` (or `conda install -c conda-forge pybullet` on macOS).

## Train (headless)

```bash
python train.py --steps 2000 --pop 20 --gen 10
```

Saves the best policy to `best_policy.npz`. Options: `--forward-vel`, `--foot-clearance`, `--max-torque`, `--no-cpg`, `--sigma`, `--out`.

## Run with GUI

**MuJoCo** (preferred; same env as training):

```bash
# On macOS: use mjpython so the viewer works
mjpython run.py --policy best_policy.npz --gui
```

On Linux you can use `python run.py --policy best_policy.npz --gui` if MuJoCo is installed.

Options: `--smooth`, `--torque_scale`, `--max_torque`, `--no-cpg`.

Press Enter in the terminal to close the simulation.

## Design

- **Robot**: 18 revolute joints (3 per leg: Coxa, Femur, Tibia), L1–L3, R1–R3. MuJoCo model: `robot/six_leg_insect.xml`.
- **Environment**: MuJoCo (or PyBullet). State = 40 (roll, pitch, rates, 18 joint positions, 18 joint velocities). Action = 18 torques in [-1, 1] × `max_torque`. Reward = balance + forward velocity + foot clearance (stepping) + straight walking (lateral/yaw penalty).
- **Controller**: Evolved SNN + optional CPG. Train with `train.py`; run loads `best_policy.npz` only (no fixed rhythm mode).

See **[HOW_IT_WORKS.md](HOW_IT_WORKS.md)** for data flow, joint order, and optional C++ build (Kilowatt/MuJoCo).
