# How SimpleChimera Works

Minimal 6-legged insect balance and walking: train an SNN policy, then run it in simulation. Optional C++ build provides the same controller and MuJoCo run without Python.

## Overview

1. **Robot**: 18 revolute joints (3 per leg: Coxa, Femur, Tibia), L1–L3, R1–R3.
2. **Simulation**: MuJoCo (preferred) or PyBullet. State = roll, pitch, angular rates + 18 joint positions + 18 joint velocities (40 dims). Action = 18 joint torques in [-1, 1], scaled by `max_torque`.
3. **Controller**: Evolved SNN + optional CPG (tripod rhythm). Train with `train.py`; run with `run.py` and a saved policy (`best_policy.npz`).

## Data flow (Python)

- **train.py**: Evolution strategy. Reward = balance (low tilt) + forward velocity + foot clearance (feet in air = stepping) + straight walking (lateral/yaw penalty). Saves best params to `best_policy.npz`.
- **run.py**: Loads policy, runs env (MuJoCo or PyBullet), steps with `policy.act(state)`, applies action (with optional smoothing). On macOS with GUI use: `mjpython run.py --policy best_policy.npz --gui`.

### Joint order (everywhere)

L1_coxa, L1_femur, L1_tibia, L2_..., L3_..., R1_..., R2_..., R3_... (18 total).

## Running (Python)

- **Train** (evolve SNN, with or without CPG):
  ```bash
  python train.py --steps 2000 --pop 20 --gen 10
  ```
  Saves `best_policy.npz`. Options: `--no-cpg`, `--forward-vel`, `--foot-clearance`, `--max-torque`, `--sigma`.

- **Run with GUI** (MuJoCo; on macOS use `mjpython`):
  ```bash
  mjpython run.py --policy best_policy.npz --gui
  ```
  Options: `--smooth`, `--torque_scale`, `--max_torque`.

## C++ (optional)

The C++ build provides a Kilowatt-style 2-neuron SNN chain (same logic as `Chimera/Kilowatt`) and MuJoCo/Bullet runners. Useful for deployment or running without Python.

- **Build**: see `cpp/README.md`. You get `libsimplechimera.a`, `kilowatt_standalone`, `run_mujoco`, `run_simulation`.
- **run_mujoco**: MuJoCo simulation + Kilowatt controller (no training).
- **run_simulation**: Bullet simulation + Kilowatt controller.

Training remains in Python (`train.py`); C++ is for running a fixed (Kilowatt) or exported controller.

## Summary

| Component     | Role |
|--------------|------|
| **BalanceEnv** | MuJoCo or PyBullet: 40-dim state, 18-dim action, reward = balance + forward vel + foot clearance + straight. |
| **SNN + CPG**  | Evolved policy; train with `train.py`, run with `run.py --policy best_policy.npz`. |
| **C++ Kilowatt** | Fixed 3×2-neuron chain; use `run_mujoco` or `run_simulation` for Python-free runs. |
