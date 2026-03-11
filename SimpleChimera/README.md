# 6-Legged Insect Balance (SNN)

Minimal from-scratch project: a simple 6-legged insect in PyBullet, controlled by a spiking neural network (SNN), trained to **balance** in a tailored environment. No dependency on NeuroMechFly.

## Setup

```bash
cd /Users/devthstvr3/Desktop/SPRING2026/CS485/insect_balance
conda create -n insect_balance python=3.8 -y
conda activate insect_balance
pip install -r requirements.txt
```

If you already have PyBullet in another env (e.g. `neuromechfly38`), you can use that instead:

```bash
conda activate neuromechfly38
# Use that env’s Python when running scripts (see commands below).
```

## Train (headless)

```bash
python train.py --steps 2000 --pop 20 --gen 10
```

Saves the best policy to `best_policy.npz`. Use the **same Python** that has `pybullet` (e.g. from your conda env: `path/to/env/bin/python train.py ...`).

## Run with GUI

```bash
python run.py --policy best_policy.npz --gui
```

Press Enter in the terminal when you want to close the simulation window.

### On macOS (especially Apple Silicon)

PyBullet has no pip wheel for Python 3.8 on arm64, so `pip install pybullet` may try to build from source and fail. Use conda instead:

```bash
conda install -c conda-forge pybullet   # in the env you use for this project
```

If you get `ModuleNotFoundError: No module named 'pybullet'` even after installing, your shell’s `python` may not be from that env. Use either:

- **Launcher script** (recommended):  
  `./run_gui.sh`  
  This runs with `conda run -n neuromechfly38` so the correct Python is used.

- **Or run explicitly**:  
  `conda run -n neuromechfly38 python run.py --gui`

## Design

- **Robot**: 6-legged insect with **3 DOF per leg (Coxa, Femur, Tibia)**, matching the fly model: 18 actuated joints total. URDF in `robot/`.
- **Environment**: Flat floor, soft contacts, optional reduced gravity. Reward = stay upright (penalize tilt and angular velocity).
- **SNN**: Leaky integrate-and-fire (LIF) neurons; state (orientation, angular vel, joint angles) → hidden layer → 12 joint torques. Trained with evolution strategy (no backprop).
- **Training**: Random / evolutionary search over SNN parameters; each candidate runs for a few seconds and gets a balance reward.
