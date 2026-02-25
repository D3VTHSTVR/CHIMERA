# NeuroMechFly: How the Fly Moves, Learns, and CLI Reference

## 1. How the Fly Moves (Control Architecture)

The fly uses a **neuromuscular** control pipeline inspired by insect biology:

```
CPG (Central Pattern Generator)  →  Motor Neuron Activity  →  Muscles  →  Joint Torques  →  Movement
```

### Step 1: Central Pattern Generator (CPG)

- **What:** A network of coupled oscillators (one per joint, per leg). Each joint has **flexion** and **extension** oscillators.
- **Where:** `data/config/network/locomotion_network.graphml`, created by `data/locomotion_network/locomotion.py`
- **How:** Oscillators produce sine-like phase outputs. Oscillators are coupled:
  - **Within a leg:** Coxa → Femur → Tibia (phase offset ~π/2)
  - **Between legs:** Tripod gait (LF–RF, LM–RH, LH–RM alternate)
- **Output:** Phase values (e.g. `phase_joint_LFCoxa_flexion`) that oscillate over time.

### Step 2: Motor Neuron Activity → Muscles

- **What:** The CPG phases are treated as **motor neuron activity**. The muscle model converts these into **activation** (flexor vs extensor).
- **Muscle model:** Ekeberg-style spring–damper antagonist muscles (see `NeuroMechFly/control/spring_damper_muscles.py`):
  - **Active torque:** `α × (flexor_act − extensor_act)` + stiffness term
  - **Passive torque:** Passive stiffness + damping
  - **Activation:** `flexor_act = amp × (1 + sin(phase))`

### Step 3: Joint Torques → Physics

- **What:** Torques are applied to Coxa, Femur, Tibia (and Coxa_roll for middle/hind legs) via PyBullet’s torque control.
- **Result:** Legs move the fly on a ball treadmill or on the floor.

**Summary:** The fly uses CPG “nerves” (oscillators) to drive muscle activations, which produce torques and joint motion. No sensory feedback or learning in the loop.

---

## 2. How Learning (Training) Works

There is **no** online learning or reinforcement learning. The system uses **evolutionary optimization** (meta‑learning).

### Algorithm: NSGA-II

- **Type:** Multi-objective genetic algorithm (GA)
- **Population:** Many candidate parameter sets (CPG + muscle parameters)
- **Each candidate:** Simulated for 2 seconds; objectives and penalties are computed.

### What Is Optimized (63 Variables)

| Variable | Count | Description |
|----------|-------|-------------|
| CPG frequency | 1 | Oscillation frequency (6–10 Hz) |
| Muscle gain (α, β, γ, δ, rest_pos) | 45 | 5 per joint × 9 joints (left side; right is symmetric) |
| Phase relationships | 17 | 12 intraleg + 5 interleg |

### Objectives (Both Minimized)

1. **Distance:** Negative = bad. Forward distance = ball rotation (or base displacement on floor).
2. **Stability:** Static stability metric.

### Penalties (Added to Objectives)

- **Duty factor:** Stance phase between 40–90%
- **Lava:** Moving boundary (speed limits)
- **Velocity:** Joint angular velocity limits
- **Joint limits:** Range-of-motion limits

### Process

1. **Create population** of random parameter sets.
2. **Evaluate** each candidate in simulation (headless).
3. **Select** non-dominated solutions.
4. **Crossover** and **mutate** to create offspring.
5. **Repeat** for many generations.

**Summary:** “Learning” is genetic search over CPG and muscle parameters. Better solutions are selected and evolved; no neural networks inside the simulation are trained.

---

## 3. Immediate Next Steps

| Priority | Action |
|----------|--------|
| 1 | Use pre-trained example: `run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59` |
| 2 | Run longer optimization: `run_multiobj_optimization --pop 50 --gen 20 --process 4` |
| 3 | Run full optimization: `run_multiobj_optimization --pop 200 --gen 60 --process 8` |
| 4 | Analyze results: `run_optimization_analysis --plot -p optimization_results/run_Drosophila_example/` |
| 5 | Record a video: `run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59 --record` |
| 6 | Inspect gait: `run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59 --plot` |

---

## 4. CLI Reference

### `run_neuromuscular_control`

Run a trained neuromuscular controller with a chosen solution.

**Syntax:**
```bash
run_neuromuscular_control [OPTIONS]
```

**Options:**

| Option | Short | Type | Default | Description |
|--------|-------|------|---------|-------------|
| `--path` | `-p` | str | `''` | Path to optimization results (relative to `scripts/neuromuscular_optimization`). Example: `optimization_results/run_Drosophila_example/` |
| `--gen` | `-g` | str | `'txt'` | Generation number. Use `59` for gen 59, or `'txt'` for FUN.txt/VAR.txt. |
| `--sol` | `-s` | str | `'tradeoff'` | Solution type: `fastest`, `tradeoff`, `most_stable`, or an integer index. |
| `--gui` | | flag | False | Show PyBullet GUI. |
| `--record` | | flag | False | Record simulation as video. |
| `--plot` | | flag | False | Plot Pareto front and gait diagram after run. |
| `--slow` | | flag | False | Slow down simulation for viewing. |
| `--log_penalties` | | flag | False | Write penalties to PENALTIES.<gen>. |
| `--profile` | | flag | False | Profile performance. |
| `--solver_iterations` | | int | 100 | PyBullet physics solver iterations per step. |
| `--ground` | | str | `ball` | `ball` or `floor`. Must match the optimization run. |

**Examples:**
```bash
run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59
run_neuromuscular_control --gui -p optimization_results/run_DrosophilaEvolution_var_63_obj_2_pop_10_gen_5_YYMMDD_HHMMSS -g 1 --ground floor
run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59 -s fastest --record
run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59 --plot
```

---

### `run_multiobj_optimization`

Run NSGA-II multi-objective optimization.

**Syntax:**
```bash
run_multiobj_optimization [OPTIONS]
```

**Options:**

| Option | Short | Type | Default | Description |
|--------|-------|------|---------|-------------|
| `--pop` | `-p` | int | 200 | Population size per generation. |
| `--gen` | `-g` | int | 60 | Number of generations. |
| `--process` | `-n` | int | 8 | Number of parallel workers. |
| `--ground` | | str | `ball` | `ball` or `floor`. Floor uses free support joints. |
| `--controller` | | str | `cpg` | `cpg` or `snn`. |
| `--warm-start` | | str | None | Path to Phase 1 results to seed initial population. |

**Total evaluations:** `pop × gen` (e.g. 200×60 = 12,000).

**Examples:**
```bash
run_multiobj_optimization --pop 10 --gen 5 --process 4
run_multiobj_optimization --pop 200 --gen 60 --process 8
run_multiobj_optimization --pop 20 --gen 50 --ground floor --warm-start optimization_results/run_DrosophilaStability_var_63_obj_2_pop_20_gen_15_YYMMDD_HHMMSS
```

**Output:** `optimization_results/run_DrosophilaEvolution_var_63_obj_2_pop_<pop>_gen_<gen>_<timestamp>/`

---

### `run_stability_optimization`

Phase 1: Stability-only optimization. Use results as warm start for `run_multiobj_optimization`. Defaults: `--pop 20`, `--gen 15`, `--ground floor`.

**Output:** `optimization_results/run_DrosophilaStability_var_63_obj_2_pop_<pop>_gen_<gen>_<timestamp>/`

---

### `run_optimization_analysis`

Analyze and compare optimization runs.

**Syntax:**
```bash
run_optimization_analysis [OPTIONS]
```

**Options:**

| Option | Short | Type | Default | Description |
|--------|-------|------|---------|-------------|
| `--path` | `-p` | str | `''` | Path to optimization results. |
| `--gen` | `-g` | str | `'txt'` | Generation to analyze. |
| `--frequency` | `-f` | int | 5 | Frequency of generation skip for plots. |
| `--gui` | | flag | False | Show GUI when running simulations. |
| `--record` | | flag | False | Record simulations. |
| `--plot` | | flag | False | Generate plots. |
| `--slow` | | flag | False | Slow down simulations. |
| `--log_penalties` | | flag | False | Log penalties. |
| `--profile` | | flag | False | Profile performance. |
| `--ncores` | `-n` | int | 8 | Number of cores for parallel runs. |
| `--solver_iterations` | | int | 100 | PyBullet solver iterations. |

---

## 5. File Layout

```
scripts/neuromuscular_optimization/
├── optimization_results/
│   ├── run_Drosophila_example/     # Pre-trained example
│   │   ├── FUN.0, FUN.1, ... FUN.59   # Objective values per generation
│   │   ├── VAR.0, VAR.1, ... VAR.59   # Parameter values per generation
│   │   └── CONFIG.yaml
│   └── run_Drosophila_var_63_obj_2_pop_10_gen_5_<timestamp>/
│       └── ...
├── simulation_<exp>/               # Dumped simulation data after run
│   └── gen_<gen>/sol_<sol>/
└── run_neuromuscular_control
```

---

## 6. Other Scripts

| Script | Purpose |
|--------|---------|
| `run_kinematic_replay -b walking` | Replay recorded motion (no CPG). |
| `run_kinematic_replay_ground` | Replay on floor. |
| `run_morphology_experiment` | Test different leg/antenna morphologies. |
| `cd data/locomotion_network && python locomotion.py` | Run CPG network alone (no simulation). |
