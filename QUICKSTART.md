# NeuroMechFly – What you can run

Activate the env first: `conda activate neuromechfly` (or your env name).

---

## Run faster (kinematic replay)

- **No GUI (fastest):**  
  `run_kinematic_replay -b walking --headless`
- **Short run (e.g. 1 second):**  
  `run_kinematic_replay -b walking --duration 1`
- **Combine both:**  
  `run_kinematic_replay -b walking --headless --duration 2`
- **With GUI but fewer frames:**  
  `run_kinematic_replay -b walking --sim_speed 20`

---

## 1. Kinematic replay (ball treadmill)

- **Walking on ball:**  
  `run_kinematic_replay -b walking`
- **Grooming on ball:**  
  `run_kinematic_replay -b grooming`
- **Different fly (1–3):**  
  `run_kinematic_replay -b walking -fly 2`
- **Record video:**  
  `run_kinematic_replay -b walking --record`
- **Show collisions (green):**  
  `run_kinematic_replay -b walking --show_collisions`

---

## 2. Kinematic replay on ground

- **Walking on floor:**  
  `run_kinematic_replay_ground`
- **Grooming:**  
  `run_kinematic_replay_ground -b grooming`
- **With perturbations:**  
  `run_kinematic_replay_ground --perturbation`
- **Record / collisions:**  
  `run_kinematic_replay_ground --record` or `--show_collisions`

---

## 3. Morphology experiment

- **Default (nmf)::**  
  `run_morphology_experiment`
- **Other morphologies:**  
  `run_morphology_experiment --model stick_legs`  
  `run_morphology_experiment --model stick_legs_antennae`
- **Record:**  
  `run_morphology_experiment --record`

---

## 4. Gait optimization (neuromuscular)

- **Visualize a pre-run solution (with GUI):**  
  `run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59`
- **With plots:**  
  `run_neuromuscular_control --gui -p optimization_results/run_Drosophila_example/ -g 59 --plot`
- **Run optimization from scratch (slow, many generations):**  
  `run_multiobj_optimization`
- **Short floor run:**  
  `run_multiobj_optimization --pop 10 --gen 5 --process 4 --ground floor`
- **Two-phase training (recommended for floor):**  
  1. Phase 1 – stability:  
     `run_stability_optimization --pop 20 --gen 15 --process 4 --ground floor`  
  2. Phase 2 – walking (warm start from Phase 1):  
     `run_multiobj_optimization --pop 20 --gen 50 --process 4 --ground floor --warm-start optimization_results/run_DrosophilaStability_var_63_obj_2_pop_20_gen_15_YYMMDD_HHMMSS`
- **Analyze optimization:**  
  `run_optimization_analysis`

---

## 5. Sensitivity analysis

- **Run analysis (needs data in `data/sensitivity_analysis`):**  
  `run_sensitivity_analysis`
- **Grid search:**  
  `run_grid_search`

---

## 6. Other

- **CPG network (no simulation):**  
  `cd data/locomotion_network && python locomotion.py`

Results from kinematic replays are written under  
`scripts/kinematic_replay/simulation_results/`.
