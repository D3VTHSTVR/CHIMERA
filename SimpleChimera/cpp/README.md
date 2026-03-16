# SimpleChimera C++ (replacing Python)

C++ implementation that can **replace the Python stack**: controller (Kilowatt), simulation (Bullet), and balance env. You can run the full loop in C++ with no Python or PyBullet.

## Build

Run from the **cpp** directory (the one that contains `CMakeLists.txt`):

```bash
cd /path/to/CS485/Chimera/SimpleChimera/cpp
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

**Required for full simulation:** [Bullet](https://github.com/bulletphysics/bullet3) (physics engine). On macOS:

```bash
brew install bullet
```

Then reconfigure and build; the **run_simulation** executable will be built.

**Without Bullet** you still get:

- **libsimplechimera.a** – `TwoMotorSNN` and `KilowattChain`
- **kilowatt_standalone** – reads state (40 doubles, binary) from stdin, writes action (18 doubles) to stdout

**With Bullet** you also get:

- **run_simulation** – full C++ run: Bullet world + 6-legged robot + Kilowatt controller. Replaces `python run.py --controller kilowatt`.
  ```bash
  ./run_simulation --steps 5000 --amplitude 0.5
  ```
  **Show a simulation window:** install GLFW (`brew install glfw`), then reconfigure and rebuild. Run with `--gui`:
  ```bash
  ./run_simulation --gui --steps 5000 --amplitude 0.5
  ```
  Without GLFW, the run is headless (no window; only reward is printed).
  **If the window doesn’t appear:** check behind other windows or the Dock; the build prints a reminder. On macOS the OpenGL window may open in the background.

**Run with MuJoCo (same look as MuJoCo’s viewer):** You can use MuJoCo for physics and rendering instead of Bullet, so the simulation looks like the standard MuJoCo viewer (lighting, shadows, etc.):

1. Install MuJoCo’s Python package (includes the C library and headers):
   ```bash
   pip install mujoco
   ```
2. Configure and build. CMake **auto-detects** MuJoCo from your Python `mujoco` package, so you don’t need to set `MUJOCO_PATH`. If you use a non-pip install, set `MUJOCO_PATH` to the directory that contains `include/mujoco/` and `libmujoco*.dylib`.
   ```bash
   cd build
   rm -f CMakeCache.txt   # only if you previously set MUJOCO_PATH to a wrong path
   cmake .. -DCMAKE_BUILD_TYPE=Release
   cmake --build .
   ```
3. Run (model path is auto-searched; or pass `--model /path/to/six_leg_insect.xml`):
   ```bash
   ./run_mujoco --steps 5000 --amplitude 0.5
   ```
   The MuJoCo window will open and show the same Kilowatt controller driving the 6-legged robot with MuJoCo’s native rendering.

Optional: if **pybind11** is installed, the **kilowatt_cpp** Python extension is built so you can use the C++ controller from Python.

## API (C++)

- **TwoMotorSNN**: two neurons with reciprocal inhibition; `step(m1, m2)` updates and returns motor outputs in [0, 1].
- **KilowattChain**: three `TwoMotorSNN` modules (front, middle, rear) with coupling.
  - `reset()` – call at episode start.
  - `step(state[40], action[18])` – one simulation step; fills `action` with torques in [-1, 1]. `state` is ignored (kept for API compatibility).

Joint order (same as Python BalanceEnv): L1_coxa, L1_femur, L1_tibia, L2_..., L3_..., R1_..., R2_..., R3_... (18 joints).

## Using the standalone from Python

You can drive the C++ controller from the Python env by piping state/action as binary (40 doubles in, 18 doubles out per step). Alternatively use the Python Kilowatt controller in `snn/kilowatt_controller.py`, which mirrors this C++ logic.

## Python bindings (optional)

If built, copy the `.so` (or rename to match your Python) into the SimpleChimera directory and use:

```python
import kilowatt_cpp
import numpy as np
c = kilowatt_cpp.KilowattChain(0.5)
c.reset()
state = np.zeros(40, dtype=np.float64)
action = np.zeros(18, dtype=np.float64)
c.step(state, action)
# action now holds 18 torques in [-1, 1]
```
