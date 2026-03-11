# Installing NeuroMechFly

> Modified from [NeuroMechFly](https://github.com/NeLy-EPFL/NeuroMechFly) (Apache 2.0).

**Prerequisites:** Git, conda (or virtualenv). For Apple Silicon (M1/M2): arm64 conda environment.

See [README.md](../README.md) for project overview and [QUICKSTART.md](../QUICKSTART.md) for common commands. We recommend using a virtual environment to avoid conflicts with your Python setup.

---

## Quick install (Linux/macOS)

**Recommended Python:** 3.8 (3.6–3.8 supported; 3.8 works best on Apple Silicon).

**Important on macOS:** PyBullet must be installed from a **binary** (conda or pip wheel), not built from source. Install it **before** `pip install -e .`:

```bash
cd NeuroMechFly
conda create -n neuromechfly python=3.8 numpy Cython shapely
conda activate neuromechfly
pip install git+https://gitlab.com/FARMSIM/farms_container.git
# Install pybullet from conda (binary) so it is not built from source
conda install -c conda-forge -y pybullet
conda install -y pytables matplotlib networkx scipy pillow pyyaml trimesh
pip install treelib jmetalpy scikit-posthocs df3dpostprocessing==1.1.0
pip install -e farms_network
pip install -e .
```

If `conda install -c conda-forge pybullet` fails (no wheel for your platform), run:  
`python -m pip install pybullet --only-binary :all:`  
then continue with the remaining steps.

**Alternatively**, from repo root with conda env active:  
`bash scripts/install_macos.sh`  
(This script installs pybullet from binary first, then the rest.)

**Note:** PyBullet is not in pip’s `install_requires` on purpose for macOS: install it with conda to avoid building from source. The package still requires PyBullet at runtime for simulation.

**Apple Silicon:** If you get an import error for `bullet_sensors`, rebuild for arm64:

```bash
python setup.py build_ext --inplace
```

**Microsoft Windows:** Microsoft Visual Studio C++ 14.0 or newer is required for `farms_container`.

### Troubleshooting

**"Wrong Python" when `pip install` runs:** If you see paths like `/opt/homebrew/` or `python3.10` while your prompt shows `(neuromechfly38)`, your shell is using the system Python instead of the conda env. Use the env’s Python explicitly:

```bash
conda activate neuromechfly38
which python   # should be .../miniforge3/envs/neuromechfly38/bin/python
python -m pip install -e ".[test]"
python -m pytest tests/ -v
```

**"mach-o file, but is an incompatible architecture (have 'x86_64', need 'arm64')":** On Apple Silicon, `farms_container` (or another dep) was installed for x86_64. Use the **conda env’s** pip so everything is built for arm64:

```bash
conda activate neuromechfly38
python -m pip uninstall farms_container -y
python -m pip install git+https://gitlab.com/FARMSIM/farms_container.git
# then from NeuroMechFly/
python -m pip install -e ".[test]"
```

**"Failed building wheel for pybullet" / clang exit code 1:** PyBullet is building Bullet from source and the build fails (common on macOS with newer Xcode/clang). Install a **pre-built** PyBullet instead of building from source:

```bash
conda activate neuromechfly38
# Prefer conda-forge binary (avoids building Bullet)
conda install -c conda-forge pybullet
# Then install the rest (pip will see pybullet already satisfied)
cd NeuroMechFly
python -m pip install -e ".[test]"
```

If `conda install -c conda-forge pybullet` fails (e.g. no wheel for your Python), try forcing pip to use only binaries (no source build): `python -m pip install pybullet --only-binary :all:` then `python -m pip install -e ".[test]"`.

---

## Step-by-step

1. **Install Git LFS** (required for meshes): [git-lfs](https://github.com/git-lfs/git-lfs). Use `git lfs clone` if you need the Blender model.

2. **Clone and enter the repo:**
   ```bash
   git clone <this-repo-url>
   cd NeuroMechFly
   ```

3. **Create and activate conda env:**
   ```bash
   conda create -n neuromechfly python=3.8 numpy Cython shapely
   conda activate neuromechfly
   ```

4. **Install FARMS Container:**
   ```bash
   pip install git+https://gitlab.com/FARMSIM/farms_container.git
   ```

5. **Install remaining dependencies:**
   ```bash
   conda install -y pybullet pytables matplotlib networkx scipy pillow pyyaml trimesh
   pip install treelib jmetalpy scikit-posthocs df3dpostprocessing==1.1.0
   pip install -e farms_network
   pip install -e .
   ```

6. **Verify:** `run_kinematic_replay -b walking --headless --duration 1`

---

## Running tests

Install the test extra and run pytest (use `python -m pytest` so the active env’s Python is used):

```bash
pip install -e ".[test]"
python -m pytest tests/ -v
```

One integration test is skipped if PyBullet or the data directory is missing; all other tests run without simulation.

---

## Using virtualenv

To use `virtualenv` instead of conda, create and activate your environment, then follow the same `pip install` and `conda install` steps. For conda packages (pybullet, etc.), use `pip install pybullet` if needed.

---

## Activating the environment

Each time you use NeuroMechFly, activate the environment:

```bash
conda activate neuromechfly
```
