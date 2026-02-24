#!/bin/bash
# One-time setup for NeuroMechFly on Monsoon HPC
# Run interactively: bash scripts/setup_monsoon_env.sh

set -e
module load mambaforge

# Create env with numpy/Cython first (setup.py needs these), or add to existing
if conda env list | grep -q "neuromechfly38"; then
  echo "Env neuromechfly38 exists. Installing numpy, Cython, shapely..."
  conda install -n neuromechfly38 numpy Cython shapely -y
else
  conda create -n neuromechfly38 python=3.8 numpy Cython shapely -y
fi
source $(conda info --base)/etc/profile.d/conda.sh
conda activate neuromechfly38

cd ~/NeuroMechFly

# Core deps (farms_container needed before pip install -e .)
pip install git+https://gitlab.com/FARMSIM/farms_container.git
conda install -y pybullet pytables matplotlib networkx scipy pillow pyyaml trimesh
pip install treelib jmetalpy scikit-posthocs "df3dpostprocessing==1.1.0"

# NeuroMechFly (farms_network etc. from setup.py install_requires)
pip install -e .

# Rebuild bullet_sensors for Linux
python setup.py build_ext --inplace

echo "=== Setup complete. Run: sbatch --export=CONDA_ENV=neuromechfly38 scripts/neuromuscular_optimization/run_training_monsoon.slurm ==="
