#!/usr/bin/env bash
# Install NeuroMechFly on macOS (including Apple Silicon).
# Run from repo root: bash scripts/install_macos.sh
# Requires: conda activated (e.g. conda activate neuromechfly38)

set -e
cd "$(dirname "$0")/.."

echo "Installing PyBullet from binary (avoids build failure on macOS)..."
if conda install -c conda-forge -y pybullet 2>/dev/null; then
  echo "PyBullet installed via conda-forge."
else
  echo "Conda install failed, trying pip --only-binary..."
  python -m pip install pybullet --only-binary :all
fi

echo "Installing remaining dependencies..."
python -m pip install git+https://gitlab.com/FARMSIM/farms_container.git
conda install -y pytables matplotlib networkx scipy pillow pyyaml trimesh 2>/dev/null || \
  python -m pip install pytables matplotlib networkx scipy pillow pyyaml trimesh
python -m pip install treelib jmetalpy scikit-posthocs df3dpostprocessing==1.1.0
python -m pip install -e farms_network
python -m pip install -e ".[test]"

echo "Checking for bullet_sensors (Apple Silicon)..."
python -c "from NeuroMechFly.simulation.bullet_sensors import JointSensors" 2>/dev/null || {
  echo "Rebuilding bullet_sensors for arm64..."
  python setup.py build_ext --inplace
}

echo "Done. Run tests with: python -m pytest tests/ -v"
