#!/usr/bin/env bash
# Run the balance sim with GUI on Mac. Uses neuromechfly38 so pybullet is found.
# Usage: ./run_gui.sh
#        ./run_gui.sh --torque_scale 0.7
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
# Use conda env so we get the Python that has pybullet (avoids wrong python in PATH)
exec conda run -n neuromechfly38 python run.py --gui "$@"
