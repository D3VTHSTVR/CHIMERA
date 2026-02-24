#!/bin/bash
#SBATCH --job-name=nmf-optim
#SBATCH --output=optimization_%j.log
#SBATCH --error=optimization_%j.log
#SBATCH --time=24:00:00
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --partition=core
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=vr527@nau.edu

# NeuroMechFly gait optimization on Monsoon HPC
# Submit with: sbatch run_training_monsoon.slurm
# Or with custom args: sbatch --export=POP=100,GEN=60 run_training_monsoon.slurm

set -e
exec 2>&1
echo "=== Job started $(date) ==="

cd ~/NeuroMechFly
echo "=== CWD: $(pwd) ==="

# Pull latest (optional - comment out if no network in job)
git fetch origin 2>/dev/null || true
git checkout monsoon 2>/dev/null || true
git pull origin monsoon 2>/dev/null || true

# Load conda - batch jobs don't source .bashrc, so we must init conda manually
echo "=== Loading conda ==="
CONDA_LOADED=0
if module load mambaforge 2>/dev/null; then
  source $(conda info --base)/etc/profile.d/conda.sh
  CONDA_LOADED=1
  echo "Loaded mambaforge module"
fi
if [ $CONDA_LOADED -eq 0 ] && module load anaconda3 2>/dev/null; then
  source $(conda info --base)/etc/profile.d/conda.sh
  CONDA_LOADED=1
  echo "Loaded anaconda3 module"
fi
if [ $CONDA_LOADED -eq 0 ] && [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  CONDA_LOADED=1
  echo "Using miniforge3 from home"
fi
if [ $CONDA_LOADED -eq 0 ] && [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  CONDA_LOADED=1
  echo "Using miniconda3 from home"
fi
if [ $CONDA_LOADED -eq 0 ]; then
  echo "ERROR: Could not find conda. Try: module load mambaforge (or anaconda3)"
  exit 1
fi

# Activate env (override with sbatch --export=CONDA_ENV=your_env,...)
# On Monsoon, only 'base' exists by default. Create neuromechfly38 first for isolation.
CONDA_ENV=${CONDA_ENV:-base}
echo "=== Activating $CONDA_ENV ==="
conda activate $CONDA_ENV || {
  echo "ERROR: Conda env '$CONDA_ENV' not found."
  echo "Available envs:"
  conda env list
  exit 1
}
echo "=== Python: $(which python) ==="

# Install NeuroMechFly if not present (needed when using base env)
python -c "import NeuroMechFly" 2>/dev/null || {
  echo "=== Installing NeuroMechFly (pip install -e .) ==="
  pip install -e . --quiet
}

# Training params (override via sbatch --export=POP=100,GEN=60)
POP=${POP:-100}
GEN=${GEN:-60}
PROCESS=${PROCESS:-16}
GROUND=${GROUND:-floor}

echo "=== Starting optimization: pop=${POP} gen=${GEN} ground=${GROUND} ==="
echo "Started at $(date)"

# Run optimization (use script directly if not in PATH)
cd ~/NeuroMechFly
if command -v run_multiobj_optimization &>/dev/null; then
  run_multiobj_optimization --pop $POP --gen $GEN --process $PROCESS --ground $GROUND
else
  echo "run_multiobj_optimization not in PATH, using python script"
  python scripts/neuromuscular_optimization/run_multiobj_optimization --pop $POP --gen $GEN --process $PROCESS --ground $GROUND
fi

EXIT_CODE=$?
echo "Finished at $(date) with exit code $EXIT_CODE"

if [ $EXIT_CODE -eq 0 ]; then
  # Record completion and push
  RUN_DIR=$(ls -td scripts/neuromuscular_optimization/optimization_results/run_Drosophila_* 2>/dev/null | head -1)
  if [ -n "$RUN_DIR" ]; then
    echo "Successful run: $RUN_DIR" >> scripts/neuromuscular_optimization/TRAINING_LOG.txt
    echo "  pop=$POP gen=$GEN ground=$GROUND completed $(date)" >> scripts/neuromuscular_optimization/TRAINING_LOG.txt
    git add scripts/neuromuscular_optimization/TRAINING_LOG.txt
    git commit -m "Training completed: pop=$POP gen=$GEN $GROUND ($(date +%Y-%m-%d))" || true
    git push origin monsoon || echo "Push failed - run 'git push origin monsoon' manually"
  fi
fi

exit $EXIT_CODE
