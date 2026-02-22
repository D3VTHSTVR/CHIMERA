#!/bin/bash
#SBATCH --job-name=nmf-optim
#SBATCH --output=optimization_%j.log
#SBATCH --error=optimization_%j.err
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
cd ~/NeuroMechFly

# Load conda (required on Monsoon - conda not in default PATH for batch jobs)
module load mambaforge 2>/dev/null || module load anaconda3 2>/dev/null || true
source $(conda info --base)/etc/profile.d/conda.sh

# Activate env (override with sbatch --export=CONDA_ENV=your_env,...)
CONDA_ENV=${CONDA_ENV:-neuromechfly38}
conda activate $CONDA_ENV || {
  echo "ERROR: Conda env '$CONDA_ENV' not found. Run 'conda env list' on Monsoon."
  echo "Then: sbatch --export=CONDA_ENV=your_env_name run_training_monsoon.slurm"
  exit 1
}

# Training params (override via sbatch --export=POP=100,GEN=60)
POP=${POP:-100}
GEN=${GEN:-60}
PROCESS=${PROCESS:-16}
GROUND=${GROUND:-floor}

echo "=== Starting optimization: pop=${POP} gen=${GEN} ground=${GROUND} ==="
echo "Started at $(date)"

# Run optimization
run_multiobj_optimization --pop $POP --gen $GEN --process $PROCESS --ground $GROUND

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
