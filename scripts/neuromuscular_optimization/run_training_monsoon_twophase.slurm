#!/bin/bash
#SBATCH --job-name=nmf-twophase
#SBATCH --output=optimization_twophase_%j.log
#SBATCH --error=optimization_twophase_%j.log
#SBATCH --time=48:00:00
#SBATCH --cpus-per-task=16
#SBATCH --mem=32G
#SBATCH --partition=core
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=vr527@nau.edu

# Full two-phase optimization on Monsoon: Phase 1 (stability) then Phase 2 (multiobj) with warm-start.
# Submit and disconnect; job keeps running. Check with: squeue -u $USER
#
# Submit: cd ~/NeuroMechFly && sbatch scripts/neuromuscular_optimization/run_training_monsoon_twophase.slurm
# With custom params: sbatch --export=POP1=20,GEN1=15,POP2=20,GEN2=50 scripts/.../run_training_monsoon_twophase.slurm

set -e
exec 2>&1
echo "=== Two-phase job started $(date) ==="

cd ~/NeuroMechFly
echo "=== CWD: $(pwd) ==="

git fetch origin 2>/dev/null || true
git checkout monsoon 2>/dev/null || true
git pull origin monsoon 2>/dev/null || true

# Load conda
CONDA_LOADED=0
if module load mambaforge 2>/dev/null; then
  source $(conda info --base)/etc/profile.d/conda.sh
  CONDA_LOADED=1
fi
if [ $CONDA_LOADED -eq 0 ] && module load anaconda3 2>/dev/null; then
  source $(conda info --base)/etc/profile.d/conda.sh
  CONDA_LOADED=1
fi
if [ $CONDA_LOADED -eq 0 ] && [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  CONDA_LOADED=1
fi
if [ $CONDA_LOADED -eq 0 ] && [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  CONDA_LOADED=1
fi
if [ $CONDA_LOADED -eq 0 ]; then
  echo "ERROR: Could not find conda."
  exit 1
fi

CONDA_ENV=${CONDA_ENV:-neuromechfly38}
conda activate $CONDA_ENV || { echo "ERROR: Conda env $CONDA_ENV not found."; conda env list; exit 1; }
echo "=== Python: $(which python) ==="

python -c "import NeuroMechFly" 2>/dev/null || { pip install -e . --quiet; }

# Phase 1 and Phase 2 params (monsoon defaults)
POP1=${POP1:-20}
GEN1=${GEN1:-15}
POP2=${POP2:-20}
GEN2=${GEN2:-50}
PROCESS=${PROCESS:-16}
GROUND=${GROUND:-floor}
OPT_DIR="scripts/neuromuscular_optimization"

echo "=== Phase 1: stability  pop=${POP1} gen=${GEN1} ==="
python "$OPT_DIR/run_stability_optimization" --pop $POP1 --gen $GEN1 --process $PROCESS --ground $GROUND
PHASE1_EXIT=$?
if [ $PHASE1_EXIT -ne 0 ]; then
  echo "Phase 1 failed with exit code $PHASE1_EXIT"
  exit $PHASE1_EXIT
fi

# Latest Phase 1 result dir (run_DrosophilaStability_...)
WARM_START=$(ls -td "$OPT_DIR/optimization_results"/run_DrosophilaStability_* 2>/dev/null | head -1)
if [ -z "$WARM_START" ]; then
  echo "ERROR: No Phase 1 result folder found under $OPT_DIR/optimization_results/"
  exit 1
fi
# Path relative to repo for --warm-start (script resolves from scripts/neuromuscular_optimization)
WARM_REL="optimization_results/$(basename "$WARM_START")"
echo "=== Phase 2: multiobj warm-start=$WARM_REL pop=${POP2} gen=${GEN2} ==="
python "$OPT_DIR/run_multiobj_optimization" --pop $POP2 --gen $GEN2 --process $PROCESS --ground $GROUND --warm-start "$WARM_REL"
EXIT_CODE=$?
echo "Finished at $(date) with exit code $EXIT_CODE"

if [ $EXIT_CODE -eq 0 ]; then
  RUN_DIR=$(ls -td "$OPT_DIR/optimization_results"/run_DrosophilaEvolution_* 2>/dev/null | head -1)
  if [ -n "$RUN_DIR" ]; then
    echo "Successful two-phase run. Phase 2: $RUN_DIR" >> "$OPT_DIR/TRAINING_LOG.txt"
    echo "  Phase1: $WARM_START  Phase2 pop=$POP2 gen=$GEN2 completed $(date)" >> "$OPT_DIR/TRAINING_LOG.txt"
    git add "$WARM_START" "$RUN_DIR" "$OPT_DIR/TRAINING_LOG.txt" 2>/dev/null || true
    git commit -m "Two-phase training: Phase2 pop=$POP2 gen=$GEN2 ($(date +%Y-%m-%d))" || true
    git push origin monsoon || echo "Push failed - run 'git push origin monsoon' manually"
    echo "=== Pushed to origin monsoon. Pull locally: git pull origin monsoon ==="
  fi
fi
exit $EXIT_CODE
