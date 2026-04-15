#!/bin/bash 
#SBATCH --job-name=JOBNAME
#SBATCH -t HOURS:MINUTES:00
#SBATCH --array=ARRAY_START-ARRAY_END
#SBATCH -c 1
#SBATCH --mem=2G
#SBATCH -e ./logs/error.%A-%a.out 
#SBATCH -o ./logs/output.%A-%a.out

module load CMake/3.24.3-GCCcore-12.2.0
module load FreeImage/3.18.0-GCCcore-12.2.0
module load NLopt/2.7.1-GCCcore-12.2.0

ENERGY_TYPE=ENERGY_TYPE_PLACEHOLDER TAU_CHARGER_CAPACITY=TAU_CHARGER_CAPACITY_PLACEHOLDER ZETA_NUM_WORKERS=ZETA_NUM_WORKERS_PLACEHOLDER ETA_WORK_ENERGY_RATE=ETA_WORK_ENERGY_RATE_PLACEHOLDER DELTA_COMMUTE=DELTA_COMMUTE_PLACEHOLDER /users/co1gm/swarm-energy-replenishment/experiments/work_and_charge/run_simulation.sh
