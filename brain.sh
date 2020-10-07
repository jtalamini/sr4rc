#!/bin/bash
#SBATCH --time=2-23:00
#SBATCH --partition=slowq

BODY=serialized
ROBOT=0
TASK=locomotion
DIRNAME=sr4rc.$SLURM_ARRAY_JOB_ID.brain_$BODY_$ROBOT_$TASK
mkdir $DIRNAME
module load java/jdk/14.0.2

#evolution
/cm/shared/apps/java/jdk/14.0.2/bin/java -cp brain.jar it.units.erallab.ControllerOptimization randomSeed=$SLURM_ARRAY_TASK_ID bodyType=$BODY robotIndex=$ROBOT taskType=$TASK gridW=5 gridH=4 robotVoxels=20 popSize=1000 iterations=200 births=10000 dir=$DIRNAME statsFile=stats-$SLURM_ARRAY_TASK_ID.txt

# SCHEDULE: sbatch --array=0-9 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt brain.sh
# STATUS: squeue -u $USER
# CANCEL: scancel
