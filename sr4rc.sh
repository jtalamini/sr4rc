#!/bin/bash
#SBATCH --cpus-per-task=36
#SBATCH --mem=16GB
#SBATCH --account=uTS20_Talamini
#SBATCH --time=10:00:00
#SBATCH --partition=gll_usr_prod

DIRNAME=sr4rc.$SLURM_ARRAY_JOB_ID

mkdir $DIRNAME

# create cymbolic link for common folder
# ln -s $WORK common

#evolution
common/jdk-14.0.2/bin/java -Xmx12g -cp sr4rc.jar it.units.erallab.SR4RC randomSeed=$SLURM_ARRAY_TASK_ID evolver=cmaes gridSize=10 avalancheThreshold=0.0002 nGaussian=10 dir=$DIRNAME statsFile=stats-$SLURM_ARRAY_TASK_ID.txt