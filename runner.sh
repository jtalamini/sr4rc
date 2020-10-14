#!/bin/bash

CONTROLLER=centralized
for TASK in locomotion jump hiking stairway;
do
  # serialized robots
  for i in {0..9}
  do
    sbatch --array=0-9 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt brain.sh serialized $i $TASK 0 0 $CONTROLLER
  done
  # random robots
  for i in {0..9}
  do
    sbatch --array=0-9 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt brain.sh random $i $TASK 20 20 $CONTROLLER
  done
  # box robots
  sbatch --array=0-9 --nodes=1 -o logs/out.%A_%a.txt -e logs/err.%A_%a.txt brain.sh box 0 $TASK 5 4 $CONTROLLER
done
