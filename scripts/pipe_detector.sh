#!/bin/bash

source ~/miniconda3/etc/profile.d/conda.sh

if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    conda deactivate
fi

conda activate pipes2

source ~/catkin_ws/devel/setup.bash

exec python3 ~/catkin_ws/src/dgcnn/sem_seg/inference_ros_tf_slam_nobb_unify.py "$@"