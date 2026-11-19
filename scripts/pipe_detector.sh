#!/bin/bash

# \file pipe_detector.sh
# \brief Script to launch pipe detector
#
# \author Bo Miquel Nordfeldt-Fiol (2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


source ~/miniconda3/etc/profile.d/conda.sh

if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    conda deactivate
fi

conda activate pipes2

source ~/catkin_ws/devel/setup.bash

exec python3 ~/catkin_ws/src/dgcnn/sem_seg/inference_ros_tf_slam_nobb_unify.py "$@"