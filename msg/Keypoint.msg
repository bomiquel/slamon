# \file Keypoint.msg
# \brief Custom ROS message representing a 2D visual keypoint and its feature descriptor.
#
# \author Bo Miquel Nordfeldt-Fiol (2024–2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


# Feature descriptor vector of the keypoint.
float32[] desc

# 2D image coordinates of the keypoint (x, y) [px].
geometry_msgs/Point pt