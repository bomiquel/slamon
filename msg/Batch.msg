# @file Batch.msg
# @brief Personalised message to define a keypoint cluster.
# @package slamon
# @project SLAMON
#
# @author
#   Bo Miquel Nordfeldt-Fiol (2024–2025)
#
# @license BSD-4-Clause
#   This file is part of the SLAMON project and is released under
#   the BSD 4-Clause License. See the LICENSE file for details.


# Global cluster id.
int32 global_id

# Left and right Keypoints.
Keypoint[] l_kpts
Keypoint[] r_kpts

# 3D camera points.
geometry_msgs/Point[] l_cam_pts

# 3D centroid
geometry_msgs/Point centroid