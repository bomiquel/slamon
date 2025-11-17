# @file Batch.msg
# @brief Personalised message to define a keypoint cluster.
# @package slamon
#
# @author
#        Bo Miquel Nordfeldt-Fiol (2024–2025)
#
# @copyright
#        Copyright (c) 2025 Bo Miquel Nordfeldt-Fiol
#
# @license BSD-3-Clause
#        This software is released under the BSD 3-Clause License.
#        See the LICENSE file for full text.

# Global cluster id.
int32 global_id

# Left and right Keypoints.
Keypoint[] l_kpts
Keypoint[] r_kpts

# 3D camera points.
geometry_msgs/Point[] l_cam_pts

# 3D centroid
geometry_msgs/Point centroid