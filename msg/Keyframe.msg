# @file Keyframe.msg
# @brief Personalised message to define a keyframe.
# @package slamon
# @project SLAMON
#
# @author
#   Bo Miquel Nordfeldt-Fiol (2024–2026)
#
# @license BSD-4-Clause
#   This file is part of the SLAMON project and is released under
#   the BSD 4-Clause License. See the LICENSE file for details.


# Typical ROS message header
Header header

# Keyframe id.
int32 id

# Clusters.
Batch[] batches

# Cluster pairs.
int32[] cluster_pairs # c_cluster_id - p_cluster_id

# Inliers per cluster pair.
int32[] inliers_per_pair

# Left image (optional).
sensor_msgs/Image l_img

# Right image (optional).
sensor_msgs/Image r_img