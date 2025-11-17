# @file Keyframe.msg
# @brief Personalised message to define a keyframe.
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