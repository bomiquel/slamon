# \file Keyframe.msg
# \brief Custom ROS message representing a stereo keyframe and its keypoint clusters.
#
# \author Bo Miquel Nordfeldt-Fiol (2024–2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


# Standard ROS message header (timestamp and coordinate frame).
Header header

# Unique keyframe ID.
int32 id

# List of spatial keypoint clusters extracted from this keyframe.
Batch[] batches

# Flattened array of matched cluster ID pairs between the current and previous keyframes (c_cluster_id, k_cluster_id).
int32[] cluster_pairs # c_cluster_id - p_cluster_id

# Number of matched geometrical inliers for each cluster pair.
int32[] inliers_per_pair

# Left stereo image (optional, populated when debugging is enabled).
sensor_msgs/Image l_img

# Right stereo image (optional, populated when debugging is enabled).
sensor_msgs/Image r_img