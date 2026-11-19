# \file Batch.msg
# \brief Custom ROS message representing a spatial keypoint cluster within a keyframe.
#
# \author Bo Miquel Nordfeldt-Fiol (2024–2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


# Global cluster ID.
int32 global_id

# Left and right stereo 2D keypoints and descriptors.
Keypoint[] l_kpts
Keypoint[] r_kpts

# Triangulated 3D points relative to the left camera coordinate frame.
geometry_msgs/Point[] l_cam_pts

# 3D geometric centroid of the cluster in the left camera coordinate frame.
geometry_msgs/Point centroid