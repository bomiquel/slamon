# Global cluster id.
int32 global_id

# Left and right Keypoints.
Keypoint[] l_kpts
Keypoint[] r_kpts

# 3D camera points.
geometry_msgs/Point[] l_cam_pts

# 3D centroid
geometry_msgs/Point centroid