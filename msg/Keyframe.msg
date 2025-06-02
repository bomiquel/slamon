# Typical ROS message header
Header header

# Keyframe id.
int32 keyframe_id

# Clusters.
Batch[] batches

# Cluster pairs.
int32[] cluster_pairs

# Inliers per cluster pair.
int32[] inliers_per_pair

# Left image (optional).
sensor_msgs/Image l_img

# Right image (optional).
sensor_msgs/Image r_img