# Typical ROS message header
Header header

# Keyframe id.
int32 keyframe_id

# Identifier of the robot that obtained the cluster.
int16 robot_id

# Clusters.
Batch[] batches

# Left image (optional).
sensor_msgs/Image l_img