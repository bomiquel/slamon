# Customised message to evaluate the internal 
# performance of a Visual Odometer

# Typical ROS message header
Header header

# True if in the last iteration all keypoints
# could be matched and the movement was correctly
# estimated.
bool valid_motion

# Runtimes
float64 feature_extraction_run_time
float64 stereo_matching_run_time
float64 triangulation_run_time
float64 successive_matching_run_time
float64 motion_estimation_run_time
float64 total_run_time