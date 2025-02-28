# Custom message to evaluate SLAMON execution times.
# To avoid creating a message for each part of the 
# system, this message is reused in all three threads.

# Typical ROS message header
Header header

# Successes in the last iteration of the thread in question.
# VO: If it has achieved a valid estimate of the movement 
# between images (bool), MM: If it has included a keyframe
# on the map (bool) and LC: Number of closed loops
int16 successes_num

# Run times.
float64 run_time_0      # Time needed for VO: Feature extraction, MM: From StereoImageAttributes msg to Frame object or LC: Process current cluster.
float64 run_time_1      # Time needed for VO: Stereo matching, MM: Need new keyframe? or LC: Search for candidates to close loops by proximity.
float64 run_time_2      # Time needed for VO: Triangulation, MM: Add vertex (cluster) to graph or LC: Search for candidates to close loops by hash.
float64 run_time_3      # Time needed for VO: Temporal matching, MM: Add edges to graph or LC: Read cluster candidates.
float64 run_time_4      # Time needed for VO: Bucketing, MM: Write graph to disk and publish updated robot pose or LC: Loop detection.
float64 run_time_5      # Time needed for VO: Motion estimation, MM: Nothing or LC: Close loop.
float64 total_run_time  # Time needed for each thread to perform one iteration. 