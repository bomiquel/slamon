# \file Performance.msg
# \brief Custom ROS message for reporting per-stage execution runtimes across SLAMON modules.
#        To avoid defining a separate message for each subsystem, this message is shared and 
#        reused across all three main threads: Visual Odometry (VO), Mapmaker (MM), and Loop Closer (LC).
#
# \author Bo Miquel Nordfeldt-Fiol (2024–2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


# Standard ROS message header (timestamp and coordinate frame).
Header header

# Number of successful operations in the last iteration of the thread:
# - VO: Valid frame-to-frame motion estimation achieved (0 or 1).
# - MM: New keyframe inserted into the map (0 or 1).
# - LC: Number of loop closures detected in the current iteration.
int16 successes_num

# Per-stage execution runtimes [s].
float64 run_time_0      # Execution time for stage 0 (VO: Feature extraction | MM: Pose update | LC: Process current cluster) [s].
float64 run_time_1      # Execution time for stage 1 (VO: Stereo matching | MM: Add vertex to graph | LC: Proximity candidate search) [s].
float64 run_time_2      # Execution time for stage 2 (VO: Triangulation | MM: Add intra-keyframe edges | LC: Hash candidate search) [s].
float64 run_time_3      # Execution time for stage 3 (VO: Clustering | MM: Add inter-frame edges | LC: Read candidate clusters) [s].
float64 run_time_4      # Execution time for stage 4 (VO: Temporal matching | MM: Publish messages | LC: Loop detection) [s].
float64 run_time_5      # Execution time for stage 5 (VO: Keypoint bucketing | MM: Write map data to disk | LC: Loop correction) [s].
float64 run_time_6      # Execution time for stage 6 (VO: Motion estimation | MM: Unused | LC: Unused) [s].
float64 run_time_7      # Execution time for stage 7 (VO: Keyframe insertion decision | MM: Unused | LC: Unused) [s].
float64 total_run_time  # Total execution time required by the thread to complete one full iteration [s].