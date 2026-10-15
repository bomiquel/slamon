#!/bin/bash

# @file mem_logger.sh
# @brief Script to monitor RAM usage.
#
# @package slamon
# @project SLAMON
#
# @author
#   Bo Miquel Nordfeldt-Fiol (2026)
#
# @license BSD-4-Clause
#   This file is part of the SLAMON project and is released under
#   the BSD 4-Clause License. See the LICENSE file for details.


# ================= CONFIGURATION =================
# Array of nodes to monitor. To make it scalable, simply add your new nodes here.
NODES_TO_MONITOR=("slamon_vo" "slamon_map")
INTERVAL_SECONDS=1
# =================================================

# Trap to gracefully exit when roslaunch sends SIGINT or SIGTERM
trap "echo 'Halting RAM logger smoothly...'; exit 0" SIGINT SIGTERM

echo "Initialising RAM monitor..."

# 1. Retrieve the working directory from ROS parameters
WORKING_PATH=""
echo "Awaiting the ROS parameter '*slamon/working_path' (namespace-agnostic)..."

while [ -z "$WORKING_PATH" ]; do
    # Auto-discover the full parameter path, regardless of the robot's namespace
    # It will automatically match paths like "/girona500/slamon/working_path"
    PARAM_NAME=$(rosparam list 2>/dev/null | grep -m 1 "slamon/working_path$")

    if [ -n "$PARAM_NAME" ]; then
        # Parameter found! Retrieve its value
        WORKING_PATH=$(rosparam get "$PARAM_NAME" 2>/dev/null)
    fi

    # If it is still not ready, pause and retry
    if [ -z "$WORKING_PATH" ]; then
        sleep 1
    fi
done

# Remove any potential surrounding quotes from the ROS parameter output
WORKING_PATH=$(echo "$WORKING_PATH" | tr -d '"')
echo "Target directory identified: $WORKING_PATH"

# Ensure the directory actually exists, create it if it does not
mkdir -p "$WORKING_PATH"

# 2. Initialise the .txt files with the required header
for NODE in "${NODES_TO_MONITOR[@]}"; do
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_ram.txt"
done

echo "Recording data. Awaiting termination signal from roslaunch..."

# 3. Main monitoring loop
while true; do
    TIMESTAMP=$(date +%s%N)

    for NODE in "${NODES_TO_MONITOR[@]}"; do

        # Filter
        FULL_NODE_NAME=$(rosnode list 2>/dev/null | grep -E "${NODE}$" | head -n 1)

        if [ -n "$FULL_NODE_NAME" ]; then
            # 2. Ask ROS for the exact PID of that specific node
            PID=$(rosnode info "$FULL_NODE_NAME" 2>/dev/null | grep -i "Pid:" | awk '{print $2}')
        else
            PID=""
        fi

        if [ -n "$PID" ]; then
           
            RAM_KB=$(ps -p "$PID" -o rss --no-headers | awk '{print $1}')
            if [ -n "$RAM_KB" ]; then
                RAM_MB=$(echo "scale=2; $RAM_KB / 1024" | bc -l) # Convert Kilobytes to Megabytes with 2 decimal places
            else
                RAM_MB="0.00"
            fi
        else
            RAM_MB="0.00"
        fi

        # Append the extracted data to their respective .txt files
        echo "$TIMESTAMP,$RAM_MB" >> "$WORKING_PATH/${NODE}_ram.txt"
    done

    sleep "$INTERVAL_SECONDS"
done