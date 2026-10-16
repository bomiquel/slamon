#!/bin/bash

# @file cpu_mem_logger.sh
# @brief Script to monitor CPU and RAM usage.
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
NODES_TO_MONITOR=("slamon_vo" "slamon_map" "seg_pc")
INTERVAL_SECONDS=1
# =================================================

# Trap to gracefully exit when roslaunch sends SIGINT or SIGTERM
trap "echo 'Halting CPU/RAM logger smoothly...'; exit 0" SIGINT SIGTERM

echo "Initialising CPU and RAM monitor..."

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
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_cpu.txt"
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_ram.txt"
done

echo "Recording data. Awaiting termination signal from roslaunch..."

# Dictionaries to store previous CPU ticks to compute exact deltas
declare -A PREV_TICKS
declare -A PREV_UPTIME
HZ=$(getconf CLK_TCK) # System ticks per second (usually 100)

# 3. Main monitoring loop
while true; do
    TIMESTAMP=$(date +%s%N)

    for NODE in "${NODES_TO_MONITOR[@]}"; do

        # Filter
        # PID=$(ps -eo pid,cmd | grep "$NODE" | grep -v -E "grep|rostopic|automator|cpu_mem_logger" | awk 'NR==1{print $1}')
        # PID=$(ps -eo pid,cmd | grep "$NODE" | grep -v -E "grep|rostopic|automator|cpu_mem_logger|roslaunch" | awk 'NR==1{print $1}')
        FULL_NODE_NAME=$(rosnode list 2>/dev/null | grep -E "${NODE}$" | head -n 1)

        if [ -n "$FULL_NODE_NAME" ]; then
            # 2. Ask ROS for the exact PID of that specific node
            PID=$(rosnode info "$FULL_NODE_NAME" 2>/dev/null | grep -i "Pid:" | awk '{print $2}')
        else
            PID=""
        fi

        if [ -n "$PID" ]; then
            # Extract CPU (%) and RAM in Kilobytes (rss)
            # STATS=$(ps -p "$PID" -o %cpu,rss --no-headers)
            # CPU_USAGE=$(echo "$STATS" | awk '{print $1}')
            # RAM_KB=$(echo "$STATS" | awk '{print $2}')
            # CPU_USAGE=$(LC_ALL=C top -b -n 2 -d 0.1 -p "$PID" 2>/dev/null | awk -v pid="$PID" '$1==pid {cpu=$9} END {print cpu}')
            
            # --- 1. MEMORY ACQUISITION ---
            RAM_KB=$(ps -p "$PID" -o rss --no-headers | awk '{print $1}')
            if [ -n "$RAM_KB" ]; then
                RAM_MB=$(echo "scale=2; $RAM_KB / 1024" | bc -l) # Convert Kilobytes to Megabytes with 2 decimal places
            else
                RAM_MB="0.00"
            fi

            # --- 2. CPU ACQUISITION (Exact Delta Method) ---
            # Read directly from the Linux kernel to get total CPU time
            STAT=$(cat /proc/$PID/stat 2>/dev/null)
            if [ -n "$STAT" ]; then
                # Extract fields 14 (user mode ticks) and 15 (kernel mode ticks)
                UTIME=$(echo "$STAT" | awk '{print $14}')
                STIME=$(echo "$STAT" | awk '{print $15}')
                TOTAL_TICKS=$((UTIME + STIME))
                
                # Get exact system uptime
                UPTIME=$(cat /proc/uptime | awk '{print $1}')

                if [ -n "${PREV_TICKS[$NODE]}" ]; then
                    # Calculate difference in CPU ticks and difference in real time
                    DELTA_TICKS=$((TOTAL_TICKS - PREV_TICKS[$NODE]))
                    DELTA_TIME=$(awk -v t1="${PREV_UPTIME[$NODE]}" -v t2="$UPTIME" 'BEGIN { print t2 - t1 }')

                    # Calculate precise % over the elapsed interval (approx 1s)
                    CPU_USAGE=$(awk -v dticks="$DELTA_TICKS" -v hz="$HZ" -v dtime="$DELTA_TIME" 'BEGIN { 
                        if (dtime > 0) printf "%.1f", (dticks / hz) / dtime * 100 
                        else print "0.0" 
                    }')
                else
                    # First iteration: we don't have a baseline yet
                    CPU_USAGE="0.0"
                fi

                # Save current data to serve as baseline for the next second
                PREV_TICKS[$NODE]=$TOTAL_TICKS
                PREV_UPTIME[$NODE]=$UPTIME
            else
                CPU_USAGE="0.0"
                unset PREV_TICKS[$NODE]
                unset PREV_UPTIME[$NODE]
            fi

        else
            CPU_USAGE="0.0"
            RAM_MB="0.00"
            # Reset baseline if process dies
            unset PREV_TICKS[$NODE]
            unset PREV_UPTIME[$NODE]
        fi

        # Append the extracted data to their respective .txt files
        echo "$TIMESTAMP,$CPU_USAGE" >> "$WORKING_PATH/${NODE}_cpu.txt"
        echo "$TIMESTAMP,$RAM_MB" >> "$WORKING_PATH/${NODE}_ram.txt"
    done

    sleep "$INTERVAL_SECONDS"
done