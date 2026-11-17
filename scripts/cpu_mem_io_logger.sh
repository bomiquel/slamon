#!/bin/bash

# \file cpu_mem_logger.sh
# \brief Script to monitor CPU, RAM and I/O usage.
#
# \author Bo Miquel Nordfeldt-Fiol (2026)
#
# \copyright BSD-4-Clause
# This file is part of the SLAMON project (package: slamon) and is released under
# the BSD 4-Clause License. See the top-level LICENSE file for details.


# ================= CONFIGURATION =================
NODES_TO_MONITOR=("slamon_vo" "slamon_map")
INTERVAL_SECONDS=1
# =================================================

trap "echo 'Halting CPU/RAM/IO logger smoothly...'; exit 0" SIGINT SIGTERM

echo "Initialising real-time CPU, RAM and Disk I/O monitor..."

# 1. Retrieve the working directory
WORKING_PATH=""
echo "Awaiting the ROS parameter '*slamon/working_path' (namespace-agnostic)..."

while [ -z "$WORKING_PATH" ]; do
    PARAM_NAME=$(rosparam list 2>/dev/null | grep -m 1 "slamon/working_path$")
    if [ -n "$PARAM_NAME" ]; then
        WORKING_PATH=$(rosparam get "$PARAM_NAME" 2>/dev/null)
    fi
    if [ -z "$WORKING_PATH" ]; then
        sleep 1
    fi
done

WORKING_PATH=$(echo "$WORKING_PATH" | tr -d '"')
echo "Target directory identified: $WORKING_PATH"
mkdir -p "$WORKING_PATH"

# 2. Initialise the .txt files
for NODE in "${NODES_TO_MONITOR[@]}"; do
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_cpu.txt"
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_ram.txt"
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_io_read_mbps.txt"
    echo "%time,field.data" > "$WORKING_PATH/${NODE}_io_write_mbps.txt"
done

echo "Recording data. Awaiting termination signal from roslaunch..."

# Dictionaries for baselines
declare -A PREV_TICKS
declare -A PREV_UPTIME
declare -A PREV_RCHAR
declare -A PREV_WCHAR
HZ=$(getconf CLK_TCK)

# 3. Main monitoring loop
while true; do
    TIMESTAMP=$(date +%s%N)

    for NODE in "${NODES_TO_MONITOR[@]}"; do

        # Filter to get exact PID
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
            
            # --- 1. MEMORY ACQUISITION ---
            RAM_KB=$(ps -p "$PID" -o rss --no-headers | awk '{print $1}')
            if [ -n "$RAM_KB" ]; then
                RAM_MB=$(echo "scale=2; $RAM_KB / 1024" | bc -l)
            else
                RAM_MB="0.00"
            fi

            # --- 2. CPU & DISK I/O ACQUISITION ---
            STAT=$(cat /proc/$PID/stat 2>/dev/null)
            IO_STAT=$(cat /proc/$PID/io 2>/dev/null)
            
            if [ -n "$STAT" ] && [ -n "$IO_STAT" ]; then
                # CPU Ticks
                UTIME=$(echo "$STAT" | awk '{print $14}')
                STIME=$(echo "$STAT" | awk '{print $15}')
                TOTAL_TICKS=$((UTIME + STIME))
                
                # Disk I/O Bytes (rchar = read characters, wchar = write characters)
                RCHAR=$(echo "$IO_STAT" | grep -w "rchar:" | awk '{print $2}')
                WCHAR=$(echo "$IO_STAT" | grep -w "wchar:" | awk '{print $2}')
                
                UPTIME=$(cat /proc/uptime | awk '{print $1}')

                if [ -n "${PREV_TICKS[$NODE]}" ]; then
                    # CPU Calculation
                    DELTA_TICKS=$((TOTAL_TICKS - PREV_TICKS[$NODE]))
                    DELTA_TIME=$(awk -v t1="${PREV_UPTIME[$NODE]}" -v t2="$UPTIME" 'BEGIN { print t2 - t1 }')

                    CPU_USAGE=$(awk -v dticks="$DELTA_TICKS" -v hz="$HZ" -v dtime="$DELTA_TIME" 'BEGIN { 
                        if (dtime > 0) printf "%.1f", (dticks / hz) / dtime * 100 
                        else print "0.0" 
                    }')
                    
                    # Disk I/O Calculation (Delta Bytes -> MB/s)
                    DELTA_RCHAR=$((RCHAR - PREV_RCHAR[$NODE]))
                    DELTA_WCHAR=$((WCHAR - PREV_WCHAR[$NODE]))
                    
                    READ_MBPS=$(awk -v dbytes="$DELTA_RCHAR" -v dtime="$DELTA_TIME" 'BEGIN { 
                        if (dtime > 0) printf "%.2f", (dbytes / 1048576) / dtime 
                        else print "0.00" 
                    }')
                    WRITE_MBPS=$(awk -v dbytes="$DELTA_WCHAR" -v dtime="$DELTA_TIME" 'BEGIN { 
                        if (dtime > 0) printf "%.2f", (dbytes / 1048576) / dtime 
                        else print "0.00" 
                    }')
                else
                    CPU_USAGE="0.0"
                    READ_MBPS="0.00"
                    WRITE_MBPS="0.00"
                fi

                # Save baselines for next iteration
                PREV_TICKS[$NODE]=$TOTAL_TICKS
                PREV_UPTIME[$NODE]=$UPTIME
                PREV_RCHAR[$NODE]=$RCHAR
                PREV_WCHAR[$NODE]=$WCHAR
            else
                CPU_USAGE="0.0"
                READ_MBPS="0.00"
                WRITE_MBPS="0.00"
                unset PREV_TICKS[$NODE] PREV_UPTIME[$NODE] PREV_RCHAR[$NODE] PREV_WCHAR[$NODE]
            fi

        else
            CPU_USAGE="0.0"
            RAM_MB="0.00"
            READ_MBPS="0.00"
            WRITE_MBPS="0.00"
            unset PREV_TICKS[$NODE] PREV_UPTIME[$NODE] PREV_RCHAR[$NODE] PREV_WCHAR[$NODE]
        fi

        # Append to files
        echo "$TIMESTAMP,$CPU_USAGE" >> "$WORKING_PATH/${NODE}_cpu.txt"
        echo "$TIMESTAMP,$RAM_MB" >> "$WORKING_PATH/${NODE}_ram.txt"
        echo "$TIMESTAMP,$READ_MBPS" >> "$WORKING_PATH/${NODE}_io_read_mbps.txt"
        echo "$TIMESTAMP,$WRITE_MBPS" >> "$WORKING_PATH/${NODE}_io_write_mbps.txt"
    done

    sleep "$INTERVAL_SECONDS"
done