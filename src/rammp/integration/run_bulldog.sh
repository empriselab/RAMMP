#!/bin/bash

# Function to clean up background processes
cleanup() {
    echo "Stopping background processes..."
    if kill -0 $estops_pid 2>/dev/null; then
        kill $estops_pid
    fi
    if kill -0 $bulldog_pid 2>/dev/null; then
        kill $bulldog_pid
    fi
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT

# ROS2 does not require roscore

# Run estops_publisher.py in the background
cd ~/RAMMP/src/rammp/safety
python estops_publisher.py --user_id 2 --exp_id 1 &
estops_pid=$!  # Store the PID of estops_publisher

# Wait for 2 seconds
sleep 2

# Run bulldog
python bulldog.py

cleanup  # Ensure cleanup is called when bulldog finishes
wait $estops_pid
