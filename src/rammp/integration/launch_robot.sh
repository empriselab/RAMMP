#!/bin/bash

USER_HOME="$HOME"
BASE_DIR="$USER_HOME/rammp_ws/src/RAMMP/src/rammp"

CONTROL_DIR="$BASE_DIR/control/robot_controller"
MISC_DIR="$BASE_DIR/misc"
SAFETY_DIR="$BASE_DIR/safety"

# Function to clean up background processes
cleanup() {
    echo "Stopping background processes..."
    if kill -0 $joint_states_publisher_pid 2>/dev/null; then
        kill $joint_states_publisher_pid
    fi
    if kill -0 $speaker_pid 2>/dev/null; then
        kill $speaker_pid
    fi
    if kill -0 $collision_sensor_pid 2>/dev/null; then
        kill $collision_sensor_pid
    fi
    if kill -0 $transfer_button_pid 2>/dev/null; then
        kill $transfer_button_pid
    fi
    if kill -0 $watchdog_pid 2>/dev/null; then
        kill $watchdog_pid
    fi
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT

# Start joint states publisher
cd "$CONTROL_DIR"
python joint_states_publisher.py &
joint_states_publisher_pid=$!  # Store the PID of joint_states_publisher

# Start speaker
cd "$MISC_DIR"
python speak.py &
speaker_pid=$!  # Store the PID of speaker

# Start transfer button
# cd "$MISC_DIR"
# python transfer_button_listener.py --button_id -1 &
# transfer_button_pid=$!  # Store the PID of transfer_button_listener

cd "$SAFETY_DIR"

# Start collision sensor
python collision_sensor.py &
collision_sensor_pid=$!  # Store the PID of collision_sensor

# Wait for 2 second to make sure collision sensor is running
sleep 2

# Run watchdog
python watchdog.py 

cleanup  # Ensure cleanup is called when bulldog finishes
wait $joint_states_publisher_pid
wait $speaker_pid
wait $collision_sensor_pid
# wait $transfer_button_pid
