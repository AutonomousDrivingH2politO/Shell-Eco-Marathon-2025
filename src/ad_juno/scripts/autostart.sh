#!/bin/bash

# Configuration
LOG_FILE="/home/bylogix/autostart.log"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_NAME="Shell-Eco-Marathon-2025"
PYTHON_SCRIPT_NAME="engage_button.py"

# Logging
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== Startup at $(date) ==="

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "Running CAN setup script..."
modprobe can
modprobe mttcan 
modprobe can_raw
ip link set can1 type can bitrate 1000000
ip link set can1 up
ip link set can0 type can bitrate 250000
ip link set can0 up


log "Initializing autonomous driving system..."

# Source ROS 2
log "Sourcing ROS 2 setup..."
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    log "ERROR: ROS 2 setup not found at $ROS_SETUP"
    exit 1
fi

# Find workspace
log "Looking for workspace '$WORKSPACE_NAME'..."
WORK_DIR=$(find /home/bylogix -type d -name "$WORKSPACE_NAME" -print -quit)
if [ -z "$WORK_DIR" ]; then
    log "ERROR: Workspace not found"
    exit 1
fi
log "Workspace found at: $WORK_DIR"

# Build the workspace
log "Running colcon build..."
cd "$WORK_DIR"
colcon build --symlink-install
if [ $? -ne 0 ]; then
    log "ERROR: colcon build failed"
    exit 1
fi

# Source workspace
SETUP_SCRIPT="$WORK_DIR/install/setup.bash"
if [ -f "$SETUP_SCRIPT" ]; then
    source "$SETUP_SCRIPT"
    log "Workspace sourced successfully"
else
    log "ERROR: setup.bash not found in install/"
    exit 1
fi

# Find and run Python script
log "Searching for Python script '$PYTHON_SCRIPT_NAME'..."
PYTHON_SCRIPT=$(find "$WORK_DIR" -name "$PYTHON_SCRIPT_NAME" -print -quit)
if [ -z "$PYTHON_SCRIPT" ]; then
    log "ERROR: Python script not found"
    exit 1
fi
log "Found Python script: $PYTHON_SCRIPT"

cd "$(dirname "$PYTHON_SCRIPT")" || {
    log "ERROR: Failed to enter Python script directory"
    exit 1
}

# Run the script inside an infinite loop with full ROS environment sourced
while true; do
    log "Launching $PYTHON_SCRIPT_NAME..."
    python3 "$(basename "$PYTHON_SCRIPT")"
    EXIT_CODE=$?
    log "Script exited with code $EXIT_CODE. Restarting in 5 seconds..."
    sleep 5
done

