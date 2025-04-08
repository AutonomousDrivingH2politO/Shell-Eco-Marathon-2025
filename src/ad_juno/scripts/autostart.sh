#!/bin/bash

# Configuration
LOG_FILE="/home/bylogix/autostart.log"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_NAME="Shell-Eco-Marathon-2025"
PYTHON_SCRIPT_NAME="engage_button.py"

# Initialize logging
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== Startup at $(date) ==="

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "Initializing autonomous driving system"

# Source ROS
log "Sourcing ROS..."
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    log "ERROR: ROS setup not found at $ROS_SETUP"
    exit 1
fi

# Find and source workspace
log "Locating workspace..."
WORKSPACE_PATH=$(find /home/bylogix -maxdepth 3 -name "$WORKSPACE_NAME" -type d -print -quit)
if [ -z "$WORKSPACE_PATH" ]; then
    log "ERROR: Workspace not found"
    exit 1
fi
log "Workspace found: $WORKSPACE_PATH"

WORKSPACE_SETUP="$WORKSPACE_PATH/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
else
    log "ERROR: Workspace setup not found"
    exit 1
fi

# Locate Python script
log "Finding Python script..."
PYTHON_SCRIPT=$(find "$WORKSPACE_PATH" -name "$PYTHON_SCRIPT_NAME" -print -quit)
if [ -z "$PYTHON_SCRIPT" ]; then
    log "ERROR: Python script not found"
    exit 1
fi
log "Python script: $PYTHON_SCRIPT"

# Run indefinitely (systemd will handle restarts)
log "Starting main application loop..."
cd "$(dirname "$PYTHON_SCRIPT")" || {
    log "ERROR: Failed to enter script directory"
    exit 1
}

while true; do
    log "Launching $PYTHON_SCRIPT_NAME..."
    python3 "$(basename "$PYTHON_SCRIPT")"
    EXIT_CODE=$?
    log "Script exited with code $EXIT_CODE. Restarting in 5 seconds..."
    sleep 5
done
