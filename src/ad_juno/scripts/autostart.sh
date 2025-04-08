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

<<<<<<< HEAD
# Source ROS
log "Sourcing ROS..."
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    log "ERROR: ROS setup not found at $ROS_SETUP"
=======
# Wait for system to fully boot (check service status)
log_message "Checking system boot..."
systemctl is-active --quiet autostart.service || systemctl start autostart.service
if [ $? -ne 0 ]; then
    log_message "Failed to start or check autostart.service"
    exit 1
fi
log_message "System boot wait complete"

gnome-terminal

# Source the ROS 2 environment
log_message "Sourcing ROS 2 environment..."
source "/opt/ros/humble/setup.bash"
if [ $? -ne 0 ]; then
    log_message "Failed to source ROS 2 environment from /opt/ros/humble/setup.bash"
    exit 1
fi
log_message "ROS 2 environment sourced"

# Dynamically find the workspace directory (restricted search to a specific path)
log_message "Searching for the workspace 'Shell-Eco-Marathon-2025'..."
work_dir=$(find /home/bylogix -type d -name 'Shell-Eco-Marathon-2025' -print -quit)
if [ -z "$work_dir" ]; then
    log_message "Workspace 'Shell-Eco-Marathon-2025' not found in /home/bylogix!"
    exit 1
fi
log_message "Workspace found at: $work_dir"


# Source the workspace setup file
log_message "Sourcing workspace setup file..."
source "$work_dir/install/setup.bash"
if [ $? -ne 0 ]; then
    log_message "Failed to source workspace setup.bash from $work_dir/install/setup.bash"
    exit 1
fi
log_message "Workspace sourced"

log_message "Starting autonomous driving system"

# Launch ROS 2 nodes (uncomment both if needed)
log_message "Launching engage_button.py"
# ros2 launch juno_bringup button_engage.launch.py &
exec python3 engage_button.py &

if [ $? -ne 0 ]; then
    log_message "Failed to launch engage_button.py"
>>>>>>> 9b118d82f9bfa9e90b2ae189dceaa562167c6108
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
