#!/bin/bash

# Log file for debugging
rm -f "/home/bylogix/autostart_log.txt"

LOG_FILE="/home/bylogix/autostart_log.txt"

# Function to log messages
log_message() {
    echo "$(date): $1" >> $LOG_FILE
}

log_message "Autostart script initiated"

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
log_message "Launching ROS 2 nodes..."
ros2 launch juno_bringup button_engage.launch.py &
if [ $? -ne 0 ]; then
    log_message "Failed to launch button_engage.launch.py"
    exit 1
fi

log_message "System launched successfully"

# Keep the script running to maintain the launched processes
wait
