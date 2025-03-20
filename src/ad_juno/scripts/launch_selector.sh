#!/bin/bash

# Read the selected launch file from a config file
LAUNCH_CONFIG="/home/bylogix/launch_config.txt"
if [[ -f "$LAUNCH_CONFIG" ]]; then
    LAUNCH_FILE=$(cat $LAUNCH_CONFIG)
else
    # Default to path_planning if no config file exists
    LAUNCH_FILE='path_planning.launch.py'
fi

# Launch the selected file
ros2 launch juno_bringup $LAUNCH_FILE