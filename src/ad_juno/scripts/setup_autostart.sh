#!/bin/bash

SERVICE_NAME="autonomous_driving"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_PATH="/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh"
SERVICE_USER="root"

# Make the script executable
chmod +x "$SCRIPT_PATH" || {
    echo "Error: Failed to make script executable"
    exit 1
}

# Create systemd service file
sudo tee "$SERVICE_FILE" > /dev/null <<EOL
[Unit]
Description=Autonomous Driving System (CAN + ROS + Python Script)
After=network.target

[Service]
Type=simple
ExecStart=/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target

EOL

# Reload, enable, and start
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl start "$SERVICE_NAME"

