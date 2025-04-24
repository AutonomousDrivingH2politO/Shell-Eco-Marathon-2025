#!/bin/bash

SERVICE_NAME="autonomous_driving"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_PATH="/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh"
SERVICE_USER="bylogix"

# Make the script executable
chmod +x "$SCRIPT_PATH" || {
    echo "Error: Failed to make script executable"
    exit 1
}

# Create systemd service file
sudo tee "$SERVICE_FILE" > /dev/null <<EOL
[Unit]
Description=Autonomous Driving System
After=graphical.target

[Service]
Type=simple
User=$SERVICE_USER
ExecStart=$SCRIPT_PATH
Restart=on-failure
RestartSec=5s
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/$SERVICE_USER/.Xauthority"

[Install]
WantedBy=graphical.target
EOL

# Reload, enable, and start
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl start "$SERVICE_NAME"

