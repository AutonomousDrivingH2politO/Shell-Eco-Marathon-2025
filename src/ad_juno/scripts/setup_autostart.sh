#!/bin/bash

# Configuration
SERVICE_NAME="autonomous_driving"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_PATH="/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh"
SERVICE_USER="bylogix"

# Verify the user exists
if ! id -u "$SERVICE_USER" >/dev/null 2>&1; then
    echo "Error: User '$SERVICE_USER' does not exist!"
    exit 1
fi

# Verify the script exists and is executable
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: Script not found at $SCRIPT_PATH"
    exit 1
fi

# Make the script executable
if ! sudo -u "$SERVICE_USER" chmod +x "$SCRIPT_PATH"; then
    echo "Error: Failed to make script executable"
    exit 1
fi

# Create the systemd service file
sudo tee "$SERVICE_FILE" > /dev/null <<EOL
[Unit]
Description=Autonomous Driving System
After=graphical.target
Requires=graphical.target

[Service]
Type=simple
<<<<<<< HEAD
User=$SERVICE_USER
ExecStart=$SCRIPT_PATH
Restart=always                # Restart indefinitely
=======
User=bylogix
ExecStart=/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh
Restart=on-failure
>>>>>>> 9b118d82f9bfa9e90b2ae189dceaa562167c6108
RestartSec=5s
Environment="DISPLAY=:0"      # Explicitly set display
Environment="XAUTHORITY=/home/$SERVICE_USER/.Xauthority"

[Install]
WantedBy=graphical.target
EOL

# Validate service file creation
if [ ! -f "$SERVICE_FILE" ]; then
    echo "Error: Failed to create service file at $SERVICE_FILE"
    exit 1
fi

# Reload systemd and enable/start the service
sudo systemctl daemon-reload || { echo "Error: Failed to reload systemd"; exit 1; }
sudo systemctl enable "$SERVICE_NAME" || { echo "Error: Failed to enable service"; exit 1; }
sudo systemctl start "$SERVICE_NAME" || { echo "Error: Failed to start service"; exit 1; }

# Verify service status
echo -e "\nService status:"
if ! sudo systemctl status "$SERVICE_NAME"; then
    echo -e "\nError: Service failed to start. Checking logs..."
    sudo journalctl -u "$SERVICE_NAME" -b --no-pager | tail -n 20
    exit 1
fi

echo -e "\nService installed successfully! Key features:"
echo "- Starts after graphical environment (DISPLAY=:0)"
echo "- Restarts indefinitely on failure/crash (Restart=always)"
echo "- Runs under user '$SERVICE_USER' with X permissions"
