#!/bin/bash

cat > /tmp/autonomous_driving.service << 'EOL'
[Unit]
Description=Autonomous Driving System
After=network.target


[Service]
Type=simple
User=bylogix
ExecStart=/home/bylogix/Shell-Eco-Marathon-2025/src/ad_juno/scripts/autostart.sh
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
EOL

# Copy the service file to the systemd directory
sudo cp /tmp/autonomous_driving.service /etc/systemd/system/

dir_name=$(find /home -d -name 'Shell-Eco-Marathon-2025')

chmod +x "$dir_name/src/ad_juno/scripts/autostart.sh"

# Enable and start the service
sudo systemctl daemon-reload
sudo systemctl enable autonomous_driving.service
sudo systemctl start autonomous_driving.service

echo "Autostart service has been set up. The autonomous driving system will start automatically on boot."
