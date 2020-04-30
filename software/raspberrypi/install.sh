#!/bin/bash
#
# tinyUPS - Install Script
#
# Make this script executable and start it from a terminal.

# Move to script's directory
cd "`dirname "$0"`"

echo "Create folder /opt/tinyUPS ..."
sudo mkdir /opt/tinyUPS

echo "Copy Python scripts to /opt/tinyUPS ..."
sudo cp tinyUPSshutdown.py /opt/tinyUPS
sudo cp tinyUPSrequest.py /opt/tinyUPS

echo "Make Python scripts executable ..."
sudo chmod +x /opt/tinyUPS/tinyUPSshutdown.py
sudo chmod +x /opt/tinyUPS/tinyUPSrequest.py

echo "Copy tinyUPS.service to /etc/systemd/system ..."
sudo cp tinyUPS.service /etc/systemd/system

echo "Enable tinyUPS.service ..."
sudo systemctl enable tinyUPS.service

echo "Done."
echo "Reboot the Pi to finish installation!"
