#!/bin/bash
#
# tinyUPS - Install Script
#
# Make this script executable and start it from a terminal.

# Move to script's directory
cd "`dirname "$0"`"

echo "Create folder /opt/tinyUPS ..."
sudo mkdir /opt/tinyUPS

echo "Copy scripts to /opt/tinyUPS ..."
sudo cp tinyUPSshutdown.py /opt/tinyUPS
sudo cp tinyUPSrequest.py /opt/tinyUPS
sudo cp tinyUPSshutdown.sh /opt/tinyUPS

echo "Make scripts executable ..."
sudo chmod +rx /opt/tinyUPS/tinyUPSshutdown.py
sudo chmod +rx /opt/tinyUPS/tinyUPSrequest.py
sudo chmod +rx /opt/tinyUPS/tinyUPSshutdown.sh

echo "Copy tinyUPS.service to /etc/systemd/system ..."
sudo cp tinyUPS.service /etc/systemd/system

echo "Enable tinyUPS.service ..."
sudo systemctl enable tinyUPS.service

echo "Done."
echo "Reboot the Pi to finish installation!"
