#!/bin/bash
#
# tinyUPS - Uninstall Script
#
# Make this script executable and start it from a terminal.

echo "Disable tinyUPS.service ..."
sudo systemctl disable tinyUPS.service

echo "Remove tinyUPS.service ..."
sudo rm /etc/systemd/system/tinyUPS.service

echo "Remove /opt/tinyUPS and the Python scripts ..."
sudo rm -r /opt/tinyUPS

echo "Done."
echo "Reboot the Pi to finish uninstallation!"
