#!/bin/bash

echo "sudo rm /etc/udev/rules.d/rplidar.rules"
sudo rm /etc/udev/rules.d/rplidar.rules
echo " "
echo "restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "udev restarted"
