#!/bin/bash

echo "start copy rplidar.rules to  /etc/udev/rules.d/"
echo "cp ./rplidar.rules /etc/udev/rules.d"
sudo cp ./rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
