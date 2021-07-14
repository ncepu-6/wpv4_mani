#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to rh56df3"
echo "start copy rh56df3.rules to  /etc/udev/rules.d/"
sudo cp `rospack find rh56df3_bringup`/scripts/rh56df3.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
