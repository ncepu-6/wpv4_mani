#!/bin/bash

echo "***************"
echo "delete the remap device serial port to rh56df3"
sudo rm   /etc/udev/rules.d/rh56df3.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
