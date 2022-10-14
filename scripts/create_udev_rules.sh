#!/bin/bash

echo "start copy arducam.rules to  /etc/udev/rules.d/"
echo "arducam.rules"
sudo cp arducam.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
