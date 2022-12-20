#!/bin/bash
# This bash script can be used to activate all realtime optimisations
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Author:  Aw Robotics (robotics@automationware.it)

# Settings: have to be changed manually
cpu_id="3"
nic_device="eno2"

# Check if script is run as root
if [ "$EUID" -ne 0 ]
  then
    echo "Error: This script has to be run as root '$ sudo ./activate_all'!"
    exit 1
fi

# Call scripts
echo "Turning off Wifi..."
sudo ./toggle_wifi.sh off
echo ""
echo "Isolating IRQs..."
sudo ./isolate_irqs.sh $nic_device $cpu_id
echo ""
echo "Limiting C-states..."
sudo ./limit_cstates.sh all disable
echo ""
echo "Limiting P-states..."
sudo ./limit_pstates.sh all on
echo ""
echo "Done!"
