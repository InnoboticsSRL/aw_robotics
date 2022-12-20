#!/bin/bash
# This bash script can be used to turn the Wifi in Ubuntu on and off
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Author:  Aw Robotics (robotics@automationware.it)

if [ $# -ne 1 ]
  then
    echo "Error: Wrong number of input arguments provided: 1 required, '$#' provided!"
    echo "Usage: '$ ./toggle_wifi.sh state'"
    exit 1
fi

arg=$1
if [ "$arg" == "on" ]
  then
    echo "Turning on wifi..."
    nmcli radio wifi on
elif [ "$arg" == "off" ]
  then
    echo "Turning off wifi..."
    nmcli radio wifi off
else
  echo "Error: Argument '$arg' not recognised: Argument can be either 'on' or 'off' only."
  exit 1
fi

echo "Done!"
