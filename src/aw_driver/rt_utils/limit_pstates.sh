#!/bin/bash
# This bash script sets the frequency scaling given by the so called P-states (cpufreq in Linux).
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Authors: Aw Robotics <robotics@automationware.it>,  Aw Robotics (robotics@automationware.it)

# Check if script is run as root
if [ "$EUID" -ne 0 ]
  then
    echo "Error: This script has to be run as root '$ sudo ./limit_pstates.sh cpu_id state'!"
    exit 1
fi

# Check if one argument provided
if [ $# -ne 2 ]
  then
    echo "Error: Wrong number of input arguments provided: 2 required, '$#' provided!"
    echo "Usage: '$ sudo ./limit_pstates.sh cpu_id state'"
    exit 1
fi

mode="performance"
if [ "$2" == "off" ]
  then
    mode="powersave"
elif [ "$2" == "on" ]
  then
    mode="performance"
else 
  echo "Error: Argument 2 state ('$2') invalid: Can be either 'on' or 'off' only!"
  exit 1
fi

# Parse input argument
cpu_id=$1

# Set CPU frequency
echo ""
echo "Setting CPU frequency to mode '$mode'..."

cpu_cores=$(grep -c ^processor /proc/cpuinfo)
if [ "$cpu_id" == "all" ]
  then
    for (( i=0; i<$cpu_cores; i++ ))
    do
      echo "Setting CPU frequency of CPU '$i' to mode '$mode'..."
      echo $mode > /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
    done

  echo "Done!"
else
  if ! [[ $cpu_id =~ ^-?[0-9]+$ ]]
    then
      echo "Error: Numeric expression for argument 1 cpu_id required, '$cpu_id' provided!"
      exit 1
  elif [[ "$cpu_id" -lt "0" ]]
    then
      echo "Error: Argument 1 cpu_id ('$cpu_id') has to be positive!"
      exit 1
  elif [[ "$cpu_id" -gt "$cpu_cores" ]]
    then
      echo "Error: Argument 1 cpu_id ('$cpu_id') has to be smaller than the available number of cores ($cpu_cores)!"
      exit 1
  fi
  
  echo "Setting CPU frequency of CPU '$cpu_id' to mode '$mode'..."
  echo $mode > /sys/devices/system/cpu/cpu$cpu_id/cpufreq/scaling_governor
  echo "Done!"
fi
