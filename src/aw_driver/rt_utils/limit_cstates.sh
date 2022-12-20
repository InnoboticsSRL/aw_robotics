#!/bin/bash
# This bash script limits the sleep stages, the so called C-states (cpuidle in Linux) of a given computer.
# It copies the corresponding script to 'usr/local/bin/couidle' and launches it with the two given input arguments
# generally either '$ ./limit_cstates.sh all disable' or '$ ./limit_cstates.sh all enable'"
# Alternatively it could by called as a bash script directly.
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Authors: Aw Robotics <robotics@automationware.it>,  Aw Robotics (robotics@automationware.it)

# Check correct number of input arguments
if [ $# -ne 2 ]
  then
    echo "Error: Wrong number of input arguments provided: 2 required, '$#' provided!"
    echo "Usage: '$ ./limit_cstates.sh all disable'"
    exit 1
fi

# Check if file exists already
file="cpuidle"
src_dir="cpuidle-tools"
dst_dir="/usr/local/bin"

# Check if destination file exists already
src_file="$src_dir/$file"
dst_file="$dst_dir/$file"
if [ -f "$dst_file" ]
  then
    echo "File '$dst_file' found."
else
  echo "Warning: File '$dst_file' not found."
  
  # Check if directory exists
  if [ ! -d "$dst_dir" ]
    then
      echo "Error: Directory '$dst_dir' found."
      exit 1
  fi
  
  # Check if source file exists
  echo "Trying to copy file '$src_file' to '$dst_file'..."
  if [ -f "$src_file" ]
    then
      echo "File '$src_file' found."
      
      # Check if script is run as root
      if [ "$EUID" -ne 0 ]
        then
          echo "Error: In order to copy file '$src_file' to '$dst_file' this script has to be run as root '$ sudo ./limit_cstates.sh all disable'!"
          exit 1
      fi
      
      # Copy the source file to the destination
      sudo cp -p $src_file $dst_file
      sudo chmod +x $dst_file
      echo "File '$src_file' copied to '$dst_file' successfully."
  else
    echo "Error: File '$src_file' not found."
    exit 1
  fi
fi

# Run the script
sudo cpuidle $1 $2 || echo "Error: Could not find the 'cpuidle' tools! Please install them."
echo "Done!"
