#!/bin/bash
# This bash script can be used to isolate CPUs on the system permanently.
# The script takes two parameters: 'isolate_cpus.sh cpu_1 cpu_2' for isolating the CPUs permanently
# through the add of the isolcpus flag in the /etc/default/grub file.
# TODO: implement for n cpus.
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Authors: Aw Robotics <robotics@automationware.it>
# Credits:  Aw Robotics (robotics@automationware.it) 

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script has to be run as root '$ sudo ./isolate_irqs.sh nic_device cpu_id'!"
    exit 1
fi

# Check if CPUs are already id is isolated
isolated_cpu_ids=$(cat /proc/cmdline | grep isolcpus)' '
isolated_cpu_ids=$(echo $isolated_cpu_ids | awk 'BEGIN { RS=" " ; FS="=" } $1 ~ /isolcpus/ { print $2 }')
isolated_cpu_ids="${isolated_cpu_ids//$','/ }"
if [ ${#isolated_cpu_ids[@]} -ne 0 ]; then
    while true; do
        read -p "These CPUS are already isolated: $isolated_cpu_ids, do you want to override [y/n]? " yn
        case $yn in
            [Yy]* ) break;;
            [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

# Parse input arguments from command line
cpu_cores=$(grep -c ^processor /proc/cpuinfo)
echo "Number of available cpus: $cpu_cores"
echo "--> To improve realtime performances, you might want to isolate threads that are located on the same core."
echo "See here the output of the lscpu command" # todo this could be more automated here
echo " "
lscpu --all --parse=cpu,core
echo ""
echo "Please insert here desired numbers of CPUs to isoltate (e.g. for cpu 4 and 20: 4 20)"
read -ra array
echo ""
echo "Desired CPUs to isolate: ${#array[@]}"

# Check if at least one argument provided
if [ ${#array[@]} -lt 1 ]; then
    echo "Error: Wrong number of input arguments provided: 1 required, '$#' provided!"
    exit 1
fi


for i in ${#array[@]}; do
    if ! [[ $i =~ ^-?[0-9]+$ ]]; then
        echo "Error: Numeric expression for argument 2 cpu_id required, '$i' provided!"
        exit 1
        elif [[ "$i" -lt "0" ]]; then
        echo "Error: Argument 2 cpu_id ('$i') has to be positive!"
        exit 1
        elif [[ "$i" -gt "$cpu_cores" ]]; then
        echo "Error: Argument 2 cpu_id ('$i') has to be smaller than the available number of cores ($cpu_cores)!"
        exit 1
    fi
done

grep -n "GRUB_CMDLINE_LINUX" /etc/default/grub
var=$(sed -n '/GRUB_CMDLINE_LINUX/=' /etc/default/grub)

echo "$var"
# todo implement for n cpus.
sed -i '10s/.*/GRUB_CMDLINE_LINUX="isolcpus='${array[0]}','${array[1]}'"/' /etc/default/grub
grep -n "GRUB_CMDLINE_LINUX" /etc/default/grub
sudo update-grub
echo ""
echo "Please restart your PC to make changes become effective."