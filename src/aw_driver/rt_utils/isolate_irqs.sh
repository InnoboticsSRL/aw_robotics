#!/bin/bash
# This bash script can be used to run IRQs of a specified NIC-device nic_device on a specific (possible isolated) CPU cpu_id
# The script takes two parameters: 'isolate_irqs.sh nic_device cpu_id' for isolating the IRQs or none for auto-balance mode.
# Possible future update: Create a one argument 'auto' mode that binds all the IRQs of the enp... NICs to the first isolated CPU core.
#   Copyright (c) 2022, Automationware ( HIND Spa Group)

# Authors: Aw Robotics <robotics@automationware.it>,  Aw Robotics (robotics@automationware.it)
# Credits https://github.com/OpenEtherCATsociety/SOEM/issues/171#issue-316624363

# Check if script is run as root
if [ "$EUID" -ne 0 ]
  then
    echo "Error: This script has to be run as root '$ sudo ./isolate_irqs.sh nic_device cpu_id'!"
    exit 1
fi

# Check if no arguments provided
if [ $# -eq 0 ]
  then
    echo "Restarting IRQ auto-balance..."
    service irqbalance restart
    echo "Done!"
    #service irqbalance status
    exit 0
fi

# Check if two arguments provided
if [ $# -ne 2 ]
  then
    echo "Error: Wrong number of input arguments provided: 2 required, '$#' provided!"
    echo "Usage to isolate IRQs: '$ sudo ./isolate_irqs.sh nic_device cpu_id'"
    echo "Usage to auto-balance IRQs: '$ sudo ./isolate_irqs.sh'"
    exit 1
fi

# Parse input arguments from command line
nic_device=$1
cpu_id=$2

# Check if NIC is available
nic_devices=$(ls /sys/class/net | grep enp)' '
nic_devices="${nic_devices//$'\n'/ }"
if [[ "$nic_devices" != *"$nic_device "* ]]
  then
    echo "Error: NIC-device '$nic_device' not found in nic_devices '$nic_devices'!"
    exit 1
fi

# Check if CPU id is plausible
cpu_cores=$(grep -c ^processor /proc/cpuinfo)
if ! [[ $cpu_id =~ ^-?[0-9]+$ ]]
  then
    echo "Error: Numeric expression for argument 2 cpu_id required, '$cpu_id' provided!"
    exit 1
elif [[ "$cpu_id" -lt "0" ]]
  then
    echo "Error: Argument 2 cpu_id ('$cpu_id') has to be positive!"
    exit 1
elif [[ "$cpu_id" -gt "$cpu_cores" ]]
  then
    echo "Error: Argument 2 cpu_id ('$cpu_id') has to be smaller than the available number of cores ($cpu_cores)!"
    exit 1
fi

# Check if CPU id is isolated
isolated_cpu_ids=$(cat /proc/cmdline |grep isolcpus)' '
isolated_cpu_ids=$(echo $isolated_cpu_ids | awk 'BEGIN { RS=" " ; FS="=" } $1 ~ /isolcpus/ { print $2 }')
isolated_cpu_ids="${isolated_cpu_ids//$','/ }"
if [[ "$isolated_cpu_ids" != *"$cpu_id "* ]]
  then
    echo "Warning: The CPU id '$cpu_id' does not belong to an isolated CPU! The isolated CPUs are '$isolated_cpu_ids'."
fi

# Isolate IRQs
echo ""
echo "Binding IRQS of NIC-device '$nic_device' on CPU number '$cpu_id'..." 

# Comment the following line if unwanted behaviour of the operating system is observed...
# Restart it by service irqbalance start
service irqbalance stop

for irq_id in $(cat /proc/interrupts | grep $nic_device | cut -d: -f1 | sed "s/ //g") ; do 
  echo "Mapping IRQ '$irq_id' of NIC-device '$nic_device' to CPU '$cpu_id'..."
  echo $cpu_id >/proc/irq/$irq_id/smp_affinity_list
  #irqbalance --banirq $irq_id
done

# Bring NIC interface down and up, in order to let irq switch the CPU
sudo ifconfig $nic_device down
sudo ifconfig $nic_device up

echo "Done!"
