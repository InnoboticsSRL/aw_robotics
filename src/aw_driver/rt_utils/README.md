# rt_utils

Copyright (c) 2022, Automationware ( HIND Spa Group)

This package includes scripts that can be used to tune the realtime-behavior of a linux system.

#### Settings for activate_all.sh

Open the script `activate_all.sh` and modify the following lines 

```
# Settings: have to be changed manually
cpu_id="4"  # the isolated CPU onto which irqs should be isolted to
nic_device="eno2"  # ths NIC device's irqs will be isolated to 'cpu_id'

```

### Run

The scripts can be executed separately, or called all together. Make sure all of them are executeable through:
```
sudo chmod +x script_name.sh
```
Run all scripts:
```
sudo ./activate_all.sh
```

