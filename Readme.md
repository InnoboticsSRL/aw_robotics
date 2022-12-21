<a href="http://www.automationware.it/">
    <img src="doc/img/logoAW.png" alt="Aw logo" title="AutomationWare" align="right" height="40" />
</a>

Authors: 
- [Mattia Dei Rossi](deirossi@automationware.it) - Automationware
- [Armando Selvija](selvija@automationware.it) - Automationware
- [Michele Tasca](tasca@automationware.it) - Automationware
# Aw robotics

This workspace has been developed and tested on `ros melodic`.

## Requirements
- ros melodic http://wiki.ros.org/melodic/Installation/Ubuntu
- docker https://docs.docker.com/engine/install/ubuntu/
- docker privileges https://docs.docker.com/engine/install/linux-postinstall/
- rt patch reccomended ( not mandatory ) 
- `sudo apt install ros-melodic-pilz*`

## Compile
```bash
cd ~/aw_robotics
catkin build
source devel/setup.bash # for each terminal launched
```

## Launch AwJoint and Awtube on simulation
possible combination of `joint_id` with `joint_size`: 
* awjoint1750   - J17
* awjoint2080   - J20 
* awjoint2580   - J25
* awjoint32100  - J32
* awjoint40100  - J40

### Run tests

### AwJoint expected behaviour
<img src="./doc/img/j_actuator_test.gif" width="50%"/>

#### Command
```bash
roslaunch j_actuator_test test.launch sim:=true joint_id:=awjoint32100 joint_size:=J32
```

### Awtube expected behaviour
### Joint position Test
<img src="./doc/img/joint_position_test.gif" width="60%"/>

### Command
```bash
roslaunch awtube_test joint_position_test.launch sim:=true awtube_size:=M
```

### Target Pose test
<img src="./doc/img/pose_coords_test.gif" width="60%"/>

### Command
```bash
roslaunch awtube_test pose_coords_test.launch sim:=true awtube_size:=M
```

### Cartesian coordinate test
<img src="./doc/img/cartesian_coords_test.gif" width="60%"/>

### Command
```bash
roslaunch awtube_test cartesian_coords_test.launch sim:=true awtube_size:=M
```

## Launch AwJoint and Awtube on real hardware
### Setup:
- pull docker container
```bash
docker pull automationware/robovu
```
- make script executable inside aw_robotics workspace
```
cd ~/aw_robotics
chmod +x src/aw_driver/awtube_meta/scripts/robovu.sh
```
- configure realtime tuning script [rt_config](./src/aw_driver/rt_utils/README.md)

- run realtime tuning script
```
cd ~/aw_robotics/src/aw_driver/rt_utils
sudo ./activate_all.sh
```

### Run tests
- Configure awtube_meta pkg like [awtube_meta configuration](./doc/configuration_awtube_meta.md)

### Launch RoboVu with
```bash
roslaunch awtube_meta bringup.launch
```

- Launch AwTube test 
```bash
roslaunch awtube_test joint_position_test.launch sim:=false awtube_size:=M
```
```bash
roslaunch awtube_test pose_coords_test.launch sim:=false awtube_size:=M
```
```bash
roslaunch awtube_test cartesian_coords_test.launch sim:=false awtube_size:=M
```

- Launch AwJoint test 
```
roslaunch j_actuator_test test.launch sim:=false joint_id:=awjoint32100 joint_size:=J32
```

