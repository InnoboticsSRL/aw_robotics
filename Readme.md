<a href="http://www.automationware.it/">
    <img src="doc/img/logoAW.png" alt="Aw logo" title="AutomationWare" align="right" height="40" />
</a>

Authors: 
- [Mattia Dei Rossi](deirossi@automationware.it) - Automationware
- [Armando Selvija](selvija@automationware.it) - Automationware
- [Michele Tasca](tasca@automationware.it) - Automationware
# Aw robotics

This workspace has been developed and tested on `ros melodic`.

## Compile
```bash
cd ~/aw_robotics
catkin build
source devel/setup.bash # for each terminal launched
```

## Launch AwJoint simulation  
possible combination of `joint_id` with `joint_size`: 
* awjoint1750   - J17
* awjoint2080   - J20 
* awjoint2580   - J25
* awjoint32100  - J32
* awjoint40100  - J40

```bash
roslaunch j_actuator_moveit_config demo.launch joint_id:=awjoint32100 joint_size:=J32
```
### Launch with pilz pipeline
```bash
roslaunch j_actuator_moveit_config demo.launch joint_id:=awjoint32100 joint_size:=J32 pipeline:=pilz_industrial_motion_planner
```

## Launch Awtube simulation
### Awtube S
```bash
roslaunch awtube30808v1_moveit_config demo.launch
```
### Awtube M
```bash
roslaunch awtube31210v1_moveit_config demo.launch
```
### Awtube L
```bash
roslaunch awtube31814v2_moveit_config demo.launch
```

### Launch with pilz pipeline
```bash
roslaunch awtube31814v2_moveit_config demo.launch pipeline:=pilz_industrial_motion_planner
```

## Launch AwJoint demo

### Expected behaviour
<img src="/doc/img/j_actuator_test.gif" width="300" height="300"/>

### Commmand
```bash
roslaunch j_actuator_test test.launch
```

## Launch AwTube demo
### Joint position Test
<img src="/doc/img/joint_position_test.gif" width="300" height="300"/>

### Commmand
```bash
roslaunch awtube_test joint_position_test.launch
```

### Target Pose test
<img src="/doc/img/pose_coords_test.gif" width="300" height="300"/>

### Commmand
```bash
roslaunch awtube_test pose_coords_test.launch
```

### Cartesian coordinate test
<img src="/doc/img/cartesian_coords_test.gif" width="300" height="300"/>

### Commmand
```bash
roslaunch awtube_test cartesian_coords_test.launch
```