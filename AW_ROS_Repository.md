<a href="http://www.automationware.it/">
    <img src="ROS/logo_automationware3.png" alt="logoAW" title="AutomationWare" align="right" height="40" />
</a>

Author: [Michele Tasca](tasca@automationware.it) - Automationware

# AW Robotics ROS Architecture

<img src="ROS/RoboVu_Arch_noSoem.png" alt="RoboVu_Arch" title="RoboVu_Arch" align="center"/>

## RoboVu

RoboVu is a software interface, developed by Automationware, to connect ROS with EtherCAT driven J-Actuators. Cyclically it translates the motion information planned by ROS in EtherCAT datagrams to manage EtherCAT drivers and it reads information from each EtherCAT Slave (eg. actual position, torque, velocity values) to send them to ROS Control.

RoboVu implements EtherCAT Master functionalities using a proper library.

It runs on ROS Melodic installed in a Linux Ubuntu 18.04 LTS equipped with a «RT Patch» to allow realtime performances.

RoboVu is published in a proper Docker container downloadable at the following link:

https://hub.docker.com/r/automationware/robovu

In `Readme.md` file users can found the instructions to download and launch it. 

## J-Actuator

J-Actuators are robotic joints developed by AutomationWare.
From a mechanical point of view, J-Actuators are in large parte designed, produced and assembled internally in Automationware.
Each J-Actuator is equipped with an electronic motor control board and 2 encoders provided by third parts. 
Each J-Actuator is an EtherCAT Slave and communicates with an EtherCAT Master (RoboVu) through EtherCAT fieldbus.
There are 6 sizes of J-Actuators:
 
| J-Actuator model  | aw_robotics ID |
| -----------       | -----------    |
| J17               | awjoint1750    |
| J20               | awjoint2080    |
| J25               | awjoint2580    |
| J32               | awjoint32100   |
| J40 LP            | awjoint40100   |
| J40 HP            | awjoint40100   |

<img src="ROS/j_actuator.png" alt="j_actuator" title="j_actuator" align="center" height="300"/>

## AWTube 

AWTube is 6-DOF robotic arm prototype, composed by 6 J-Actuators of different sizes linked to each other.
AWTube is modular because the composition of J-Actuators can be customized depending on the project requirements.
For AWTube 3 sizes are available:

| AWTube model| aw_robotics ID |
| ----------- | -----------    |
| S           | awtube30808v1  |
| M           | awtube31210v1  |
| L           | awtube31814v2  |


<img src="ROS/Robot Size M.png" alt="Robot Size M" title="Robot Size M" align="center" height="300"/>

## Description of aw_robotics
Automationware published on GitHub the repository called `aw_robotics` with the aim to allow users to test the ROS packages developed for managing J-Actuator and AWTube models.

In `aw_robotics/src/aw_descriptions/` are published the description packages (including URDF and meshes files) for each J-Actuator and AWTube models. These files are derived from the related CAD projects.

In `aw_robotics/src/moveit_configs/` are published the moveit_config packages for the 3 AWTube sizes and the package `j_actuator_moveit_config` which allow the user to manage a single J-Actuator model depending on the parameter previously selected.

In `aw_robotics/src/aw_tests/` are published 3 test nodes for AWTube and 1 test node for J-Actuator.
For further information, please refer to `Readme.md`.

## Realtime PC configuration for RoboVu
For convenience the name ROS_MASTER_PC will be used to refer to the pc in which RoboVu runs and it becomes an EtherCAT Master device. This implies that it needs to reach at least "soft realtime" performance to manage adequately the connected EtherCAT Slaves.
The EtherCAT Master implemented in RoboVu exchanges EtherCAT datagrams with a frequency of 1 kHz. If the system is affected by high values of jitter (eg. >20us) RoboVu can terminates the communication indicating that **overrun** events occurred. 
Bad realtime perfomances can lead to undesiderable vibrations during the actuator working.

**Users shall configure autonomously their ROS_MASTER_PC devices to reach acceptable realtime performances.**

AutomationWare merely make some suggestions:
- run RoboVu in a dedicated pc partition of ROS_MASTER_PC in which Ubuntu 18.04 LTS is installed
- install RT-patch (eg. PREEMPT_RT) and proper cyclictest results
- launch the script file `activate_all.sh`, contained in `rt_utils`, to configure ROS_MASTER_PC to reach "soft realtime" perfomances. The related instructions are contained in `rt_utils/Readme.md`.

## Notes and warnings

  <img src="ROS/warning.png" alt="warning" title="warning" align="center" height="50"/>

1. All files and documents contained in `Automationware/aw_robotics` are     prototypes:

    - AutomationWare distributed them in the hope that they will be useful, but WITHOUT ANY WARRANTY.
    - Automationware is not responsible for damage to persons or things due to the usage of the files contained in `Automationware/aw_robotics`.
2. RoboVu works only with J-Actuators provided by AutomationWare.

   If a different EtherCAT Slave is connected to RoboVu the following error will be shown on the terminal:

    ```bash
    terminate called after throwing an instance of 'ethercat::EtherCatError'
      what():  [ethercat_client]: Sy Client not supported
    ```
    If J-Actuators are connected to RoboVu the right message is similar to the following one:

    ```bash
    [ INFO] [1670931103.943257051]: [ethercat_manager]: ========== Summary: =========
    [ INFO] [1670931103.943289022]:             ------->clients:1
    [ INFO] [1670931103.943309912]:             ------->operation mode: cyclic_sync_pos_mode
    [ INFO] [1670931103.943328055]:             ------->ethercat_thread frequency[hz]: 1000
    [ INFO] [1670931103.943357811]: [ethercat_manager]: Change ethercat-manager-state to ACTIVE 
    ```

