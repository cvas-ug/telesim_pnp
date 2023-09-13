# Telesim PnP

Master package for Telesim, a modular and plug and play teleoperation system, that can be used with any kind of robotic arms. This repository contains both Baxter and UR Robot. Seperate version containing only one robot are available for [Baxter](https://github.com/09ubberboy90/telesim_pnp_baxter.git) or [UR3](https://github.com/09ubberboy90/telesim_pnp_ur.git).

TODO: Insert Picture

## Paper Abstract

This repository is published along with a paper [A Modular and Plug-and-Play Framework for Robotic Arm Teleoperation using a Digital Twin](TODO). Please cite it if you use part of this package.

TODO: Include abstract and paper link

## Needed packages

- Omniverse Isaac Sim
- SteamVR
- ROS1 Noetic
- ROS2 Galactic (May work on other version but has not been tested)

## Installation

This repository contains multiple submodules of the needed pakckages. Please use:

`git clone --recurse-submodules git@github.com:09ubberboy90/telesim_pnp`

or

`git clone --recurse-submodules https://github.com/09ubberboy90/telesim_pnp.git`

to download all of them

Navigate to the ROS1 folder and use catkin_make to build the workspace. Make sure you have sourced ROS Noetic before doing so.

Navigate to the ROS2 folder and use colcon build to build the workspace. Make sure you have sourced ROS Galactic before doing so. ROS noetic needs to be installed to build any of the baxter packages but does not need to be sourced

Source the local workspace and ROS2 for all the following steps except Isaac Sim.

## How to Use

More detailed version of the each part of setup can be found on the individual repositories.

### [VR](https://github.com/09ubberboy90/vr_publisher)

Run SteamVR and make sure to reset the origin of the headset.

Build and source the workspace using `colcon build`

Run this package by using:

`ros2 run vr_publish vr_publish`

### [Isaac Sim](https://github.com/09ubberboy90/telesim_isaac)

BEWARE: Do not source ROS or have ROS sourced for the following !

Find your isaac sim python path; It should be in `~/.local/share/ov/pkg/isaac_sim-2022.2.0/python.sh`. It will be referred henceforth as `isp`

Then you need to export the path of the packages you are going to use so that Isaac Sim can load them.

`export ROS_PACKAGE_PATH=ROS_PACKAGE_PATH:/opt/ros/galactic/share`

Note: This will only load the packages installed through APT not the local packages. You need to add them manually (See below for example)

#### First Time Usage

Make sure you also have installed pyquaternion for Isaac Sim. This can be done by running:

`isp -m pip install pyquaternion`

Make sure you have updated the urdf and rmp path according to your need in either [ur_world.py](ur3/ur_world.py) or [baxter_world.py](baxter/baxter_world.py) file. They are defined in the init as `self.urdf` and `self.rmp` respectively

#### Baxter

To add the packages needed for Baxter:

`export ROS_PACKAGE_PATH=ROS_PACKAGE_PATH:{ros_ws}/install/rethink_ee_description/share:{ros_ws}/install/baxter_description/share`

To run for baxter

`isp ROS2/src/isaac_sim/baxter/baxter_world.py`

#### UR

To add the packages needed for UR3 with the T42 gripper:

`export ROS_PACKAGE_PATH=ROS_PACKAGE_PATH:{ros_ws}/install/t42_gripper_description/share`

To run for the UR3

`isp ROS2/src/isaac_sim/ur/ur_world.py`

## Real robot control

### UR

Make sure you have installed [ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) for UR and followed the instructions there.

Once this is done you can connect to the real robot by running:

`ros2 launch ur_bringup ur_control.launch.py ur_type:={ur_type} robot_ip:={robot_ip} launch_rviz:=true`

Note: `ur_type` needs to be ur3 for this package. But additional type of UR robot can be used by creating another package similar to [this](https://github.com/09ubberboy90/ur_robotiq) for the UR5 and Robotiq Gripper. More instruction on how to add new robots to the system is available [here](https://github.com/09ubberboy90/telesim_isaac/blob/master/README.md#adding-new-robots).

In another terminal run:

`ros2 control switch_controllers --start forward_position_controller --stop scaled_joint_trajectory_controller`

And make sure you have URCap setup on the real robot.

Once this is done you can start transmitting joint position from isaac sim to the robot using:

`ros2 run ur_isaac joint_controller`

### Baxter

Baxter needs to have some terminal using ROS1 and ROS2

#### ROS1

Run the following command

``` sh
source ROS1/devel/setup.zsh
export ROS_IP="{Your PC IP}"
export ROS_MASTER_URI="{Baxter IP}"
rosrun baxter_tools enable_robot.py -e
```

#### ROS2

Run the following command

```sh
export ROS_IP="{Your PC IP}"
export ROS_MASTER_URI="{Baxter IP}"
ros2 launch baxter_bridge baxter_bridge_launch.py
```

and in another terminal

`ros run baxter_joint_controller controller`

## Senseglove

For the senseglove controller additional setup is needed:

[Sensecom](ROS2/src/senseglove_ros2_ws/SenseCom/Linux/SenseCom.x86_64) needs to be running

`ros2 launch senseglove_launch senseglove_hardware_demo.launch.py`

and in another terminal

`ros2 run t42_gripper_controller gripper_controller`

## T42 gripper

Master and Slave from [here](ROS2/src/t42_gripper/t42_gripper_controller/arduino) needs to be compiled on 2 different arduino connected through I2C. One of the arduino needs to be connected to the PC through USB, while the other needs to be connected to 2 Dynamyxel using the Dynamyxel shield.

You need to update the port in the [position_controller](ROS2/src/t42_gripper/t42_gripper_controller/t42_gripper_controller/position_controller.py) and then you can run.

`ros2 run t42_gripper_controller joint_sim_controller`
