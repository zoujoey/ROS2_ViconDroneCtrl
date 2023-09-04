# ROS2_ViconDroneCtrl

Welcome to the Holybro Demo Repository— a collection of ROS2 packages, executables, and configuration files for streamlined drone control using the My580 vicon system. 

## Repo Overview

This repo provides the user with 3 ways to control the drone:
- Using the communication bridge from the vicon system to the drone controller via the **vicon_position_bridge** package, the user may fly the drone manually with an ***RC controller*** in *position lock mode*.
- Using a ros2 launch file and configuration file from the **drone_keyboard_controller** package, the user may flexibly run customizable autonomous missions or manually using the ***computer's keyboard***, all while flying in the drone's *offboard mode*.
- Using the singular ros2 nodes from the **drone_offboard_controller** package, the user may fly various autonomous missions using the ***computer terminal*** to fly the drone in *offboard mode*. 
(*Using the **drone_offboard_controller** package will require the user to be familiar with how the PX4 firmware and ROS2 integration works.*)
## Guides
Each of the following guides can be found in the pdfs in the root of the repo or through a link to an editable google docs below.

**Quick Start Guide**: Instructions for swift setup and drone control.  
https://docs.google.com/document/d/1MAl2qnRF_2aW_TNH4cIVihMYBwpg-J_iwrsse65H9UI/edit?usp=sharing 

**Installation/Calibration Guide**: Detailed setup instructions for the drone control 
environment.  
https://docs.google.com/document/d/1vBU4GRG0dCPRwetB_ik6oC_IXExsfwYnezXxN3A-flA/edit#heading=h.v69talx8uz7b 

**Customization/Pipeline Guide**: Modify drone behaviour/parameters in ROS2 Pipeline as needed.  
https://docs.google.com/document/d/1OmSO_3oGtAG3CVVjXAy_KHn_xMD42szhOk2jm1l0Ne0/edit

**Troubleshooting Guide**: Solutions for common problems.  
https://docs.google.com/document/d/1OmSO_3oGtAG3CVVjXAy_KHn_xMD42szhOk2jm1l0Ne0/edit
## Getting Started

Before the setup process, ensure all the Installation Requirements are met as below. and be sure to sanity check the installation by running a quick simulation. 

Before flying the drone, Read the Safety Guidelines to ensure responsible drone operation. Once acquainted with the safety guidelines, proceed to the Quick Setup Guide for initial drone control setup.

For detailed instructions, customisation, troubleshooting, and a demonstration of the drone's capabilities, consult the other guides above.

## Requirements before starting 
### Hardware Requirements:
*Make sure that all the hardware components listed in the Drone_Components.jpg above are on the drone's top plate*  

Drone Model: **Holybro Pixhawk 6c Controller, Holybro x500 v2 model**

### Software Requirements: 
*Make sure the software on the ground control and onboard computers are as follows*   

Operating System: **Ubuntu Linux, version 20**  
ROS2 Version: **ROS2 Foxy**  
Wifi Newtork: **TP-Link_ROB498**

*Make sure the firmware on the drone components are as follows*   
Controller Firmware Type, Version: **PX4 (with microRTPS), v1.13**  
Current Controller

### ROS2 Packages and other Software Requirements:  
*Install the following github repos and applications, and colcon build the respective ROS2 Packages*  
**ROS2 foxy:**  
follow instructions on website below (install in home directory)  
(Install on Ground Control Computer + Onboard)  
https://docs.ros.org/en/foxy/Installation.html 

**QGroundControl for communicating with Drone Controller:**  
follow instructions on website below (install in home directory)  
(Install Ground Control Computer only)
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html    

**ROS2 nodes for processing vicon position:**  
run the following command in terminal (install in home directory)  
(Install Ground Control Computer Only)
```
git clone https://github.com/OPT4SMART/ros2-vicon-receiver/tree/master
```    
**PX4_ROS_COM msgs & MicroRTPS Bridge for UorB Topics:**  
follow instructions on website below (install in home directory)  
(Install on Ground Control Computer + Onboard)  
https://docs.px4.io/v1.13/en/ros/ros2_comm.html  

**ROS2 package for building PX4 controller firmware/simulations:**  
run the following command in terminal (install in home directory)  
(Install Ground Control Computer Only)

```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```   
**ROS2 nodes for controlling the drone (this repo):**  
run the following command in terminal (install in home directory)  
(Install on Ground Control Computer + Onboard)  
```
git clone https://github.com/zoujoey/ROS2_ViconDroneCtrl.git
```  
**Pip install dependencies**   
run the following command in terminal after navigating to ROS2_ViconDroneCtrl directory   
(Install on Ground Control Computer + Onboard)  
```
pip install -r requirements.txt
```  
## Sanity Check Installation / Gazebo Simulation Guide
To make sure all the packages were correctly installed or to run a simulation with this pipeline, please follow the steps below:

### Terminal 1: Starting the Simulation
Open a terminal and cd into the PX4-Autopilot Directory that was just installed.

In that directory, run the following command to start the Gazebo Simulation:

```
make px4_sitl_rtps gazebo
```
Once the microRTPS bridge is started in the next step, come back to this terminal and run the following commands in the pxh terminal:
```
param set COM_RCL_EXCEPT 4
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

### Terminal 2: Starting MicroRTPS Bridge
Open a second terminal, and source the following setup scripts:

```
source ~/ros2-vicon-receiver/vicon_receiver/install/setup.bash
source ~/ROS2_ViconDroneCtrl/flight_controller_ws/install/setup.bash
source ~/px4_ros_com_ros2/install/setup.bash
```
then, run the following command to start the microRTPS bridge
```
micrortps_agent -t UDP
```

### Terminal 3: Starting fake vicon position lock
Before opening a third terminal, go into the following directory:
```
ROS2_ViconDroneCtrl/flight_controller_ws/src/vicon_position_bridge/launch
```
Open the launch file graphing_launch.py, and set the 'simulation' parameter to True and save the file.   
*Be sure to set this parameter back to False once flying the physical drone*

Navigate back to the workspace directory, remove build and install folders, colcon build, source necessary setup scripts, and launch the fake position lock:
```
cd ../../..
rm -rf build install
colcon build --symlink-install
source ~/ros2-vicon-receiver/vicon_receiver/install/setup.bash
source ~/ROS2_ViconDroneCtrl/flight_controller_ws/install/setup.bash
source ~/px4_ros_com_ros2/install/setup.bash
ros2 launch vicon_position_bridge graphing_launch.py
```

### Terminal 4: Starting the drone_keyboard_controller
In the last terminal, run the following commands, and the drone should start hovering after 30 seconds. If it does so, that means the pipeline is working correctly.  
*For more information on how to use the keyboard controller, see Quick-Start Guide*
```
source ~/ros2-vicon-receiver/vicon_receiver/install/setup.bash
source ~/ROS2_ViconDroneCtrl/flight_controller_ws/install/setup.bash
source ~/px4_ros_com_ros2/install/setup.bash
ros2 launch drone_keyboard_controller control_command.py
```
## Contributors/Credits

The ROS2_ViconDroneCtrl package is developed by Joey Zou with support from ASRL (Autonomous Space Robotics Laboratory)
