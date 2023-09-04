# ROS2_ViconDroneCtrl

Welcome to the Holybro Demo Repository— a collection of ROS2 packages, executables, and configuration files for streamlined drone control using the My580 vicon system. 

## Packages

**vicon_position_bridge**: vicon communication pipeline to drone controller

**drone_offboard_missions**: singular ROS2 executable autonomous missions

**drone_keyboard_controller**: keyboard controller with configurable autonomous missions

## Guides

**Quick Setup Guide**: Instructions for swift setup and drone control.

**Installation/Calibration Guide**: Detailed setup instructions for the drone control 
environment.

**Customization/Pipeline Guide**: Modify drone behaviour/parameters in ROS2 Pipeline as needed.

**Troubleshooting Guide**: Solutions for common problems.

## Getting Started

Before the setup process, ensure all the Environment Requirements are met as below. Read the Safety Guidelines to ensure responsible drone operation. Once acquainted with the safety guidelines, proceed to the Quick Setup Guide for initial drone control setup.
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
follow instructions on website below  
(Install on Ground Control Computer + Onboard)  
https://docs.ros.org/en/foxy/Installation.html 

**QGroundControl for communicating with Drone Controller:**  
follow instructions on website below  
(Install Ground Control Computer only)
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html    

**ROS2 nodes for processing vicon position:**  
run the following command in terminal  
(Install Ground Control Computer Only)
```
git clone https://github.com/OPT4SMART/ros2-vicon-receiver/tree/master
```    
**ROS2 msg for PX4 MicroRTPS UorB Topics:**  
follow instructions on website below under "Build ROS2 Workspace"  
(Install on Ground Control Computer + Onboard)  
https://docs.px4.io/v1.13/en/ros/ros2_comm.html  

**ROS2 package for building PX4 controller firmware/simulations:**  
run the following command in terminal  
(Install Ground Control Computer Only)

```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```   
**ROS2 nodes for controlling the drone (this repo):**  
run the following command in terminal  
(Install on Ground Control Computer + Onboard)  
```
git clone https://github.com/zoujoey/ROS2_ViconDroneCtrl.git
```    

## Contributors/Credits

The ROS2_ViconDroneCtrl package is developed by Joey Zou with support from ASRL (Autonomous Space Robotics Laboratory)
