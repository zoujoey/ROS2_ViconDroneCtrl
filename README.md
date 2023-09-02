# ROS2_ViconDroneCtrl

Welcome to the Holybro Demo Repository— a collection of ROS2 packages, executables, and configuration files for streamlined drone control using the My580 vicon system. 

## Packages

**vicon_position_bridge**: vicon communication pipeline to drone controller

**drone_offboard_missions**: singular executable autonomous missions

**drone_keyboard_controller**: keyboard controller with configurable autonomous missions

## Guides

**Quick Setup Guide**: Instructions for swift setup and drone control.

**Installation Guide**: Detailed setup instructions for the drone control 
environment.

**Customization/Tuning Guide**: Modify drone behaviour/parameters as needed.

**Safety Guide**: Essential safety guidelines for responsible drone operation.

**Troubleshooting Guide**: Solutions for common problems.

## Getting Started

Before you begin the setup process, ensure you are familiar with the Safety Guidelines to ensure responsible drone operation. Once you are acquainted with the safety guidelines, proceed to the Prequisite section below, and then to Quick Setup Guide for initial drone control setup.   
For detailed instructions, customisation, troubleshooting, and a demonstration of the drone's capabilities, consult the sections in the table of contents.

## Prequisites before starting 
### Hardware Prequisite:

### Software Prequisites: 
*Make sure the software on the ground control and onboard computers are as follows*   

Operating System: **Linux**  
ROS2 Version: **ROS2 Foxy**  
Wifi Newtork: **TP-Link_ROB498**

*Make sure the firmware on the drone controller are as follows*   
Controller Firmware Type, Version: **PX4 (with microRTPS), v1.13**  

### ROS2 Packages Prequisites:  
*Install the following github repos:*  

**ROS2 nodes for processing vicon position:**
```
git clone https://github.com/OPT4SMART/ros2-vicon-receiver/tree/master
```    
**ROS2 msg for PX4 MicroRTPS UorB Topics:**  
```
git clone https://github.com/PX4/px4_ros_com/tree/release/1.13
```   
**ROS2 package for building PX4 controller firmware/simulations:**
```
git clone https://github.com/PX4/PX4-Autopilot 
```   
**ROS2 nodes for controlling the drone (this repo):**
```
git clone https://github.com/PX4/px4_ros_com.git 
```    

## Contributors

The Holybro-Demo package is developed by: Joey Zou 