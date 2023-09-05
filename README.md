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

**Safety Guidelines**: Guidelines for flying the drone safely.
https://docs.google.com/document/d/1OmSO_3oGtAG3CVVjXAy_KHn_xMD42szhOk2jm1l0Ne0/edit

## Getting Started

Before the setup process, ensure all the Installation Requirements are met as below. and be sure to sanity check the installation by running a quick simulation. 

Before flying the drone, read the Safety Guidelines to ensure responsible drone operation. Once acquainted with the safety guidelines, proceed to the Quick Setup Guide for initial drone control setup.

For detailed instructions, customisation, troubleshooting, and a demonstration of the drone's capabilities, consult the other guides above.

## Requirements before starting 
### Hardware Requirements:
*Make sure that all the hardware components listed in the Drone_Components.jpg above are on the drone's top plate*  

Drone Model: **Holybro Pixhawk 6c Controller, Holybro x500 v2 model**
Onboard Model **Nvidia Jetson Nano**

*If you want to connect to the onboard computer through USB and power it through USB, you need to remove the jumper (at pins DC EN and D63).*

*username: rob498, password: 99bobotw*

### Software Requirements: 
*Make sure the software on the ground control and onboard computers are as follows*   

Operating System: **Ubuntu Linux, version 20**  
ROS2 Version: **ROS2 Foxy**  
Wifi Newtork: **TP-Link_ROB498**

*Make sure the firmware on the drone components are as follows*   

Controller Firmware Type, Version: **PX4 (with microRTPS), v1.13**  

### ROS2 Packages and other Software Requirements:  
*Install the following github repos and applications, and colcon build the respective ROS2 Packages. (GCS) means these parts have to be installed on the ground control station (the laptop) only, all other parts have to be installed on the onbard computer and GCS.*  

**ROS2 foxy:**  
Follow instructions on website below:  
https://docs.ros.org/en/foxy/Installation.html 

**QGroundControl for communicating with Drone Controller (GCS):**  
Follow instructions on website below:
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html    

**ROS2 nodes for controlling the drone (this repo):**  
Run the following command in terminal:
```
git clone --recursive https://github.com/zoujoey/ROS2_ViconDroneCtrl.git
```  

**Pip install dependencies**   
Run the following command in terminal   
```
pip install -r requirements.txt
```  

**ROS2 nodes for processing vicon position (GCS):**  
go to submodule `ros2-vicon-receiver` and follow its installation instructions.

**PX4_ROS_COM msgs & MicroRTPS Bridge for UorB Topics:**  
Below instructions are adopted from the following website, but simplified as the correct repos are already added as submodules to this repo.
https://docs.px4.io/v1.13/en/ros/ros2_comm.html  

1. Install Fast DDS using [this section](https://docs.px4.io/v1.13/en/ros/ros2_comm.html#install-fast-dds)

2. Build the px4_ros_com and px4_msgs packages:
```
cd px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```

**ROS2 package for building PX4 controller firmware/simulations (GCS):**  
Go to submodule `PX4-Autopilot` and follow installation instructions (see also below link).
https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html

**Build all packages:**
From the root of this repository, run
```
colcon build --symlink-install
source install/local_setup.bash
```
When you build one package only, you can run
```
colcon build --symlink-install --packages-select <package-name>
```

## Sanity Check Installation / Gazebo Simulation Guide

### Quick sanity check

As a first sanity check, you can run the below command and make sure it runs without errors.
```
ros2 launch drone_keyboard_controller control_launch.py 
```
This should output a stream similar to below. Make sure that when you press the key i, both KEY: and SET: change.
```
[command_control-2] POSE:0.0 0.0 0.0                                                          
[command_control-2] SET:0.0 0.0 -0.25                                                         
[command_control-2] [INFO] [1693946756.037120745] [OffboardControl]:                          
[command_control-2] KEY: z                                                                    
[command_control-2] REACHED/MODE:False True                                                   
[command_control-2] POSE:0.0 0.0 0.0                                                          
[command_control-2] SET:0.0 0.0 -0.25                                                         
[command_control-2] [INFO] [1693946756.039957106] [OffboardControl]:                          
[command_control-2] KEY: z                                                                    
[command_control-2] REACHED/MODE:False True                                                   
[command_control-2] POSE:0.0 0.0 0.0                                                          
[command_control-2] SET:0.0 0.0 -0.25                                                         
[command_control-2] [INFO] [1693946756.047085888] [OffboardControl]:                          
[command_control-2] KEY: z                                                                    
[command_control-2] REACHED/MODE:False True                                                   
[command_control-2] POSE:0.0 0.0 0.0  
```
When killing the node, you should see messages of 10 nodes that are killed.

### Simulation

To run a simulation with this pipeline, please follow the steps below:

#### Terminal 1: Starting the Simulation
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
This allows you to arm the drone in simulation even though it is not connected to a remote controller.

#### Terminal 2: Starting MicroRTPS Bridge
Open a second terminal, and source the following setup script:
```
source install/local_setup.bash
```
then, run the following command to start the microRTPS bridge
```
micrortps_agent -t UDP
```

#### Terminal 3: Starting fake vicon position lock

Run simulation_graphing_launch to start the Vicon bridge. In simulation, this will publish the visual odometry position of the drone. 
```
source install/local_setup.bash
ros2 launch vicon_position_bridge simulation_graphing_launch.py
```

#### Terminal 4: Starting the drone_keyboard_controller
In the last terminal, run the following commands, and the drone should start hovering after 30 seconds. If it does so, that means the pipeline is working correctly.  
*For more information on how to use the keyboard controller, see Quick-Start Guide*
```
source install/local_setup.bash
ros2 launch drone_keyboard_controller control_launch.py
```
## Contributors/Credits

The ROS2_ViconDroneCtrl package is developed by Joey Zou with support from ASRL (Autonomous Space Robotics Laboratory)
