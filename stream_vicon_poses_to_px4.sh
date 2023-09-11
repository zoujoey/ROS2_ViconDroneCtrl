#!/bin/bash
micrortps_agent start -d /dev/ttyACM0 -b 57600 & 
ros2 run vicon_position_bridge pose_sub
