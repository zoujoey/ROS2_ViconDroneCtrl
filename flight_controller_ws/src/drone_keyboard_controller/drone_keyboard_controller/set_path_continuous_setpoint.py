"""
ROS2 Node for sending trajectory setpoints for custom continuous trajectory (simply run the node when placing drone at starting point, sp)
(follows trajectory defined in param_trajectory.py)
"""


import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from px4_msgs.msg import VehicleVisualOdometry, Timesync, VehicleOdometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from math import sin, cos, tan, pi
from test_flight_controller_five.param_trajectory import position_function
import time
import math
import yaml
from std_msgs.msg import String
import os
import random

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.control_mode_callback = self.create_subscription(String,
                                                          "/Wifi/Channel_Five", self.control_mode_callback,10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/Wifi/Channel_Two", 10)
        
        self.position_callback = self.create_subscription(PoseStamped,
                                                          "/Wifi/Channel_One", self.position_callback,qos_profile)

        self.offboard_setpoint_counter_ = 0
        timer_period = 0.01  # 10 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.setpoint_reached_2 = 0
        self.pose = PoseStamped()
        self.setpoint_current = TrajectorySetpoint()
        self.setpoint_future = TrajectorySetpoint()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
        config_file_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config0.yaml')
        with open(config_file_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        self.config = config
        self.sp = (self.config['start']['spx'],self.config['start']['spy'],self.config['start']['spz'],self.config['start']['spyaw'])
        self.speed = self.config['continuous_setpoints']['speed']
        self.hz = self.config['continuous_setpoints']['hheight']
        self.tof = self.config['continuous_setpoints']['time_of_flight']*100
        self.bind = 'z'
    def setpoint_function(self, time, px, py, pz, speed):
        """
        This function projects the drone's current position onto the closest point
        on the trajectory defined by position_function and then translates that point
        a distance of speed (in meters) along the trajectory.
        """
        current_time_ms = int(time)  # Get current time in milliseconds

        # Search range for finding the closest point (within 1 second)
        end_time_ms = current_time_ms + 100

        min_distance = float('inf')
        min_distance_2 = float('inf')
        closest_time = None
        closest_time_2 = None

        for t_ms in range(current_time_ms, end_time_ms + 1, 2):
            x_traj = position_function(t_ms)[0]
            y_traj = position_function(t_ms)[1]
            z_traj = position_function(t_ms)[2]
            distance = math.sqrt((px - x_traj)**2+(py - y_traj)**2+(pz - z_traj)**2)
            if distance < min_distance:
                min_distance = distance
                closest_time = t_ms
            elif abs(distance-speed)<min_distance_2:
                min_distance_2 = abs(distance-speed)
                closest_time_2 = t_ms
        if min_distance_2<speed*1.5:
            return (position_function(closest_time_2)) 
        else:
            return (position_function(closest_time))
    def control_mode_callback(self, datum:String):
        self.bind = datum.data
    def timer_callback(self):
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        pz = self.pose.pose.position.z
        if self.offboard_setpoint_counter_<=self.tof:
            posec = position_function(self.offboard_setpoint_counter_)
            posef = self.setpoint_function(self.offboard_setpoint_counter_, px, py, pz, self.speed)
        else:
            posec = position_function(self.offboard_setpoint_counter_)
            poseff = (self.sp[0],self.sp[1],self.sp[2] + self.hz, self.sp[3])
            posef = self.tpol(posec[0], posec[1], posec[2], poseff[0], poseff[1], poseff[2], 
                           self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, 
                           self.speed)
        if self.bind == 'y':
            self.publish_trajectory_setpoint(posef)
        else:
            self.offboard_setpoint_counter_=0
        msg = TrajectorySetpoint()
        msg.x, msg.y, msg.z = posec[0], posec[1], posec[2]
        msg.yaw = posec[3]
        msg.timestamp = self.time()
        self.setpoint_current = msg
        msg.x, msg.y, msg.z = posef[0], posef[1], posef[2]
        msg.yaw = posef[3]
        msg.timestamp = self.time()
        self.setpoint_future = msg
        if self.setpoint_reached():
            if self.offboard_setpoint_counter_ <= self.tof:
                self.offboard_setpoint_counter_+=1
    def tpol(self, x1, y1, z1, x2, y2, z2, x3, y3, z3, d):
        if abs(x3 - x2) <= d and abs(y3 - y2) <= d and abs(z3 - z2) <= d:
        # Use point B if the projected point is close enough
            return (x2,y2,z2)
        # Calculate the direction vector AB
        dx_ab = x2 - x1
        dy_ab = y2 - y1
        dz_ab = z2 - z1
        
        # Calculate the dot product of AB and AC
        dot_product = dx_ab * (x3 - x1) + dy_ab * (y3 - y1) + dz_ab * (z3 - z1)
        
        # Calculate the square of the length of AB
        length_ab_squared = dx_ab ** 2 + dy_ab ** 2 + dz_ab ** 2
        
        # Calculate the scaling factor for projection
        t = dot_product / length_ab_squared
        
        # Calculate the projected point P(x4, y4, z4)
        x4 = x1 + t * dx_ab
        y4 = y1 + t * dy_ab
        z4 = z1 + t * dz_ab
        
        # Calculate the displacement vector along AB
        dx_disp = x2 - x4
        dy_disp = y2 - y4
        dz_disp = z2 - z4
        
        # Normalize the displacement vector
        length_disp = (dx_disp ** 2 + dy_disp ** 2 + dz_disp ** 2) ** 0.5
        dx_disp /= length_disp
        dy_disp /= length_disp
        dz_disp /= length_disp
        
        # Translate the projected point P forward by d units along AB
        x4 += d * dx_disp
        y4 += d * dy_disp
        z4 += d * dz_disp
        
        return (x4, y4, z4, self.sp[3])

    
    #..................................................................................................#
    #Callback Functions
    def position_callback(self, datum:PoseStamped):
        self.pose = datum    
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
    #Current time (vehicle)
    def time(self):
        self.last_update_time_rel = int(time.time()*1000000) - self.last_update_time_abs + self.last_update_time_rel
        self.last_update_time_abs = int(time.time()*1000000)
        return self.last_update_time_rel
    # Setpoint_reached
    def setpoint_reached(self):
        return ((self.pose.pose.position.z>(self.setpoint_current.z-self.speed) and self.pose.pose.position.z<(self.setpoint_current.z+self.speed)) and
                ((self.pose.pose.position.x>(self.setpoint_current.x-self.speed) and self.pose.pose.position.x<(self.setpoint_current.x+self.speed)) and
                (self.pose.pose.position.y>(self.setpoint_current.y-self.speed) and self.pose.pose.position.y<(self.setpoint_current.y+self.speed))))
    
    # Publish a Setpoint and Set Current Setpoint
    def publish_trajectory_setpoint(self, posef:tuple):
        msg = TrajectorySetpoint()
        msg.x, msg.y, msg.z = posef[0], posef[1], posef[2]
        msg.yaw = posef[3]
        msg.timestamp = self.time()
        self.trajectory_setpoint_publisher_.publish(msg)
        self.get_logger().info(str(self.offboard_setpoint_counter_))

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()