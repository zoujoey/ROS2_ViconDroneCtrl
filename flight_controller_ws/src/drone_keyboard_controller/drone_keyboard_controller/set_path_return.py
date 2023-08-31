"""
Python implementation of Offboard Control
"""


import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleVisualOdometry, Timesync, VehicleOdometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from math import sin, cos, tan, pi
import time
import math
import yaml
import os


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
        timer_period = 0.01  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.setpoint_reached_2 = 0
        self.pose = PoseStamped()
        self.setpoint_previous = TrajectorySetpoint()
        self.setpoint_current = TrajectorySetpoint()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
        config_file_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config0.yaml')
        with open(config_file_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        self.config = config
        sp = (self.config['start']['spx'],self.config['start']['spy'],self.config['start']['spz'],self.config['start']['spyaw'])
        self.setpoint_targets = [sp,
                                 (sp[0]+0.00, sp[1]+0.00, sp[2]+self.config['command']['rh'], sp[3]+0.00)]
        self.speed = self.config['command']['ms']
        self.bind = 'z'   
    def control_mode_callback(self, datum:String):
        self.bind = datum.data
    def timer_callback(self):
        pose1 = self.setpoint_targets[self.offboard_setpoint_counter_]
        pose2 = self.setpoint_targets[self.offboard_setpoint_counter_+1]
        if self.bind == 'r':
            self.publish_trajectory_setpoint(pose1,pose2)
        msg = TrajectorySetpoint()
        msg.x, msg.y, msg.z = pose1[0], pose1[1], pose1[2]
        msg.yaw = pose1[3]
        msg.timestamp = self.time()
        self.setpoint_previous = msg
        msg.x, msg.y, msg.z = pose2[0], pose2[1], pose2[2]
        msg.yaw = pose2[3]
        msg.timestamp = self.time()
        self.setpoint_current = msg
        if self.setpoint_reached():
            self.setpoint_reached_2+=1
            if self.setpoint_reached_2>=50:
                if self.offboard_setpoint_counter_<(len(self.setpoint_targets)-2):
                    self.offboard_setpoint_counter_+=1
                self.setpoint_reached_2=0
    
    #..................................................................................................#
    #Callback Functions
    def position_callback(self, datum:PoseStamped):
        self.pose = datum    
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
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
        
        return (x4, y4, z4)
    def time(self):
        self.last_update_time_rel = int(time.time()*1000000) - self.last_update_time_abs + self.last_update_time_rel
        self.last_update_time_abs = int(time.time()*1000000)
        return self.last_update_time_rel

    # Setpoint_reached
    def setpoint_reached(self):
        return ((self.pose.pose.position.z>(self.setpoint_current.z-0.1) and self.pose.pose.position.z<(self.setpoint_current.z+0.1)) and
                ((self.pose.pose.position.x>(self.setpoint_current.x-0.1) and self.pose.pose.position.x<(self.setpoint_current.x+0.1)) and
                (self.pose.pose.position.y>(self.setpoint_current.y-0.1) and self.pose.pose.position.y<(self.setpoint_current.y+0.1))))
    
    # Publish a Setpoint and Set Current Setpoint
    def publish_trajectory_setpoint(self, pose1:tuple, pose2:tuple):
        msg = TrajectorySetpoint()
        posef = (self.tpol(pose1[0], pose1[1], pose1[2], pose2[0], pose2[1], pose2[2], 
                           self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, 
                           self.speed))
        msg.x, msg.y, msg.z = posef[0], posef[1], posef[2]
        msg.yaw = pose2[3]
        msg.timestamp = self.time()
        self.trajectory_setpoint_publisher_.publish(msg)


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