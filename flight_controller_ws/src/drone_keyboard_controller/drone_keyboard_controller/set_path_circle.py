"""
ROS2 Node for sending trajectory setpoints for circular trajectory (simply run the node when placing drone at starting point, sp)
(See config file to adjust speed and dimensions of trajectory)
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
import time
import math
from math import sin, cos, tan, pi
from std_msgs.msg import String
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
        
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/Wifi/Channel_Two", 10)
        self.control_mode_callback = self.create_subscription(String,
                                                          "/Wifi/Channel_Five", self.control_mode_callback,10)
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
        self.r = self.config['circle']['radius']
        self.h = self.config['circle']['hheight']
        self.ns = 5*self.config['circle']['num_loops']
        self.xc = sp[0]
        self.yc = sp[1]
        self.setpoint_targets = [sp]
        self.setpoint_targets.append((sp[0], sp[1], sp[2]+self.h, sp[3]+0.00))
        for t in range(self.ns+1):
            xt = (self.r)*cos(2*pi*(t/5))
            yt = (self.r)*sin(2*pi*(t/5))
            zt = self.h
            self.setpoint_targets.append((sp[0]+xt, sp[1]+yt, sp[2]+zt, sp[3]+0.00))
        self.setpoint_targets.append((sp[0], sp[1], sp[2]+self.h, sp[3]+0.00))
        self.setpoint_targets.append(sp)
        self.bind = 'z' 
    def control_mode_callback(self, datum:String):
        self.bind = datum.data
    def timer_callback(self):
        pose1 = self.setpoint_targets[self.offboard_setpoint_counter_]
        pose2 = self.setpoint_targets[self.offboard_setpoint_counter_+1]
        if self.bind == 'e':
            self.publish_trajectory_setpoint(pose1,pose2)
        else:
            self.offboard_setpoint_counter_=0
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
            if self.setpoint_reached_2>=10:
                if self.offboard_setpoint_counter_<len(self.setpoint_targets)-2:
                    self.offboard_setpoint_counter_+=1
                self.setpoint_reached_2=0
    
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
        return ((self.pose.pose.position.z>(self.setpoint_current.z-0.1) and self.pose.pose.position.z<(self.setpoint_current.z+0.1)) and
                ((self.pose.pose.position.x>(self.setpoint_current.x-0.1) and self.pose.pose.position.x<(self.setpoint_current.x+0.1)) and
                (self.pose.pose.position.y>(self.setpoint_current.y-0.1) and self.pose.pose.position.y<(self.setpoint_current.y+0.1))))
    def gd(self,a,b):
        #get direction
        if a-b<0:
            return -1
        else:
            return 1
    def gd2(self,xc,yc,x1,y1,x2,y2):
        
        # Calculate the displacement vector from the center to the given point
        dx = x1 - xc
        dy = y1 - yc
        
        # Calculate the angle between the center-to-point vector and the x-axis
        atp1 = math.atan2(dy, dx)
        if atp1<0:
            atp1 = 2*pi+atp1
        # Calculate the displacement vector from the center to the given point
        dx = x2 - xc
        dy = y2 - yc
        
        # Calculate the angle between the center-to-point vector and the x-axis
        atp2 = math.atan2(dy, dx)
        if atp2<0:
            atp2 = 2*pi+atp2
        if atp1<=(4*pi/self.ns+0.2) and atp2>(2*pi-4*pi/(self.ns)-0.2):
            return 1
        elif atp1>=atp2:
            return 1
        else:
            return -1
    def closest_point_on_circle(self, x1, y1, r, x2, y2):
        # Calculate vector from circle center to second point
        dx = x2 - x1
        dy = y2 - y1
        
        # Calculate the distance from the circle center to the second point
        distance = math.sqrt(dx**2 + dy**2)
        
        # Normalize the vector to get a unit vector
        if distance != 0:
            normalized_dx = dx / distance
            normalized_dy = dy / distance
        else:
            # Handle the case when the two points coincide (distance is zero)
            normalized_dx = 0
            normalized_dy = 0
        
        # Scale the unit vector by the radius
        scaled_dx = r * normalized_dx
        scaled_dy = r * normalized_dy
        
        # Add the scaled vector to the circle center to get the closest point on the circle
        closest_x = x1 + scaled_dx
        closest_y = y1 + scaled_dy
        
        return closest_x, closest_y
    def point_along_circle(self, center_x, center_y, radius, point_x, point_y, target_x, target_y, distance):
        # Convert angle from degrees to radians
        angle_radians = distance/radius
        
        # Calculate the displacement vector from the center to the given point
        dx = point_x - center_x
        dy = point_y - center_y
        
        # Calculate the angle between the center-to-point vector and the x-axis
        angle_to_point = math.atan2(dy, dx)
        
        # Calculate the new angle by adding the desired angle to the angle to the given point
        new_angle = angle_to_point + self.gd2(center_x,center_y,target_x, target_y, point_x, point_y)*angle_radians
        
        # Calculate the new point's coordinates on the circle
        new_x = center_x + radius * math.cos(new_angle)
        new_y = center_y + radius * math.sin(new_angle)
        return new_x, new_y
    def tpoc(self, xc, yc, r, x1, y1, x2, y2, d):
        x3,y3 = self.closest_point_on_circle(xc, yc, r, x2, y2)
        x4,y4 = self.point_along_circle(xc, yc, r, x3, y3, x1, y1, d)
        return [x4,y4]
    def tpol(self,x1,y1,x2,y2,x3,y3,d):
        #translated point on line from x2,y2 to x1,y1
        dx = x3 - x1
        dy = y3 - y1
        distance = math.sqrt(dx**2 + dy**2)
        if distance<=d:
            return [x1,y1]
        elif self.setpoint_reached():
            return [x1,y1]
        elif self.offboard_setpoint_counter_== 0:
            return [x1,y1]
        elif self.offboard_setpoint_counter_ == 1:
            return [x3+d*self.gd(x1,x3),y1]
        elif self.offboard_setpoint_counter_ == self.ns+2:
            return [x3+d*self.gd(x1,x3),y1]
        elif self.offboard_setpoint_counter_>self.ns+2:
            return [x1,y1]
        else:
            return self.tpoc(self.xc,self.yc,self.r,x1,y1,x3,y3,d)
    # Publish a Setpoint and Set Current Setpoint
    def publish_trajectory_setpoint(self, pose1:tuple, pose2:tuple):
        msg = TrajectorySetpoint()
        posef = (self.tpol(pose2[0], pose2[1], pose1[0], pose1[1], self.pose.pose.position.x, self.pose.pose.position.y,0.3))
        msg.x, msg.y, msg.z = posef[0], posef[1], pose2[2]
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