"""
ROS2 Node for sending trajectory setpoints for square trajectory (simply run the node when placing drone at starting point, sp)
(Must be paired with offboard control ROS2 node running)
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
        #sp = [0.43, -2.03, -0.25, -1.5]
        sp = (0.00, -0.00, -0.25, 1.5)
        side = 1.5
        self.setpoint_targets = [sp,
                                 (sp[0]+0.00, sp[1]+0.00, sp[2]-0.20, sp[3]+0.00), 
                                 (sp[0]+side, sp[1]+0.00, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]+side, sp[1]+side, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]-side, sp[1]+side, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]-side, sp[1]-side, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]+side, sp[1]-side, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]+side, sp[1]+0.00, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]+0.00, sp[1]+0.00, sp[2]-0.20, sp[3]+0.00),
                                 (sp[0]+0.00, sp[1]+0.00, sp[2]-0.00, sp[3]+0.00)]   
    
    def timer_callback(self):
        pose1 = self.setpoint_targets[self.offboard_setpoint_counter_]
        pose2 = self.setpoint_targets[self.offboard_setpoint_counter_+1]
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
            if self.setpoint_reached_2>=200:
                self.offboard_setpoint_counter_+=1
                self.setpoint_reached_2=0
    
    #..................................................................................................#
    #Callback Functions
    def position_callback(self, datum:PoseStamped):
        self.pose = datum    
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
    def hov(self,x1, y1, x2, y2):
        #horizontal or vertical
        if x2==x1:
            return True
        elif y2== y1:
            return False
    def gd(self,a,b):
        #get direction
        if a-b<0:
            return -1
        else:
            return 1
    def tpol(self,x1,y1,x2,y2,x3,y3,d):
        #translated point on line from x2,y2 to x1,y1
        if self.hov(x1,y1,x2,y2):
            if abs(y1-y3)<=0.3:
                return [x1,y1]
            return [x1,y3+d*self.gd(y1,y3)]
        else:
            if abs(x1-x3)<=0.3:
                return [x1,y1]
            return [x3+d*self.gd(x1,x3),y1]
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
    
    # Publish a Setpoint and Set Current Setpoint
    def publish_trajectory_setpoint(self, pose1:tuple, pose2:tuple):
        msg = TrajectorySetpoint()
        posef = (self.tpol(pose2[0], pose2[1], pose1[0], pose1[1], self.pose.pose.position.x, self.pose.pose.position.y,0.3))
        msg.x, msg.y, msg.z = posef[0], posef[1], pose2[2]
        msg.yaw = pose2[3]
        msg.timestamp = self.time()
        self.trajectory_setpoint_publisher_.publish(msg)
        self.get_logger().info("NEW1:" + str(msg.z) +" " +str(self.setpoint_reached_2)+" "+
                               str(self.pose.pose.position.x)+" "+str(self.pose.pose.position.y)+" "+str(self.pose.pose.position.z)+" "+
                               str(self.offboard_setpoint_counter_))


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