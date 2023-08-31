"""
Python implementation of Offboard Control
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
from math import sin, cos, tan, pi

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
                                                          "/Wifi/Channel_One", self.position_callback,10)

        self.offboard_setpoint_counter_ = 0
        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.pose = PoseStamped()
        self.setpoint_current = TrajectorySetpoint()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)   
    
    def timer_callback(self):
        if self.setpoint_reached():
            self.offboard_setpoint_counter_+=1
        t = self.offboard_setpoint_counter_
        if t<=20:
            self.publish_trajectory_setpoint((2*cos(2*pi*(t/20)),2*sin(2*pi*(t/20)),-1.0*sin((pi/8)*t)-2.0,0.0))
        else:
            self.publish_trajectory_setpoint((0.0,0.0,0.0,0.0))
    
    #..................................................................................................#
    #Callback Functions
    def position_callback(self, datum:PoseStamped):
        self.pose = datum    
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
    def setpoint_callback(self, datum2:TrajectorySetpoint):
        self.setpoint_current = datum2
    
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
    def publish_trajectory_setpoint(self, pose:tuple):
        msg = TrajectorySetpoint()
        msg.x, msg.y, msg.z = pose[0], pose[1], pose[2]
        msg.yaw = pose[3]
        msg.timestamp = self.time()
        self.setpoint_current = msg
        self.trajectory_setpoint_publisher_.publish(msg)
        self.get_logger().info(str(msg.z) +" " +str(self.setpoint_reached())+" "+
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