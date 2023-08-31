"""
ROS2 Node for Simulated Position Lock when using Gazebo Simulations
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleVisualOdometry, Timesync, VehicleOdometry
import pyquaternion
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import time
import sys

class publishernode(Node):
    def __init__(self):
        super().__init__("pose_rec_two")
        self.get_logger().info("Hello_World1")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pos_pub = self.create_publisher(
            PoseStamped, "/Wifi/Channel_One", qos_profile)
        self.timesync_callback = self.create_subscription(
            Timesync, "/fmu/timesync/in",self.timesync_callback,qos_profile)
        self.pose_callback = self.create_subscription(
            VehicleOdometry, "/fmu/vehicle_odometry/out", self.pose_callback, qos_profile)
        self.timer_ = self.create_timer(0.01, self.publish_pose)
        self.xdatum2 = PoseStamped()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
    def publish_pose(self):
        msg = self.xdatum2
        self.get_logger().info("Datum Published: "+str(msg.header.stamp)+"\n" + str(msg.pose.position.x)+" "+str(msg.pose.position.y)+" "+str(msg.pose.position.z))
        self.pos_pub.publish(self.xdatum2)
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
    def pose_callback(self, msg:VehicleOdometry):
        datum = PoseStamped()
        datum.header.stamp = self.get_clock().now().to_msg()
        datum.header.frame_id = 'map'
        datum.pose.position.x = msg.x
        datum.pose.position.y = msg.y
        datum.pose.position.z = msg.z
        datum.pose.orientation.w,datum.pose.orientation.x,datum.pose.orientation.y,datum.pose.orientation.z = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3]) 
        self.xdatum2 = datum
        
def main(args = None):
    rclpy.init(args=args)
    node = publishernode()
    rclpy.spin(node)
    rclpy.shutdown()