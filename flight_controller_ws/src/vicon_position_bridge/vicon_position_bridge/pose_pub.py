"""
ROS2 Node for Converting Vicon FLU Reference Frame Position and Orientation to NED Reference Frame and sending it to Onboard Computer
(Needs to be paired with MicroRTPS Bridge and ROS2 node pose_sub)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from vicon_receiver.msg import Position
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from pyquaternion import Quaternion
import numpy as np

class publishernode(Node):
    def __init__(self):
        super().__init__("pose_pub")
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.get_logger().info("Hello_World1")
        self.counter = 0
        self.pos_pub = self.create_publisher(
            PoseStamped, "/Wifi/Channel_One", qos_profile)
        self.posedummy_pub = self.create_subscription(
            Position, "/vicon/Holybro_Drone/Holybro_Drone", self.publish_datum, 10)
    
    def publish_datum(self, datum:Position):
        pose_stamped_msg = PoseStamped()

        # Populate the pose information
        local_rot = Quaternion([datum.w, datum.x_rot, datum.y_rot, datum.z_rot])
        
        enu2ned = Quaternion(0, np.sqrt(2)/2, np.sqrt(2)/2, 0) # w, x, y, z,   
        quat = Quaternion(0, 1, 0, 0)
        flu2ned = Quaternion(0,1,0,0)
        quat2 = Quaternion(0,-1,0,0)
        ned_rot = flu2ned * local_rot * quat2

        
        self.get_logger().info(f"before rotation: {(np.array(local_rot.yaw_pitch_roll))}")
        self.get_logger().info(f"after new rotation: {(np.array(ned_rot.yaw_pitch_roll))}")
        pose_stamped_msg.pose.position.x = datum.x_trans/1000
        pose_stamped_msg.pose.position.y = -datum.y_trans/1000 
        pose_stamped_msg.pose.position.z = -datum.z_trans/1000
        pose_stamped_msg.pose.orientation.x = ned_rot.x
        pose_stamped_msg.pose.orientation.y = ned_rot.y
        pose_stamped_msg.pose.orientation.z = ned_rot.z
        pose_stamped_msg.pose.orientation.w = ned_rot.w


        # Populate the header information
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'map'
        t = pose_stamped_msg.header.stamp
        x = pose_stamped_msg.pose.position.x
        y = pose_stamped_msg.pose.position.y
        z = pose_stamped_msg.pose.position.z
        self.get_logger().info(f'Publishing PoseStamped message: t={t}, x={x}, y={y}, z={z}')
        self.pos_pub.publish(pose_stamped_msg)
        
def main(args = None):
    rclpy.init(args = args)
    node = publishernode()
    rclpy.spin(node)
    rclpy.shutdown()