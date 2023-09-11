"""
ROS2 Node for Receiving Vicon NED Reference Frame Position and Orientation on Onboard and sending it to Drone's Controller
(Needs to be paired with MicroRTPS Bridge and ROS2 node pose_pub)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleVisualOdometry, Timesync
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
            VehicleVisualOdometry, "/fmu/vehicle_visual_odometry/in", 10)
        self.timesync_callback = self.create_subscription(
            Timesync, "/fmu/timesync/in",self.timesync_callback,qos_profile)
        self.pose_callback = self.create_subscription(
            PoseStamped, "/Wifi/Channel_One", self.pose_callback, qos_profile)
        self.timer_ = self.create_timer(0.01, self.publish_pose)
        self.xdatum2 = VehicleVisualOdometry()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
    def publish_pose(self):
        msg = self.xdatum2
        self.last_update_time_rel = int(time.time()*1000000) - self.last_update_time_abs + self.last_update_time_rel
        self.last_update_time_abs = int(time.time()*1000000)
        msg.timestamp = self.last_update_time_rel
        msg.timestamp_sample = self.last_update_time_rel
        self.get_logger().info("Datum Published: "+str(msg.timestamp)+"\n" + str(msg.x)+" "+str(msg.y)+" "+str(msg.z)+str(msg.q[3]))
        self.pos_pub.publish(self.xdatum2)
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
        self.xdatum2.reset_counter = 5
    def pose_callback(self, datum:PoseStamped):
        msg = VehicleVisualOdometry()
        msg.local_frame = 0
        msg.x = datum.pose.position.x
        msg.y = datum.pose.position.y
        msg.z = datum.pose.position.z
        msg.q = [datum.pose.orientation.w,datum.pose.orientation.x,datum.pose.orientation.y,datum.pose.orientation.z]
        msg.velocity_frame = 1
        msg.vx = float("NaN")
        msg.vy = float("NaN")
        msg.vz = float("NaN") 
        msg.rollspeed = float("NaN")
        msg.pitchspeed = float("NaN")
        msg.yawspeed = float("NaN")
        msg.pose_covariance = [float("NaN")]*21
        # # Set the position variances to a low value (e.g., 0.001)
        # msg.pose_covariance[0] = 0.001  # x variance
        # msg.pose_covariance[6] = 0.001  # y variance
        # msg.pose_covariance[11] = 0.001  # z variance

        # # Set the orientation variances to a higher value (e.g., 0.1)
        # msg.pose_covariance[15] = 1.0  # roll variance
        # msg.pose_covariance[18] = 1.0  # pitch variance
        # msg.pose_covariance[20] = 1.0  # yaw variance
        msg.velocity_covariance = [float("NaN")]*21
        self.get_logger().info("Datum: "+str(msg.timestamp)+"\n" + str(msg.x)+" "+str(msg.y)+" "+str(msg.z))
        self.xdatum2 = msg
        
def main(args = None):
    rclpy.init(args=args)
    node = publishernode()
    rclpy.spin(node)
    rclpy.shutdown()
