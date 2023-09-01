"""
ROS2 Node for Real-Time X vs Y vs Z Position Graph based on Vicon
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from vicon_receiver.msg import Position
from px4_msgs.msg import VehicleOdometry, Timesync
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
import time

class PositionGraph(Node):
    def __init__(self):
        super().__init__('position_graph')
        # self.subscription = self.create_subscription(
        #     VehicleOdometry,
        #     '/fmu/vehicle_odometry/out',
        #     self.position_callback,
        #     10  # QoS profile depth
        # )
        self.posedummy_pub = self.create_subscription(
            Position, "/vicon/Holybro_Drone/Holybro_Drone", self.position_callback, 10)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.positions = {'x': [], 'y': [], 'z': []}
        self.fig = plt.figure()
        self.ax = plt.axes(projection='3d')  # Create a 3D subplot
        self.fig.canvas.mpl_connect('close_event', self.save_plot)

    def position_callback(self, msg):
        # self.positions['x'].append(msg.x)
        # self.positions['y'].append(msg.y)
        # self.positions['z'].append(-msg.z)
        self.positions['x'].append(msg.x_trans/1000)
        self.positions['y'].append(-msg.y_trans/1000)
        self.positions['z'].append(msg.z_trans/1000)
        self.update_graph()

    def update_graph(self):
        self.ax.clear()
        self.ax.plot(self.positions['x'], self.positions['y'], self.positions['z'], color='blue')
        # Set labels and title
        plt.title('VICON Positions X vs VICON Positions Y vs VICON Positions Z')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Position Trajectory')
        plt.draw()
        plt.pause(0.001)
    def save_plot(self, event):
        self.ax.figure.savefig('Vicon_position_trajectory.png')
        print("Plot saved as 'Vicon_position_trajectory.png'")

def main(args=None):
    rclpy.init(args=args)
    position_graph = PositionGraph()
    rclpy.spin(position_graph)
    position_graph.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()