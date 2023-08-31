"""
ROS2 Node for saving the position data (x,y,z,t) from vicon/simulated system
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from px4_msgs.msg import VehicleOdometry
import csv

class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/vehicle_odometry/out',
            self.position_callback,
            10  # QoS profile depth
        )
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.csv_file = open('position_data.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['X', 'Y', 'Z', 'Timestamp'])  # Write CSV header

    def position_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        # x = msg.x_trans/1000)
        # y = msg.y_trans/1000)
        # z = msg.z_trans/1000)
        timestamp = self.get_clock().now().to_msg()  # Convert to seconds
        self.save_to_csv(x, y, z, timestamp)

    def save_to_csv(self, x, y, z, timestamp):
        # Save current data to CSV
        data_row = [x, y, z, timestamp]
        self.csv_writer.writerow(data_row)
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    csv_logger_node = CSVLoggerNode()
    try:
        rclpy.spin(csv_logger_node)
    except KeyboardInterrupt:
        pass

    csv_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()