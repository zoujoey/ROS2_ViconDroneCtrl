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

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10)
        
        self.timesync_callback = self.create_subscription(Timesync, 
                                                          "/fmu/timesync/in",self.timesync_callback,qos_profile)
        self.position_callback = self.create_subscription(PoseStamped,
                                                          "/Wifi/Channel_One", self.position_callback,10)
        self.setpoint_callback = self.create_subscription(TrajectorySetpoint,
                                                          "/Wifi/Channel_Two", self.setpoint_callback, 10)
        self.offboard_setpoint_counter_ = 0
        timer_period = 0.01  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)


        self.pose = PoseStamped()
        self.setpoint_current = TrajectorySetpoint()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
        

    #Current time (vehicle)
    def time(self):
        self.last_update_time_rel = int(time.time()*1000000) - self.last_update_time_abs + self.last_update_time_rel
        self.last_update_time_abs = int(time.time()*1000000)
        return self.last_update_time_rel
    
    # Arm/Disarm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    # Setpoint_reached
    def setpoint_reached(self):
        return ((self.pose.pose.position.z>(self.setpoint_current.z-0.05) and self.pose.pose.position.z<(self.setpoint_current.z+0.05)) and
                ((self.pose.pose.position.x>(self.setpoint_current.x-0.05) and self.pose.pose.position.x<(self.setpoint_current.x+0.05)) and
                (self.pose.pose.position.y>(self.setpoint_current.y-0.05) and self.pose.pose.position.y<(self.setpoint_current.y+0.05))))
    
    #Callback Functions
    def position_callback(self, datum:PoseStamped):
        self.pose = datum    
    def timesync_callback(self, datum1:Timesync):
        self.last_update_time_rel= datum1.timestamp
        self.last_update_time_abs = int(time.time()*1000000)
    def setpoint_callback(self, datum2:TrajectorySetpoint):
        self.setpoint_current = datum2

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 300):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        if (self.offboard_setpoint_counter_ < 301):
            self.offboard_setpoint_counter_ += 1


    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.time()
        self.offboard_control_mode_publisher_.publish(msg)

    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''

    def publish_trajectory_setpoint(self):
        msg = self.setpoint_current
        msg.timestamp = self.time()
        self.trajectory_setpoint_publisher_.publish(msg)
    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = self.time()
        self.vehicle_command_publisher_.publish(msg)

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