"""
ROS2 Node for processing keyboard inputs from keyboard_controller, communicating to trajectories on stand-by, and manual control of the drone
"""


import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import Timesync
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
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
        
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/Wifi/Channel_Two", 10)
        self.control_mode_publisher = self.create_publisher(String,
                                                                    "/Wifi/Channel_Five", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/vehicle_command/in", 10)

        self.setpoint_callback = self.create_subscription(TrajectorySetpoint,
                                                          "/Wifi/Channel_Two", self.setpoint_callback, 10)
        self.position_callback = self.create_subscription(PoseStamped,
                                                          "/Wifi/Channel_One", self.position_callback,qos_profile)
        self.command_callback = self.create_subscription(String,
                                                          "/Wifi/Channel_Four", self.command_callback,10)
        self.moveBindings = {
            'w': (0, 1, 0, 0),
            'a': (1, 0, 0, 0),
            's': (0, -1, 0, 0),
            'd': (-1, 0, 0, 0),
            'i': (0, 0, -1, 0),
            'j': (0, 0, 0, 1),
            'k': (0, 0, 1, 0),
            'l': (0, 0, 0, -1),
            'c': (0, 0, 0, 0)
        }

        self.setBindings = {
            'q': 'rect',
            'e': 'circ',
            'h': 'heli',
            'r': 'return',
            'x': 'fail-safe land',
            'f': 'float',
            't': 'linear_setpoint',
            'y': 'continuous_setpoint'
        }
        self.control = True
        timer_period = 0.01  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        timer_period = 0.01  # 100 milliseconds
        self.timer_2 = self.create_timer(timer_period, self.timer_callback)
        self.pose = PoseStamped()
        self.setpoint_current = TrajectorySetpoint()
        self.last_update_time_rel = 0
        self.last_update_time_abs = int(time.time()*1000000)
        config_file_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config0.yaml')
        with open(config_file_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        self.config = config
        self.sp = (self.config['start']['spx'],self.config['start']['spy'],self.config['start']['spz'],self.config['start']['spyaw'])
        self.setpoint_target = self.sp
        self.bind = 'z'
        self.out_of_control = 0
        self.mx = self.config['command']['mx']
        self.my = self.config['command']['my']
        self.mz = self.config['command']['mz']
        self.ms = self.config['command']['ms']
        self.cr = self.config['command']['cr']
        self.ooct = self.config['command']['ooct']
    
    
    #..................................................................................................#
    #Callback Functions
    def command_callback(self, datum:String):
        pose = self.setpoint_target
        keyz = datum.data
        if keyz in self.moveBindings:
            self.bind = keyz
            self.control = True
            direction = self.moveBindings[keyz]
            change = self.cr
            posef = tuple(pose[i]+change*direction[i] for i in range(len(pose)))
            if self.setpoint_near(posef,self.ms) and self.setpoint_boundary(posef):
                self.setpoint_target = posef
            else:
                self.out_of_control+=1
                if self.out_of_control==self.ooct:
                    self.setpoint_target = (self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z-0.1,self.sp[3])
                    self.out_of_control = 0
        elif (keyz in self.setBindings) and (self.setpoint_near(self.sp,0.1)):
            self.bind = keyz
            self.control = False
        elif keyz == 'r':
            self.bind = keyz
            self.control = False
        if keyz == 'x' and (self.pose.pose.position.z>self.sp[2]+self.config['command']['kh']):
            self.bind = keyz
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
            self.get_logger().info("Kill command send")
            
    def timer_callback(self):
        msg = String()
        msg.data = self.bind
        self.control_mode_publisher.publish(msg)
        if self.control:
            pose = self.setpoint_target
            msg = TrajectorySetpoint()
            msg.x, msg.y, msg.z = pose[0], pose[1], pose[2]
            msg.yaw = pose[3]
            msg.timestamp = self.time()
            self.trajectory_setpoint_publisher_.publish(msg)
        else:
            self.setpoint_target = (self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z, self.sp[3])
        
        self.get_logger().info("\n"+ "KEY: " + self.bind + "\n"
                                "REACHED/MODE:" +str(self.setpoint_reached())+" "+str(self.control)+"\n"+
                                "POSE:" +str(self.pose.pose.position.x)+" "+str(self.pose.pose.position.y)+" "+str(self.pose.pose.position.z)+"\n"+
                                "SET:" +str(self.setpoint_current.x)+" "+str(self.setpoint_current.y)+" "+str(self.setpoint_current.z))

    def setpoint_callback(self, datum2:TrajectorySetpoint):
        self.setpoint_current = datum2

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
    def setpoint_boundary(self, pose):
        return (((pose[0]<(self.sp[0]+self.mx) and pose[0]>(self.sp[0]-self.mx)) and 
                 (pose[1]<(self.sp[1]+self.my) and pose[1]>(self.sp[1]-self.my))) and 
                 (pose[2]<self.sp[2] and pose[2]>-self.mz))
    # Setpoint_near
    def setpoint_near(self, pose,l):
        return ((self.pose.pose.position.z>(pose[2]-l) and self.pose.pose.position.z<(pose[2]+l)) and
                ((self.pose.pose.position.x>(pose[0]-l) and self.pose.pose.position.x<(pose[0]+l)) and
                (self.pose.pose.position.y>(pose[1]-l) and self.pose.pose.position.y<(pose[1]+l))))
    
    # Setpoint_reached
    def setpoint_reached(self):
        l = 0.1
        return ((self.pose.pose.position.z>(self.setpoint_current.z-l) and self.pose.pose.position.z<(self.setpoint_current.z+l)) and
                ((self.pose.pose.position.x>(self.setpoint_current.x-l) and self.pose.pose.position.x<(self.setpoint_current.x+l)) and
                (self.pose.pose.position.y>(self.setpoint_current.y-l) and self.pose.pose.position.y<(self.setpoint_current.y+l))))
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
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()