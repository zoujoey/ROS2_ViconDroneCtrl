"""
ROS2 Launch File for starting the keyboard_controller
"""
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    keyboard_control_node = Node(
        package = "drone_keyboard_controller",
        executable = "keyboard_controller"
    )
    command_control_node = Node(
        package = "drone_keyboard_controller",
        executable = "command_control"
    )
    offboard_control_node = Node(
        package = "drone_keyboard_controller",
        executable = "offboard_control"
    )
    circle_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_circle"
    )
    helix_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_helix"
    )
    hover_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_hover"
    )
    return_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_return"
    )
    square_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_square"
    )
    linear_setpoint_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_linear_setpoint"
    )
    continuous_setpoint_node = Node(
        package = "drone_keyboard_controller",
        executable = "set_path_continuous_setpoint"
    )


    ld.add_action(keyboard_control_node)
    ld.add_action(command_control_node)
    ld.add_action(offboard_control_node)
    ld.add_action(circle_node)
    ld.add_action(helix_node)
    ld.add_action(hover_node)
    ld.add_action(return_node)
    ld.add_action(square_node)
    ld.add_action(linear_setpoint_node)
    ld.add_action(continuous_setpoint_node)
    return ld