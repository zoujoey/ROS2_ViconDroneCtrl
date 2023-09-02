from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    
    hostname = '10.42.0.130'
    buffer_size = 1024
    topic_namespace = 'vicon'

    vicon_wrapper_node = Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )
    ld.add_action(vicon_wrapper_node)

    pose_pub = Node(
        package = "vicon_position_bridge",
        executable = "pose_pub"
    )

    ld.add_action(pose_pub)
    
    # fake_vicon_position_node = Node(
    #     package="vicon_position_bridge",
    #     executable = "fake_pose_pub",
    # )
    # ld.add_action(fake_vicon_position_node)

    # pose_data_save = Node(
    #     package = "vicon_position_bridge",
    #     executable = "pose_data_save"
    # )
    # ld.add_action(pose_data_save)

    # pose_grapher_xyz = Node(
    #     package = "vicon_position_bridge",
    #     executable = "pose_grapher_xyz"
    # )
    # ld.add_action(pose_grapher_xyz)

    # pose_grapher_xy = Node(
    #     package = "vicon_position_bridge",
    #     executable = "pose_grapher_xy"
    # )
    # ld.add_action(pose_grapher_xy)

    # pose_grapher_yz = Node(
    #     package = "vicon_position_bridge",
    #     executable = "pose_grapher_yz"
    # )
    # ld.add_action(pose_grapher_yz)

    # pose_grapher_xz = Node(
    #     package = "vicon_position_bridge",
    #     executable = "pose_grapher_xz"
    # )
    # ld.add_action(pose_grapher_xz)

    return ld