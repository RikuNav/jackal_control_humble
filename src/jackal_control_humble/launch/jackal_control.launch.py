
import os

from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'jackal_control_humble'

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'twist_mux.yaml']
    )

    filepath_config_interactive_markers = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'teleop_interactive_markers.yaml']
    )

    filepath_config_jackal = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'jackal.yaml']
    )

    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', ('teleop_ps4.yaml')]
    )

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[filepath_config_interactive_markers],
        output='screen',
    )

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy]
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[filepath_config_twist_mux]
    )

    node_twist_mux_2_cmd_drive = Node(
        package='jackal_control_humble',
        executable='twist_mux_2_cmd_drive',
        output='screen',
        name='twist_mux_2_cmd_drive',
        parameters=[jackal_config]
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
    )

    ld = LaunchDescription()
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_mux_2_cmd_drive)
    ld.add_action(micro_ros_agent)
    return ld
