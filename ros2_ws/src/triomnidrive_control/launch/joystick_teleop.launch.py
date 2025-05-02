from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('triomnidrive_control')

    grpc_client_node = Node(
        package='triomnidrive_control',
        executable='grpc_client',
        name='grpc_client',
    )

    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[os.path.join(pkg_share, 'config', 'joystick_config.yaml')]
    )

    joystick_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joystick_teleop',
        parameters=[os.path.join(pkg_share, 'config', 'joystick_teleop.yaml')]
    )

    return LaunchDescription([
        grpc_client_node,
        joystick_node,
        joystick_teleop_node
    ])