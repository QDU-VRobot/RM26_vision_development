import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ros2_libxr'
    share_dir = get_package_share_directory(package_name)

    serial_driver_node = Node(
        package='ros2_libxr',
        executable='ros2_libxr_node',
        name='rm_serial_driver',
        output='both',
        emulate_tty=True,
    )

    return LaunchDescription([
        serial_driver_node
    ])