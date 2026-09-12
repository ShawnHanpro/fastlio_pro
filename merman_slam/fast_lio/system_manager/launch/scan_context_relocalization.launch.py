from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('system_manager')
    params_file = os.path.join(
        pkg_share, 'config', 'scan_context_relocalization.yaml')

    return LaunchDescription([
        Node(
            package='system_manager',
            executable='scan_context_relocalization',
            name='scan_context_relocalization',
            output='screen',
            parameters=[params_file],
        ),
    ])
