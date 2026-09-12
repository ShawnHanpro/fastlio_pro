from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path


OTHER_CPU_CORE = EnvironmentVariable('ROBOT_OTHER_CPU_CORE', default_value='7')


def affinity_prefix(cpu_core):
    return ['taskset -c ', cpu_core]


def generate_launch_description():
    package_share = Path(get_package_share_directory('ackermann_steering_controller'))
    config_file = package_share / 'config' / 'ackermann_kinematics.yaml'

    return LaunchDescription([
        Node(
            package='ackermann_steering_controller',
            executable='ackermann_steering_controller_node',
            name='ackermann_kinematics_node',
            output='screen',
            prefix=affinity_prefix(OTHER_CPU_CORE),
            parameters=[str(config_file)]
        )
    ])
