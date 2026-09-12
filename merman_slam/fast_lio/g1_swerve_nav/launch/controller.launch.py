from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kinematics_params_file = LaunchConfiguration('kinematics_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('g1_swerve_nav'), 'config', 'kinematics.yaml'
            ]),
            description='Swerve controller + cmd_vel arbiter parameters'),

        # Node(
        #     package='g1_swerve_nav',
        #     executable='cmd_vel_arbiter',
        #     name='cmd_vel_arbiter',
        #     output='screen',
        #     parameters=[kinematics_params_file]),

        Node(
            package='g1_swerve_nav',
            executable='swerve_controller',
            name='swerve_controller',
            output='screen',
            parameters=[kinematics_params_file]),
    ])
