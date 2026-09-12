from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zlac8015d_four_wheel_driver_cpp',
            executable='four_wheel_driver_node',
            name='four_wheel_driver',
            namespace='wheel_control_can',
            output='screen',
            parameters=[{
                'can_interface': 'can0',
                'right_controller_node_id': 1,
                'left_controller_node_id': 2,
                'command_timeout_sec': 0.3,
                'control_hz': 100.0,
                'state_hz': 10.0,
                'accel_ms': 100,
                'decel_ms': 100,
                'sync_control': True,
                'use_pdo_commands': True,
                'configure_pdo_mapping': True,
                'sdo_timeout_ms': 200,
                'state_sdo_timeout_ms': 50,
                'right_wheel_sign': -1.0,
                'left_wheel_sign': 1.0,
                'emergency_stop_shutdown_enabled': True,
                'state_read_failure_shutdown_consecutive_cycles': 5,
            }],
        )
    ])
