import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('g1_nav2')

    nav2_params = os.path.join(
        package_share,
        'config',
        'nav2_g1.yaml'
    )

    controller_params = os.path.join(
        package_share,
        'config',
        'pseudo_diff_4wis.yaml'
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    # ----------------------------------------------------------
    # 4WIS pseudo differential controller
    #
    # teleop:
    #   /cmd_vel
    #
    # Nav2:
    #   /cmd_vel_nav
    #
    # output:
    #   /steer/position_cmd
    #   /wheel_control_can/wheel_rpm_cmd
    # ----------------------------------------------------------
    pseudo_diff_4wis = Node(
        package='g1_nav2',
        executable='pseudo_diff_4wis_controller',
        name='pseudo_diff_4wis_controller',
        output='screen',
        parameters=[controller_params],
    )

    # ----------------------------------------------------------
    # Nav2
    # ----------------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],

        # Nav2 自动导航命令单独走 /cmd_vel_nav。
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],

        # Spin / BackUp 等行为也走 /cmd_vel_nav。
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        pseudo_diff_4wis,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
