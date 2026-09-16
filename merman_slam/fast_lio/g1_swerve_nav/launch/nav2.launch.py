from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def affinity_prefix(cpu_list):
    return ['taskset --cpu-list ', cpu_list]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    collision_params_file = LaunchConfiguration('collision_params_file')
    chassis_cpu = EnvironmentVariable('CPU_CHASSIS', default_value='5')
    controller_cpu = EnvironmentVariable('CPU_CONTROLLER', default_value='6')
    planning_cpu = EnvironmentVariable('CPU_PLANNING', default_value='7')

    pkg_share = FindPackageShare('g1_swerve_nav')
    nav_to_pose_bt = PathJoinSubstitution([
        pkg_share, 'behavior_trees', 'navigate_to_pose.xml'
    ])
    nav_through_poses_bt = PathJoinSubstitution([
        pkg_share, 'behavior_trees', 'navigate_through_poses.xml'
    ])

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    control_nodes = [
        'velocity_smoother',
        'collision_monitor',
    ]

    common = [nav2_params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2.yaml'])),
        DeclareLaunchArgument(
            'collision_params_file',
            default_value=PathJoinSubstitution([
                pkg_share, 'config', 'collision_monitor.yaml'
            ])),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            prefix=affinity_prefix(controller_cpu),
            parameters=common,
            remappings=[('cmd_vel', '/cmd_vel_nav')]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            prefix=affinity_prefix(planning_cpu),
            parameters=common),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            prefix=affinity_prefix(planning_cpu),
            parameters=common,
            remappings=[('cmd_vel', '/cmd_vel_behavior')]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            prefix=affinity_prefix(planning_cpu),
            parameters=[
                nav2_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                    'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                }
            ]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            prefix=affinity_prefix(planning_cpu),
            parameters=common),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            prefix=affinity_prefix(chassis_cpu),
            parameters=common,
            remappings=[
                ('cmd_vel', '/cmd_vel_selected'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ]),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            prefix=affinity_prefix(chassis_cpu),
            parameters=[collision_params_file, {'use_sim_time': use_sim_time}]),

        # The command-safety chain does not depend on /map. Bring it up with a
        # dedicated manager so a map-related navigation transition cannot keep
        # velocity smoothing and collision stopping inactive.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_control',
                    output='screen',
                    prefix=affinity_prefix(chassis_cpu),
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': control_nodes,
                    }]),
            ]),

        # Start the navigation manager only after its Nav2 processes have had
        # time to create their lifecycle services. On the real robot the whole
        # stack starts together with Fast-LIO / drivers. A navigation transition
        # may still wait for /map, but it cannot block the control manager above.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    prefix=affinity_prefix(planning_cpu),
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': navigation_nodes,
                    }]),
            ]),
    ])
