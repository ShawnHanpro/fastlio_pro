from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    pkg_share = FindPackageShare('g1_swerve_nav')
    nav_to_pose_bt = PathJoinSubstitution([
        pkg_share, 'behavior_trees', 'navigate_to_pose.xml'
    ])
    nav_through_poses_bt = PathJoinSubstitution([
        pkg_share, 'behavior_trees', 'navigate_through_poses.xml'
    ])

    managed_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    common = [nav2_params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2.yaml'])),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common,),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common,),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                    'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                }
            ]),

        # Start the lifecycle manager only after all Nav2 processes have had
        # time to create their lifecycle services. On the real robot the whole
        # stack starts together with Fast-LIO / drivers, and starting the
        # manager immediately can make the first controller_server change_state
        # response time out under startup load. If that happens, none of the
        # downstream lifecycle nodes (velocity_smoother / collision_monitor)
        # become active, which also prevents keyboard commands reaching the
        # swerve controller and leaves RViz without an active costmap footprint.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': True,
                        'node_names': managed_nodes,
                    }]),
            ]),
    ])
