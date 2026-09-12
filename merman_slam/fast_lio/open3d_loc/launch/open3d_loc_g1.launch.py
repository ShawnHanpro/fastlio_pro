from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    ])

    # base_link -> motion_link
    static_tf_base_link_to_motion_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_motion_link',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'motion_link'
        ]
    )

    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                # 'path_map': map_file,
                'pcd_queue_maxsize': 10,

                'voxelsize_coarse': 0.01,
                'voxelsize_fine': 0.2,

                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,

                'loc_frequence': 2.5,

                'save_scan': False,
                'hidden_removal': False,

                'maxpoints_source': 80000,
                'maxpoints_target': 400000,

                'filter_odom2map': False,

                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,

                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,

                'use_sim_time':
                    LaunchConfiguration('use_sim_time')
            }
        ]
    )

    delayed_global_localization_node = TimerAction(
        period=2.0,
        actions=[global_localization_node]
    )

    return LaunchDescription([
        use_sim_time_arg,

        static_tf_base_link_to_motion_link,

        delayed_global_localization_node,
    ])