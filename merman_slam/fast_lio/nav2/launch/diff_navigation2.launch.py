import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    db_dir = '/home/niic/motion_control_chassis/src/nav2/map'
    os.makedirs(db_dir, exist_ok=True)

    nav2_pkg = get_package_share_directory('nav2')
    evbhuman_slam_pkg = get_package_share_directory('evbhuman_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # map / params / ekf
    map_dir = '/home/niic/motion_control_chassis/src/nav2/map/rtabmap_lidar.yaml'
    param_dir = os.path.join(nav2_pkg, 'param', 'evbhuman_diff_navigation2.yaml')
    ekf_config_file = os.path.join(evbhuman_slam_pkg, 'config', 'ekf_odom_diff.yaml')
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')

    urdf_file = '/home/niic/motion_control_chassis/src/evbhuman_description/urdf/EVBH_SEAG1_0_real.urdf'

    # OpenCV env（照你原来的保留）
    opencv_prefix = os.path.expanduser('~/third_party/opencv_aruco')
    opencv_dir = os.path.join(opencv_prefix, 'lib/cmake/opencv4')
    set_opencv_env = SetEnvironmentVariable('OpenCV_DIR', opencv_dir)
    set_cmake_prefix = SetEnvironmentVariable('CMAKE_PREFIX_PATH', f'{opencv_prefix}:{os.environ.get("CMAKE_PREFIX_PATH", "")}')
    set_ld_library = SetEnvironmentVariable('LD_LIBRARY_PATH', f'{os.path.join(opencv_prefix, "lib")}:{os.environ.get("LD_LIBRARY_PATH", "")}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    enable_rviz = LaunchConfiguration('enable_rviz')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_map = DeclareLaunchArgument('map', default_value=map_dir)
    declare_params = DeclareLaunchArgument('params_file', default_value=param_dir)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    declare_lidar_x = DeclareLaunchArgument('lidar_x', default_value='0.25')
    declare_lidar_y = DeclareLaunchArgument('lidar_y', default_value='0.192')
    declare_lidar_z = DeclareLaunchArgument('lidar_z', default_value='0.2955')
    declare_lidar_roll = DeclareLaunchArgument('lidar_roll', default_value='0.0')
    declare_lidar_pitch = DeclareLaunchArgument('lidar_pitch', default_value='0.0')
    declare_lidar_yaw = DeclareLaunchArgument('lidar_yaw', default_value='0.0')
    declare_enable_rviz = DeclareLaunchArgument('enable_rviz', default_value='true')

    # robot_description
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )

    static_tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_link_tf',
        arguments=[lidar_x, lidar_y, lidar_z, lidar_roll, lidar_pitch, lidar_yaw, 'base_link', 'livox_frame'],
        output='screen',
    )

    # ✅ IK / FK：继续用你 python 脚本（但脚本内容换成差动化版本）
    # ✅ IK / FK：使用差分运动学 (differential_IK / differential_FK)
    # FK (Wheel -> Odom) 替代原有的 forward_kinematics_node (原 executable=inverse_kinimatic 命名混乱)
    forward_kinematics_node = Node(
        package='evbhuman_description',
        executable='differential_FK.py',  # 修正为 differential_FK.py
        name='differential_fp_kinematics', # 改个名避免误会? 或者保持原名? 保持原变量名但改node name
        output='screen',
        parameters=[{
            'track_width': 0.4,
            'wheel_radius': 0.1,
        }],
    )

    # IK (cmd_vel -> Wheel) 替代原有的 steering_kinematics_node (原 executable=steering_kinematics)
    steering_kinematics_node = Node(
        package='evbhuman_description',
        executable='differential_IK.py',  # 修正为 differential_IK.py 且注意拼写
        name='differential_ip_kinematics',
        output='screen',
        parameters=[{
            'track_width': 0.4,
            'wheel_radius': 0.1,
            # 'traj_duration': 0.5, # 差分 IK 可能不需要这个? 视脚本而定
            # 'vy_to_wz_gain': 0.0,
        }],
    )

    # wheel odom：保留你原来的积分节点（发布 /odom_wheel）
    wheel_odom_node = Node(
        package='evbhuman_description',
        executable='wheel_odometry_node.py',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom_wheel',
            'base_frame_id': 'base_link',
            'publish_rate': 50.0,
            'publish_tf': False,
        }],
        remappings=[('/odom', '/odom_wheel')],
    )

    # EKF（diff 版参数）
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odom0', '/odom_wheel'),
            ('imu0', '/livox/imu'),
            ('odometry/filtered', '/odom'),
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={'map': map_file, 'use_sim_time': use_sim_time, 'params_file': params_file, 'autostart': autostart}.items()
    )

    rviz_config_file = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_rviz)
    )

    return LaunchDescription([
        set_opencv_env, set_cmake_prefix, set_ld_library,
        declare_use_sim_time, declare_map, declare_params, declare_autostart,
        declare_lidar_x, declare_lidar_y, declare_lidar_z, declare_lidar_roll, declare_lidar_pitch, declare_lidar_yaw,
        declare_enable_rviz,

        robot_state_publisher,
        static_tf_livox,

        steering_kinematics_node,
        forward_kinematics_node,
        wheel_odom_node,
        ekf_filter_node,

        nav2_bringup,
        rviz_node,
    ])
