import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


SLAM_CPU_CORE = EnvironmentVariable('ROBOT_SLAM_CPU_CORE', default_value='5')
NAV2_CPU_CORE = EnvironmentVariable('ROBOT_NAV2_CPU_CORE', default_value='6')
OTHER_CPU_CORE = EnvironmentVariable('ROBOT_OTHER_CPU_CORE', default_value='7')


def affinity_prefix(cpu_core):
    return ['taskset -c ', cpu_core]


def generate_launch_description():
    # ================================
    # 确保数据库目录存在（支持 root 用户运行）
    # ================================
    db_dir = '/home/niic/20250123_final002/ws_wyc/motion_control_chassis/src/nav2/map'
    os.makedirs(db_dir, exist_ok=True)
    
    # ================================
    # 包目录和配置文件路径
    # ================================
    nav2_pkg = get_package_share_directory('nav2')
    evbhuman_slam_pkg = get_package_share_directory('evbhuman_slam')
    evbhuman_base_pkg = get_package_share_directory('evbhuman_base')
    # 配置文件路径
    map_dir = '/home/niic/20250123_final002/ws_wyc/motion_control_chassis/src/nav2/map/rtabmap_lidar.yaml'
    param_dir = os.path.join(nav2_pkg, 'param', 'evbhuman_swerve_navigation2.yaml')
    ekf_config_file = os.path.join(evbhuman_slam_pkg, 'config', 'ekf_odom.yaml')
    nav2_launch_file = os.path.join(nav2_pkg, 'launch', 'navigation_affinity.launch.py')
    db_path = '/home/niic/20250123_final002/ws_wyc/motion_control_chassis/src/nav2/map/rtabmap_lidar.db'
    
    # URDF 文件路径
    urdf_file = '/home/niic/20250123_final002/ws_wyc/motion_control_chassis/src/evbhuman_description/urdf/EVBH_SEAG1_0_real.urdf'
    
    # ================================
    # OpenCV 环境变量设置（RTAB-Map 需要）
    # ================================
    opencv_prefix = os.path.expanduser('~/third_party/opencv_aruco')
    opencv_dir = os.path.join(opencv_prefix, 'lib/cmake/opencv4')
    
    set_opencv_env = SetEnvironmentVariable('OpenCV_DIR', opencv_dir)
    set_cmake_prefix = SetEnvironmentVariable(
        'CMAKE_PREFIX_PATH',
        f'{opencv_prefix}:{os.environ.get("CMAKE_PREFIX_PATH", "")}'
    )
    set_ld_library = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        f'{os.path.join(opencv_prefix, "lib")}:{os.environ.get("LD_LIBRARY_PATH", "")}'
    )
    
    # ================================
    # Launch 参数声明
    # ================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 雷达安装位置参数（相对于 base_link）
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    
    # 是否启用 RTAB-Map 和 RViz
    enable_rtabmap = LaunchConfiguration('enable_rtabmap')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load')
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load')
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # 打印 autostart 参数用于调试
    print(f"[INFO] Nav2 autostart 参数: true (默认启用自动启动)")
    
    declare_lidar_x = DeclareLaunchArgument(
        'lidar_x',
        default_value='0.25',
        description='雷达 X 位置（相对于 base_link，单位：米）')
    
    declare_lidar_y = DeclareLaunchArgument(
        'lidar_y',
        default_value='0.192',
        description='雷达 Y 位置（相对于 base_link，单位：米）')
    
    declare_lidar_z = DeclareLaunchArgument(
        'lidar_z',
        default_value='0.2955',
        description='雷达 Z 位置（相对于 base_link，单位：米）')
    
    declare_lidar_roll = DeclareLaunchArgument(
        'lidar_roll',
        default_value='0.0',
        description='雷达 Roll 角度（单位：弧度）')
    
    declare_lidar_pitch = DeclareLaunchArgument(
        'lidar_pitch',
        default_value='0.0',
        description='雷达 Pitch 角度（单位：弧度）')
    
    declare_lidar_yaw = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='雷达 Yaw 角度（单位：弧度）')
    
    declare_enable_rtabmap = DeclareLaunchArgument(
        'enable_rtabmap',
        default_value='false',
        description='启用 RTAB-Map 建图/定位（如果已有地图且只需定位，可设为 false）')
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='启用 RViz2 可视化（默认启用）')
    
    # ================================
    # 读取 URDF 文件内容
    # ================================
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    # ================================
    # 0) Robot State Publisher（发布机器人 TF 树）
    # ================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ================================
    # 1) 静态 TF 变换：base_link → livox_frame
    # ================================
    static_tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_base_link_tf',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        arguments=[
            lidar_x, lidar_y, lidar_z,
            lidar_roll, lidar_pitch, lidar_yaw,
            'base_link', 'livox_frame'
        ],
        output='screen',
    )
    
    # ================================
    # 2) 正运动学节点（C++ 实现，集成里程计功能）
    # ================================
    # 注意：此节点发布 /odom_wheel，由 EKF 融合后发布 /odom
    forward_kinematics_node = Node(
        package='evbhuman_base',  # 使用新的 C++ 包
        executable='forward_kinematics_node',  # C++ 可执行文件
        name='four_wheel_inverse_kinematics',
        output='screen',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        parameters=[{
            'drive_mode': 'swerve',
            'wheelbase': 0.5,
            'track_width': 0.4,
            'wheel_radius': 0.1,
            'publish_tf': False,  # 禁用 TF（由 EKF 发布）
        }],
    )
    
    # ================================
    # 3) 逆运动学节点（C++ 实现）
    # ================================
    steering_kinematics_node = Node(
        package='evbhuman_base',  # 使用新的 C++ 包
        executable='steering_kinematics_node',  # C++ 可执行文件
        name='four_wheel_steering_kinematics',
        output='screen',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        parameters=[{
            'drive_mode': 'swerve',
            'wheelbase': 0.5,
            'track_width': 0.4,
            'wheel_radius': 0.1,
            'traj_duration': 0.5,
        }],
    )
    
    # ================================
    # 4) EKF 融合节点（轮式里程计 + IMU → /odom）
    # ================================
    # 注意：正运动学节点发布 /odom_wheel，EKF 从中读取并融合 IMU
    # 如果不需要融合 IMU，可以注释掉此节点，并设置正运动学节点的 publish_tf=True

    # ekf_filter_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
    #     remappings=[
    #         # ('odom0', '/odom'),  # 删除无效映射：EKF 通过 config 文件中的 odom0 参数读取 /odom_wheel
    #         ('imu0', '/livox/imu'),
    #         ('odometry/filtered', '/odom'),  # 融合后仍发布到 /odom
    #     ],
    # )
    
    # ================================
    # 5) RTAB-Map 节点（建图/定位，发布 map → odom）
    # ================================
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(enable_rtabmap),
        prefix=affinity_prefix(SLAM_CPU_CORE),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_odom': True,
            'publish_tf': True,
            'wait_for_transform': 0.2,
            
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'scan_cloud_max_points': 200000,
            
            # ICP 配置
            'Reg/Strategy': '1',
            'Icp/InitMethod': '0',
            'Icp/GuessFromTf': 'true',
            'Icp/MaxCorrespondenceDistance': '3.0',
            'Icp/VoxelSize': '0.1',
            'Icp/Iterations': '60',
            'Icp/Epsilon': '0.0001',
            'Icp/OutlierRatio': '0.8',
            'Icp/CorrespondenceRatio': '0.1',
            'Icp/PointToPlaneRadius': '0.0',
            'Icp/Strategy': '1',
            'Icp/MaxTranslation': '1.0',
            'Icp/MaxRotation': '0.78',
            
            # 时间同步
            'tf_delay': 0.15,
            'tf_tolerance': 0.3,
            
            # 地图更新
            'RGBD/LinearUpdate': '0.08',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/LocalRadius': '5.0',
            
            # 回环检测
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityByTime': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/ProximityPathRadius': '1.0',
            'RGBD/ProximityAngle': '0.0',
            'RGBD/LoopClosureReextractFeatures': 'false',
            
            # 里程计使用
            'Odom/ResetCountdown': '1',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'false',
            'Odom/InlierDistance': '0.3',
            'Odom/InlierAngle': '0.5',
            'Odom/FillInfoData': 'false',
            
            # 图优化
            'Optimizer/Strategy': '1',
            'Optimizer/Iterations': '20',
            'Optimizer/Robust': 'true',
            'Optimizer/Pose3Sigma': '1.5',
            'Optimizer/GravitySigma': '0.0',
            'Optimizer/Epsilon': '0.001',
            'Optimizer/VarianceIgnored': 'false',
            
            # 检测率
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr': '0',
            'Mem/ImagePreDecimation': '1',
            'Mem/ImagePostDecimation': '1',
            'Mem/RehearsalSimilarity': '0.6',
            
            # 地图输出
            'Grid/RangeMin': '0.0',
            'Grid/RangeMax': '20.0',
            'Grid/RayTracing': 'true',
            'Grid/FootprintRadius': '0.0',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '0.2',
            'Grid/2D': 'false',
            'Grid/3D': 'true',
            'Grid/FromDepth': 'false',
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'Grid/CellSize': '0.05',
            'cloud_output_voxelized': True,
            'publish_map': True,
            'publish_map_data': True,
            'publish_map_graph': True,
            'publish_stats': True,
            
            # 数据库（使用动态路径）
            'database_path': db_path,
            'delete_db_on_start': False,
        }],
        remappings=[
            ('scan_cloud', '/livox/lidar'),
            ('scan', '/scan_disabled'),
        ],
    )
    
    # ================================
    # 6) Nav2 导航栈
    # ================================
    # 注意：传入 localization=True 和 autostart=True
    #   - localization=True 跳过 amcl 和 map_server 生命周期管理
    #     （map->odom TF 和 /map 话题均由 rtabmap 提供）
    #   - 不需要传 map 文件（rtabmap 直接发布 /map）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time':  use_sim_time,
            'params_file':   params_file,
            'autostart':     autostart,
            'use_composition': 'False',
        }.items()
    )
    
    # ================================
    # 7) RViz2 可视化（Nav2 配置）
    # ================================
    rviz_config_file = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        arguments=[
            '-d', rviz_config_file,
            '--ros-args',
            '--log-level', 'warn'  # 只显示警告和错误，减少日志输出
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(enable_rviz)
    )
    
    # ================================
    # 组装 LaunchDescription
    # ================================
    launch_nodes = [
        # OpenCV 环境变量
        set_opencv_env,
        set_cmake_prefix,
        set_ld_library,
        
        # 参数声明
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        declare_lidar_x,
        declare_lidar_y,
        declare_lidar_z,
        declare_lidar_roll,
        declare_lidar_pitch,
        declare_lidar_yaw,
        declare_enable_rtabmap,
        declare_enable_rviz,
        
        # 核心节点
        robot_state_publisher,  # URDF 加载和 TF 树发布
        static_tf_livox,
        forward_kinematics_node,      # 新的 C++ 正运动学节点（集成里程计）
        steering_kinematics_node,     # 新的 C++ 逆运动学节点
        # ekf_filter_node,              # EKF 融合（融合 IMU）
        rtabmap_node,                 # RTAB-Map SLAM
        nav2_bringup,                 # Nav2 导航栈
        rviz_node,                    # Nav2 可视化
    ]
    
    return LaunchDescription(launch_nodes)
