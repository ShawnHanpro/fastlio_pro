from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    
    # 注意：robot_state_publisher 已在 gazebo.launch.py 中启动，此处不再重复启动

    # ================================
    # 1) ICP 里程计节点（生成 odom → base_link）
    # ================================
    icp_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # TF 设置
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,                # ★ 发布 odom → base_link
            'wait_for_transform': 0.5,         # ★ 等待变换的超时时间

            # 输入：仅 3D 点云
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'scan_cloud_max_points': 200000,

            # ICP 参数
            'Reg/Strategy': '1',
            'Icp/InitMethod': '0',             # ★ 0=恒等变换作为初始猜测，解决 null guess 错误
            'Icp/GuessFromTf': 'true',         # ★ 从 TF 获取初始猜测（如果可用）
            'Icp/GuessMinTranslation': '0.0',   # ★ 最小平移阈值
            'Icp/GuessMinRotation': '0.0',     # ★ 最小旋转阈值
            'Icp/MaxCorrespondenceDistance': '2.0',  # ★ 增大对应距离，更宽松
            'Icp/VoxelSize': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.0001',
            'Icp/OutlierRatio': '0.8',         # ★ 允许更多异常值
            'Icp/MaxTranslation': '2.0',       # ★ 增大最大平移限制
            'Icp/MaxRotation': '1.57',         # ★ 增大最大旋转限制（约90度）
            
            # 里程计参数
            'Odom/ResetCountdown': '1',        # ★ 重置计数器
            'Odom/Strategy': '0',              # ★ 0=Frame-to-Map
        }],
        remappings=[
            ('scan_cloud', '/left_front_lidar_PointCloud2'),
        ],
    )

    # ================================
    # 2) RTAB-Map 主节点（建图，发布 map → odom）
    # ================================
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # TF 设置
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',           # ★ 现在依赖 odom
            'map_frame_id': 'map',
            'subscribe_odom': True,            # ★ 必须开启！
            'publish_tf': True,                # ★ 发布 map → odom
            'wait_for_transform': 0.5,         # ★ 等待变换的超时时间，解决时间同步问题

            # 输入：3D 点云
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'scan_cloud_max_points': 200000,

            # ICP 配置（作为 front-end）
            'Reg/Strategy': '1',
            'Icp/InitMethod': '0',             # ★ 恒等变换作为初始猜测
            'Icp/GuessFromTf': 'true',         # ★ 从 TF 获取初始猜测
            'Icp/MaxCorrespondenceDistance': '2.0',  # ★ 增大对应距离
            'Icp/VoxelSize': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.0001',
            'Icp/OutlierRatio': '0.8',         # ★ 允许更多异常值
            
            # 时间同步参数
            'tf_delay': 0.05,                  # ★ TF 延迟补偿
            'tf_tolerance': 0.1,                # ★ TF 容差

            # 地图输出
            'Grid/2D': 'false',
            'Grid/3D': 'true',
            'Grid/FromDepth': 'false',
            'RGBD/CreateOccupancyGrid': 'true',  # ★ 设置为 true 以避免警告

            'publish_map': True,
            'publish_map_data': True,
            'publish_map_graph': True,
            'publish_stats': True,

            # DB
            # 将数据库写入 Nav2 map 目录，便于后续 map_server 使用
            'database_path': '/home/test/workspace/motion_control/src/evbhuman_nav2/map/rtabmap_lidar.db',
            'delete_db_on_start': False,
        }],
        remappings=[
            ('scan_cloud', '/left_front_lidar_PointCloud2'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        icp_odom_node,          # ★ 启动里程计节点
        rtabmap_node,           # ★ 启动建图节点
    ])

