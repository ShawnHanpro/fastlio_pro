import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


OTHER_CPU_CORE = EnvironmentVariable('ROBOT_OTHER_CPU_CORE', default_value='7')


def affinity_prefix(cpu_core):
    return ['taskset -c ', cpu_core]


def get_description_package_share():
    for package_name in ('merman_description', 'evbhuman_description'):
        try:
            return get_package_share_directory(package_name)
        except PackageNotFoundError:
            continue

    raise PackageNotFoundError(
        "Neither 'merman_description' nor 'evbhuman_description' was found"
    )


def generate_launch_description():
    # ================================
    # 可写数据目录（使用家目录，避免 install/share/ 只读目录）
    # ================================
    home_dir = os.path.expanduser('~')
    db_dir = os.path.join(home_dir, 'rtabmap_maps')
    os.makedirs(db_dir, exist_ok=True)
    
    # ================================
    # 包目录和配置文件路径
    # ================================
    nav2_pkg = get_package_share_directory('nav2')
    # 地图 yaml 文件（已被移除，因 RTAB-Map 直接提供 /map 话题）
    # param_dir 等保留
    param_dir = os.path.join(nav2_pkg, 'param', 'evbhuman_swerve_navigation2.yaml')
    nav2_launch_file = os.path.join(nav2_pkg, 'launch', 'navigation_affinity.launch.py')
    
    # URDF 文件路径（使用 ament_index 动态获取，避免硬编码用户目录）
    description_pkg = get_description_package_share()
    urdf_file = os.path.join(description_pkg, 'urdf', 'EVBH_SEAG1_0_real.urdf')
    
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
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 是否启用 RViz
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
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
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='启用 RViz2 可视化（默认关闭）')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(nav2_pkg, 'rviz', 'fastlio_nav2_view.rviz'),
        description='RViz2 configuration file')
    
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
    # 注意：以下节点由 start_robot.sh 中的独立 launch 文件统一管理，
    #   此处不再重复启动，避免 TF authority 冲突：
    #   - base_link → livox_frame 静态 TF → rtabmap_localization.launch.py
    #   - base_link → camera_link 静态 TF → realsense_d435.launch.py
    #   - RTAB-Map 定位节点 → rtabmap_localization.launch.py
    #   - RealSense D435 相机驱动 → realsense_d435.launch.py
    # ================================

    
    # ================================
    # 8) Nav2 导航栈
    # ================================
    # 注意：改用本地 navigation_affinity.launch.py 直接启动核心导航节点，
    #   每个 Nav2 进程在该文件里单独设置 CPU affinity。
    #   同时跳过 bringup_launch.py 调用的 localization_launch.py (amcl + map_server)。
    #   map→odom TF 和 /map 话题均由 rtabmap_localization.launch 提供。
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
    # 9) RViz2 可视化（Nav2 配置）
    # ================================
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
    # 10) Nav2 接收端话题适配
    # ================================
    # FAST-LIO/Open3D 保持原生 /Odometry_loc，点云转换节点发布
    # /nav2_scan；仅在 Nav2 及其 RViz 的作用域内映射为它们预期的话题名。
    nav2_consumers = GroupAction(actions=[
        SetRemap(src='/odom', dst='/Odometry_loc'),
        SetRemap(src='/scan', dst='/nav2_scan'),
        nav2_bringup,
        rviz_node,
    ])
    
    # ================================
    # 组装 LaunchDescription
    # ================================
    # 基础节点列表
    launch_nodes = [
        # OpenCV 环境变量
        set_opencv_env,
        set_cmake_prefix,
        set_ld_library,
        
        # 参数声明
        declare_use_sim_time,
        declare_params,
        declare_autostart,
        declare_enable_rviz,
        declare_rviz_config_file,
        
        # 核心节点
        # 注意：livox TF、camera TF、RTAB-Map 均由独立 launch 文件管理
        robot_state_publisher,  # URDF 加载和 TF 树发布
        nav2_consumers,
    ]
    
    # 注释掉：不再自动添加 Livox 驱动启动项（手动启动）
    # if livox_rviz_launch is not None:
    #     launch_nodes.append(livox_rviz_launch)
    
    return LaunchDescription(launch_nodes)
