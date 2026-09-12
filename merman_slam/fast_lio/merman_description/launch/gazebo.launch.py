from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare("evbhuman_description")
    
    # 控制器配置文件
    controller_config = PathJoinSubstitution([
        pkg_share, "config", "controllers.yaml"
    ])

    # 默认 URDF 与 world
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "EVBH_SEAG1_0.urdf"])
    default_world = PathJoinSubstitution([pkg_share, "worlds", "custom_room.world"])
    
    # 读取 URDF 文件内容，使用 ParameterValue 并指定类型为 str
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # -------- Launch 参数 --------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="use Gazebo clock"
    )
    world_arg = DeclareLaunchArgument(
        "world", default_value=[default_world], description="path to .world file"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="spawn x")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="spawn y")
    z_arg = DeclareLaunchArgument("z", default_value="0.0", description="spawn z")
    drive_mode_arg = DeclareLaunchArgument(
        "drive_mode", 
        default_value="ackermann", 
        description="Drive mode: 'swerve' (全向) or 'ackermann' (四轮转向)"
    )

    # -------- 1) robot_state_publisher（可视化/TF用）--------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": robot_description_content
        }],
        output="screen",
    )

    # -------- 2) 启动 Gazebo（只加载 init + factory 两个经典插件）--------
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
        ],
        output="screen",
    )

    # -------- 3) 插入实体（用 -file 读取 URDF，避免依赖 /robot_description 话题）--------
    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-file",  urdf_file,
            "-entity", "evbhuman_robot",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-timeout", "60",
        ],
        output="screen",
    )

    # 延迟插入，确保 Gazebo 完全启动
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    # -------- 4) 启动 ros2_control 控制器管理器 --------
    # 注意：controller_manager 由 gazebo_ros2_control 插件自动启动
    
    # -------- 5) 启动关节状态广播器 --------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    
    # 延迟启动关节状态广播器（等待 Gazebo 和机器人加载）
    delayed_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    # -------- 6) 启动转向控制器 --------
    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_trajectory_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    
    # 延迟启动转向控制器
    delayed_steering_controller = TimerAction(
        period=10.0,
        actions=[steering_controller_spawner]
    )
    
    # -------- 7) 启动驱动控制器 --------
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
    
    # 延迟启动驱动控制器
    delayed_velocity_controller = TimerAction(
        period=12.0,
        actions=[velocity_controller_spawner]
    )
    
    # -------- 8) 启动四轮四转运动学节点 --------
    kinematics_node = Node(
        package="evbhuman_description",
        executable="four_wheel_steering_kinematics.py",
        name="four_wheel_steering_kinematics",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "wheelbase": 0.5,  # 轴距 (m)
            "track_width": 0.4,  # 轮距 (m)
            "wheel_radius": 0.1,  # 轮半径 (m)
            "drive_mode": LaunchConfiguration("drive_mode"),  # 运动学模式
        }],
    )
    
    # 延迟启动运动学节点（等待控制器启动完成）
    delayed_kinematics = TimerAction(
        period=14.0,
        actions=[kinematics_node]
    )
    
    # -------- 9) 启动 RViz --------
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "view.rviz"])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        arguments=["-d", rviz_config]
    )
    
    # 延迟启动 RViz（等待 Gazebo、机器人spawn和TF树建立完成）
    # 使用足够长的延迟确保Gazebo完全启动后再启动RViz
    delayed_rviz = TimerAction(
        period=25.0,  # 18秒延迟，确保Gazebo、world加载、机器人spawn都完成
        actions=[rviz_node]
    )

    return LaunchDescription([
        use_sim_time_arg, world_arg, x_arg, y_arg, z_arg, drive_mode_arg,
        gazebo,
        rsp,
        delayed_spawn,
        delayed_joint_state_broadcaster,
        delayed_steering_controller,
        delayed_velocity_controller,
        delayed_kinematics,
        delayed_rviz,  # 延迟启动RViz，确保Gazebo完全启动
    ])

