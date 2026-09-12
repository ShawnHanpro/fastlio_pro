from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare("evbhuman_description")
    
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "EVBH_SEAG1_0.urdf"])
    rviz_config_file = PathJoinSubstitution([pkg_share, "rviz", "view.rviz"])
    
    robot_description = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true"
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start joint_state_publisher_gui instead of joint_state_publisher"
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", 
        default_value="true", 
        description="Start RViz2 if true"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])