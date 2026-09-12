from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    fast_lio_share = FindPackageShare('fast_lio')

    open3d_loc_share = FindPackageShare('open3d_loc')

    # ============================================================
    # FAST-LIO
    # ============================================================

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share,
                'launch',
                'mapping.launch.py'
            ])
        ])
    )

    # ============================================================
    # Open3D localization
    # ============================================================

    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ])
    )

    # ============================================================
    # map_server
    #
    # 地图路径由 map_server.launch.py 自己读取
    # ============================================================

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'map_server.launch.py'
            ])
        ])
    )

    # ============================================================

    return LaunchDescription([
        fast_lio_launch,
        open3d_loc_launch,
        map_server_launch,
    ])