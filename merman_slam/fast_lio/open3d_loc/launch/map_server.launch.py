import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # loc_param_g1.yaml
    # ============================================================

    open3d_loc_share = get_package_share_directory('open3d_loc')

    config_file = os.path.join(
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    )

    # ============================================================
    # 从 Open3D 配置文件读取 2D map 路径
    # ============================================================

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    params = config[
        'global_localization_node'
    ]['ros__parameters']

    map_yaml = params['path_map_2d']

    print(f'[map_server] config: {config_file}')
    print(f'[map_server] map: {map_yaml}')

    # ============================================================
    # launch 参数
    # ============================================================

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ============================================================
    # map_server
    # ============================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',

        parameters=[{
            'use_sim_time': use_sim_time,

            # 直接使用 loc_param_g1.yaml 中的 path_map_2d
            'yaml_filename': map_yaml,

            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    # ============================================================
    # lifecycle manager
    #
    # autostart=false
    # map_server 生命周期由 Open3D 控制
    # ============================================================

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',

        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': ['map_server'],
        }],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),

        map_server,

        lifecycle_manager,
    ])