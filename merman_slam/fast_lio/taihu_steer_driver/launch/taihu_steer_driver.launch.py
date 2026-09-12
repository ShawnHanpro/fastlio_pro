import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    ld = LaunchDescription()
    driver_config = os.path.join(get_package_share_directory('taihu_steer_driver'),'config','taihu_steer_driver_config.yaml')

    with open(driver_config,'r') as f:
        params = yaml.safe_load(f)["taihu_steer_driver"]["ros__parameters"]

    taihu_driver = Node(
            package= "taihu_steer_driver",                 #功能包。
            executable= "taihu_steer_driver_node",         #节点。
            parameters= [driver_config
                ],             #接入参数文件
            namespace=namespace, 
            output= 'screen'
            )
    ld.add_action(taihu_driver)

    return ld
