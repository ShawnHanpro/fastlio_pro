#!/usr/bin/env python3

import math
import os
import sys
import threading
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


POINT_TYPE_TASK = 'task'
POINT_TYPE_VIA = 'via'


def quaternion_to_theta(quaternion) -> float:
    """
    将四元数转换为平面旋转角 theta，单位为弧度。
    """
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


class PoseSaver(Node):
    def __init__(self, pose_topic: str):
        super().__init__('pose_saver')

        self.pose_topic = pose_topic
        self.latest_msg: Optional[PoseStamped] = None
        self.pose_mutex = threading.Lock()

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'订阅位姿话题: {self.pose_topic}'
        )

    def pose_callback(
        self,
        msg: PoseStamped
    ) -> None:
        with self.pose_mutex:
            self.latest_msg = msg

    def has_pose(self) -> bool:
        with self.pose_mutex:
            return self.latest_msg is not None

    def get_current_pose(self) -> Optional[dict]:
        with self.pose_mutex:
            if self.latest_msg is None:
                return None

            pose = self.latest_msg.pose

            x = pose.position.x
            y = pose.position.y
            theta = quaternion_to_theta(pose.orientation)

        return {
            'x': round(float(x), 4),
            'y': round(float(y), 4),
            'theta': round(float(theta), 4),
        }


def load_yaml(file_path: str) -> dict:
    if not os.path.exists(file_path):
        return {
            'points': []
        }

    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as error:
        print(f'读取 YAML 文件失败: {error}')
        return {
            'points': []
        }

    if not isinstance(data, dict):
        data = {}

    if 'points' not in data or not isinstance(data['points'], list):
        data['points'] = []

    return data


def save_yaml(file_path: str, data: dict) -> bool:
    try:
        with open(file_path, 'w', encoding='utf-8') as file:
            yaml.safe_dump(
                data,
                file,
                default_flow_style=False,
                sort_keys=False,
                allow_unicode=True
            )
        return True

    except (OSError, yaml.YAMLError) as error:
        print(f'保存 YAML 文件失败: {error}')
        return False


def ros_spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception as error:
        node.get_logger().error(
            f'ROS spin 线程异常: {error}'
        )


def parse_point_type(user_input: str) -> Optional[str]:
    normalized = user_input.strip().lower()
    if normalized in {'t', 'task'}:
        return POINT_TYPE_TASK
    if normalized in {'v', 'via'}:
        return POINT_TYPE_VIA
    return None


def next_waypoint_id(points: list) -> int:
    valid_ids = []
    for point in points:
        if not isinstance(point, dict):
            continue
        try:
            waypoint_id = int(point.get('id', 0))
        except (TypeError, ValueError):
            continue
        if waypoint_id > 0:
            valid_ids.append(waypoint_id)

    return max(valid_ids, default=0) + 1


def print_pose(label: str, pose: dict) -> None:
    print(
        f'{label}: '
        f'x={pose["x"]}, '
        f'y={pose["y"]}, '
        f'theta={pose["theta"]}'
    )


def main() -> None:
    rclpy.init()

    # 默认订阅的话题。
    # 也可以通过命令行参数传入其他话题：
    #
    # python3 save_waypoints.py /localization_3d
    #
    pose_topic = '/localization_3d'

    if len(sys.argv) >= 2:
        pose_topic = sys.argv[1]

    # 获取当前 Python 脚本所在目录。
    script_directory = os.path.dirname(
        os.path.abspath(__file__)
    )

    yaml_path = os.path.join(
        script_directory,
        'waypoints.yaml'
    )

    node = PoseSaver(pose_topic)

    # 独立线程持续处理 ROS 回调。
    # 这样即使主线程正在等待 input()，位姿信息仍然会持续更新。
    spin_thread = threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    print(f'\n正在等待话题 {pose_topic} 的位姿数据……')

    try:
        while rclpy.ok() and not node.has_pose():
            spin_thread.join(timeout=0.1)

        if not rclpy.ok():
            return

        print('\n已经收到位姿数据，可以开始采点。')
        print('将机器人移动到目标位置并调整好朝向。')
        print('输入 p 预览当前位姿，不保存点位。')
        print('输入 t 保存任务点 task。')
        print('输入 v 保存途经点 via。')
        print('输入 q、quit 或 exit 退出。')
        print(f'点位文件保存路径: {yaml_path}\n')

        while rclpy.ok():
            try:
                user_input = input('操作 [p/t/v] > ').strip()
            except EOFError:
                break

            normalized_input = user_input.lower()

            if normalized_input in {
                'q',
                'quit',
                'exit'
            }:
                break

            if normalized_input in {'p', 'pose'}:
                current_pose = node.get_current_pose()
                if current_pose is None:
                    print(
                        f'还没有收到 {pose_topic} 的位姿数据，'
                        '请检查话题是否正常发布。'
                    )
                    continue

                print_pose('当前位姿', current_pose)
                continue

            point_type = parse_point_type(user_input)
            if point_type is None:
                print('无效输入，请输入 p、t 或 v。')
                continue

            current_pose = node.get_current_pose()

            if current_pose is None:
                print(
                    f'还没有收到 {pose_topic} 的位姿数据，'
                    '请检查话题是否正常发布。'
                )
                continue

            print_pose('准备保存的位姿', current_pose)

            try:
                confirmation = input(
                    '确认保存这个点位吗？[y/N] > '
                ).strip().lower()
            except EOFError:
                break

            if confirmation not in {'y', 'yes'}:
                print('已取消，本次点位未保存。\n')
                continue

            yaml_data = load_yaml(yaml_path)
            waypoint_id = next_waypoint_id(yaml_data['points'])
            waypoint = {
                'id': waypoint_id,
                'type': point_type,
                **current_pose,
            }
            yaml_data['points'].append(waypoint)
            yaml_data['points'].sort(
                key=lambda point: int(point.get('id', 0))
            )

            if not save_yaml(yaml_path, yaml_data):
                continue

            print(
                f'已保存点位 id={waypoint_id}, '
                f'type={point_type}: '
                f'x={current_pose["x"]}, '
                f'y={current_pose["y"]}, '
                f'theta={current_pose["theta"]}'
            )
            print(f'保存文件: {yaml_path}\n')

    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，程序退出。')

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
