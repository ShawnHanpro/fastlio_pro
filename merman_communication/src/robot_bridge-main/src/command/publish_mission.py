import importlib
import re

from .result import ok


ACTION = "publish_mission"
ROS_TOPIC = "/mission_topic"


def setup(context):
    context.mission_pub = None
    context.mission_array_type = None
    context.mission_waypoint_type = None
    try:
        msg_module = importlib.import_module("evbhuman_interfaces.msg")
        context.mission_array_type = getattr(msg_module, "MissionArray")

        field_type = context.mission_array_type.get_fields_and_field_types().get("waypoints", "")
        match = re.search(r"sequence<[^/]+/(?:msg/)?([^>]+)>", field_type)
        if not match:
            raise RuntimeError(f"无法解析 waypoints 字段类型: {field_type}")

        context.mission_waypoint_type = getattr(msg_module, match.group(1))
        context.mission_pub = context.create_publisher(context.mission_array_type, ROS_TOPIC, 10)
    except Exception as exc:
        context.get_logger().warn(f"未启用任务发布能力: {exc}")


def execute(context, params):
    if context.mission_pub is None or context.mission_array_type is None or context.mission_waypoint_type is None:
        raise RuntimeError("当前环境不可用 publish_mission，请先确认 evbhuman_interfaces 已安装")

    if not isinstance(params, dict):
        raise ValueError("publish_mission 的 params 必须为对象")

    mission = context.mission_array_type()
    mission.mission_id = str(params.get("mission_id") or "")
    mission.preemptive = bool(params.get("preemptive", False))

    for item in params.get("waypoints") or []:
        waypoint = context.mission_waypoint_type()
        waypoint.id = int(item.get("id"))
        if hasattr(waypoint, "gesture") and item.get("gesture") is not None:
            waypoint.gesture = str(item.get("gesture"))
        mission.waypoints.append(waypoint)

    context.mission_pub.publish(mission)
    count = len(mission.waypoints)
    context.get_logger().info(
        f"已发布多任务: topic={ROS_TOPIC}, mission_id={mission.mission_id}, waypoints={count}"
    )
    return ok(f"publish_mission executed, waypoints={count}")
