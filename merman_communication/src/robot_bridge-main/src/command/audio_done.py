from std_msgs.msg import Int32

from .result import ok


ACTION = "audio_done"
ROS_TOPIC = "/audio_done"


def setup(context):
    context.audio_done_pub = context.create_publisher(Int32, ROS_TOPIC, 10)


def extract_int_value(params, action_name: str) -> int:
    if not isinstance(params, dict):
        raise ValueError(f"{action_name} 参数必须为对象")
    for key in ("data", "value", "station_id", "stationId", "id"):
        if key in params and params.get(key) is not None:
            return int(params.get(key))
    raise ValueError(f"{action_name} 缺少 data/value")


def execute(context, params):
    value = extract_int_value(params, ACTION)
    msg = Int32()
    msg.data = int(value)
    context.audio_done_pub.publish(msg)
    context.get_logger().info(f"已发布音频结束: ros2 topic={ROS_TOPIC}, data={msg.data}")
    return ok(f"audio_done executed: {value}")
