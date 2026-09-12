from std_msgs.msg import Int32

from .result import ok


ACTION = "jump_to"
ROS_TOPIC = "/tour_cmd/jump_to"


def setup(context):
    context.jump_to_pub = context.create_publisher(Int32, ROS_TOPIC, 10)


def execute(context, params):
    point_id = params.get("pointId") if isinstance(params, dict) else None
    if point_id is None:
        raise ValueError("jump_to 缺少 params.pointId")

    msg = Int32()
    msg.data = int(point_id)
    context.jump_to_pub.publish(msg)
    context.get_logger().info(f"已发布跳点: topic={ROS_TOPIC}, data={msg.data}")
    return ok("jump_to executed")
