from geometry_msgs.msg import Twist

from .result import ok


ACTIONS = {
    "forward": (0.2, 0.0),
    "back_off": (-0.2, 0.0),
    "turn_left": (0.0, 0.5),
    "turn_right": (0.0, -0.5),
    "stop_move": (0.0, 0.0),
}
ROS_TOPIC = "/cmd_vel_web"
PUBLISH_HZ = 10


def setup(context):
    context.cmd_vel_pub = context.create_publisher(Twist, ROS_TOPIC, 10)
    context.current_twist = Twist()
    context.last_cmd_vel = None
    context.cmd_vel_timer = context.create_timer(
        1.0 / PUBLISH_HZ,
        lambda: context.cmd_vel_pub.publish(context.current_twist),
    )


def execute(context, params, action: str):
    linear_x, angular_z = ACTIONS[action]
    linear_x = float(linear_x)
    angular_z = float(angular_z)
    current_cmd_vel = (linear_x, angular_z)

    twist = context.current_twist
    twist.linear.x, twist.linear.y, twist.linear.z = linear_x, 0.0, 0.0
    twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, angular_z
    context.cmd_vel_pub.publish(twist)

    if current_cmd_vel != context.last_cmd_vel:
        context.last_cmd_vel = current_cmd_vel
        context.get_logger().info(
            f"已设置底盘控制并开始按 {PUBLISH_HZ}Hz 持续发布: "
            f"topic={ROS_TOPIC}, linear.x={linear_x}, angular.z={angular_z}"
        )
    return ok(f"{action} executed")
