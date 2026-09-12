from std_msgs.msg import Int32


ROS_TOPIC = "/audio_done"


def register_audio_done_event(node):
    def on_audio_done(msg: Int32):
        node.get_logger().info(f"监听到 ROS2 消息: topic={ROS_TOPIC}, data={msg.data}")

    try:
        subscription = node.create_subscription(Int32, ROS_TOPIC, on_audio_done, 10)
        node.get_logger().info(f"已订阅音频结束: ros2 topic={ROS_TOPIC}, msg_type=Int32")
        return subscription
    except Exception as exc:
        node.get_logger().warn(f"未启用音频结束监听能力: {exc}")
        return None
