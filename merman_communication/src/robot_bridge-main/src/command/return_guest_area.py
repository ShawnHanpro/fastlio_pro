from std_srvs.srv import Trigger

from .result import ok


ACTION = "return_guest_area"
SERVICE_NAME = "/robot/go_welcome"


def setup(context):
    context.go_welcome_client = context.create_client(Trigger, SERVICE_NAME)


def execute(context, params):
    client = context.go_welcome_client
    if client is None:
        raise RuntimeError(f"Trigger client 未初始化: {SERVICE_NAME}")

    if not client.service_is_ready():
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"服务不可用: {SERVICE_NAME}")

    future = client.call_async(Trigger.Request())

    def _done(fut):
        try:
            response = fut.result()
        except Exception as exc:
            context.get_logger().error(f"服务调用失败: {SERVICE_NAME}, error={exc}")
            return

        if getattr(response, "success", False):
            context.get_logger().info(f"服务调用成功: {SERVICE_NAME}, message={getattr(response, 'message', '')}")
        else:
            context.get_logger().warn(f"服务调用返回失败: {SERVICE_NAME}, message={getattr(response, 'message', '')}")

    future.add_done_callback(_done)
    return ok("return_guest_area requested")
