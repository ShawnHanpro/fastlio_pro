from .result import fail


ACTION = "photograph"
MESSAGE = "photograph 暂未接入 ROS"


def execute(context, params):
    return fail(MESSAGE)
