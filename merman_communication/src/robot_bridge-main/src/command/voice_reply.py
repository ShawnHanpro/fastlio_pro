from .result import fail


ACTION = "voice_reply"
MESSAGE = "voice_reply 仅由 voice 项目处理，robot_bridge 不执行 TTS"


def execute(context, params):
    return fail(MESSAGE)
