from dataclasses import dataclass
from enum import Enum
import threading

from . import audio_done
from . import jump_to
from . import move
from . import photograph
from . import publish_mission
from . import return_charging_position
from . import return_guest_area
from . import voice_reply


COMMAND_MODULES = (
    jump_to,
    return_guest_area,
    return_charging_position,
    publish_mission,
    audio_done,
    photograph,
    voice_reply,
    move,
)


class CommandState(str, Enum):
    RECEIVED = "received"
    DISPATCHING = "dispatching"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True)
class CommandHandler:
    action: str
    execute: object
    pass_action: bool = False


class RobotCommandStateMachine:
    def __init__(self, context):
        self.context = context
        self._setup_handlers()
        self.handlers = self._build_handlers()
        self._command_states: dict[str, CommandState] = {}
        self._lock = threading.Lock()

    def execute(self, action: str, params):
        action = str(action or "").strip().lower()
        handler = self.handlers.get(action)
        if handler is None:
            raise ValueError(f"未定义的 action: {action}")

        try:
            if handler.pass_action:
                result = handler.execute(self.context, params, action)
            else:
                result = handler.execute(self.context, params)
        except Exception:
            raise
        return result

    def get_command_state(self, uid: str) -> CommandState | None:
        with self._lock:
            return self._command_states.get(uid)

    def set_command_state(self, uid: str, state: CommandState) -> None:
        with self._lock:
            self._command_states[uid] = state

    def _setup_handlers(self):
        for module in COMMAND_MODULES:
            setup = getattr(module, "setup", None)
            if setup is not None:
                setup(self.context)

    @staticmethod
    def _build_handlers():
        handlers = {
            module.ACTION: CommandHandler(action=module.ACTION, execute=module.execute)
            for module in COMMAND_MODULES
            if hasattr(module, "ACTION")
        }
        handlers.update(
            {
                action: CommandHandler(action=action, execute=move.execute, pass_action=True)
                for action in move.ACTIONS
            }
        )
        return handlers
