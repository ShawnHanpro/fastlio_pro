from typing import Any


TRUE_VALUES = {"1", "true", "yes", "on", "y"}
FALSE_VALUES = {"0", "false", "no", "off", "n"}


def parse_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default

    text = str(value).strip().lower()
    if text in TRUE_VALUES:
        return True
    if text in FALSE_VALUES:
        return False
    return default
