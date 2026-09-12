def ok(message: str):
    return {"success": True, "message": message}


def fail(message: str):
    return {"success": False, "message": message}
