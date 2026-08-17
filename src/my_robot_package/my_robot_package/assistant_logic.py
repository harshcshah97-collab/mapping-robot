"""Pure command translations used by the voice assistant."""

ROBOT_MODE_COMMANDS = {
    "follow": "start_follow",
    "follow_behind": "start_follow",
    "keep_frame": "keep_frame",
    "enroll_target": "enroll_target",
    "clear_enrollment": "clear_enrollment",
    "take_photo": "take_photo",
    "start_recording": "start_recording",
    "stop_recording": "stop_recording",
    "orbit": "orbit",
    "stop": "stop",
}


def command_for_robot_mode(mode):
    """Return a safe tracking command for an assistant-selected mode."""
    return ROBOT_MODE_COMMANDS.get(str(mode).strip().lower(), "stop")
