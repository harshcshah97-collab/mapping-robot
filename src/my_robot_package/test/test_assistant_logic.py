from my_robot_package.assistant_logic import command_for_robot_mode


def test_follow_and_frame_commands_are_compatible_with_tracker():
    assert command_for_robot_mode("follow") == "start_follow"
    assert command_for_robot_mode("follow_behind") == "start_follow"
    assert command_for_robot_mode("keep_frame") == "keep_frame"


def test_capture_commands_pass_through():
    assert command_for_robot_mode("take_photo") == "take_photo"
    assert command_for_robot_mode("start_recording") == "start_recording"
    assert command_for_robot_mode("stop_recording") == "stop_recording"


def test_enrollment_commands_pass_through():
    assert command_for_robot_mode("enroll_target") == "enroll_target"
    assert command_for_robot_mode("clear_enrollment") == "clear_enrollment"


def test_unknown_mode_fails_safe():
    assert command_for_robot_mode("unexpected") == "stop"
