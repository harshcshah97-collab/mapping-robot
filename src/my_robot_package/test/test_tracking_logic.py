from my_robot_package.tracking_logic import compute_motion


DEFAULTS = {
    "target_distance_m": 1.5,
    "distance_deadband_m": 0.2,
    "center_deadband": 0.1,
    "follow_speed": 0.12,
    "turn_speed": 0.4,
}


def motion(mode, visible=True, center=0.5, distance=2.0, blocked=False):
    return compute_motion(
        mode=mode,
        target_visible=visible,
        center_x=center,
        distance_m=distance,
        blocked=blocked,
        **DEFAULTS
    )


def test_idle_never_moves():
    assert motion("IDLE") == (0.0, 0.0)


def test_lost_target_stops():
    assert motion("FOLLOW", visible=False) == (0.0, 0.0)


def test_safety_block_stops():
    assert motion("FOLLOW", blocked=True) == (0.0, 0.0)


def test_person_on_right_turns_right_before_driving():
    linear, angular = motion("FOLLOW", center=0.8)
    assert 0.0 < linear < 0.12
    assert angular < 0.0


def test_person_on_left_turns_left_before_driving():
    linear, angular = motion("FOLLOW", center=0.2)
    assert 0.0 < linear < 0.12
    assert angular > 0.0


def test_centered_distant_person_drives_forward():
    assert motion("FOLLOW", center=0.5, distance=2.0) == (0.12, 0.0)


def test_target_distance_stops_forward_motion():
    assert motion("FOLLOW", center=0.5, distance=1.6) == (0.0, 0.0)


def test_keep_frame_only_turns():
    assert motion("KEEP_FRAME", center=0.5, distance=3.0) == (0.0, 0.0)


def test_target_near_edge_rotates_without_driving():
    linear, angular = motion("FOLLOW", center=0.9)
    assert linear == 0.0
    assert angular < 0.0
