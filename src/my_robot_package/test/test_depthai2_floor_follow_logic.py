import math

from my_robot_package.depthai2_floor_follow_logic import (
    FollowLimits,
    bounded_follow_command,
    current_throttle_bits,
    normalize_angle,
    planar_distance,
)


def test_follow_is_forward_only_and_bounded():
    limits = FollowLimits()
    linear, angular = bounded_follow_command(0.5, 2.5, limits)
    assert 0.0 < linear <= 0.05
    assert angular == 0.0

    linear, _ = bounded_follow_command(0.5, 0.55, limits)
    assert linear == 0.0

    linear, _ = bounded_follow_command(0.5, 1.0, limits)
    assert linear > 0.0


def test_turns_toward_person_and_rotates_before_translating():
    limits = FollowLimits()
    linear, angular = bounded_follow_command(0.8, 2.5, limits)
    assert linear == 0.0
    assert -0.15 <= angular < 0.0

    linear, angular = bounded_follow_command(0.2, 2.5, limits)
    assert linear == 0.0
    assert 0.0 < angular <= 0.15


def test_invalid_or_blocked_observations_are_zero():
    limits = FollowLimits()
    for center, distance in ((math.nan, 2.0), (0.5, math.inf), (-0.1, 2.0)):
        assert bounded_follow_command(center, distance, limits) == (0.0, 0.0)
    assert bounded_follow_command(0.5, 2.0, limits, blocked=True) == (0.0, 0.0)


def test_budget_helpers():
    assert math.isclose(planar_distance((0.0, 0.0), (0.3, 0.4)), 0.5)
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert current_throttle_bits(0xE0000) == 0
    assert current_throttle_bits(0xE0008) == 8
