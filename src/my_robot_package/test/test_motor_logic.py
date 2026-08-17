import pytest

from my_robot_package.motor_logic import (
    feedforward_pwm,
    limit_wheel_targets,
    pi_pwm,
    select_drive_action,
    wheel_targets,
)


def action(linear, angular):
    return select_drive_action(linear, angular, 0.05, 0.30)


def test_deadband_stops():
    assert action(0.01, 0.1) == "stop"


def test_straight_commands_select_direction():
    assert action(0.2, 0.0) == "forward"
    assert action(-0.2, 0.0) == "backward"


def test_turn_commands_follow_ros_angular_sign():
    assert action(0.0, 0.4) == "left"
    assert action(0.0, -0.4) == "right"


def test_mixed_command_turns_first_instead_of_ignoring_angular_velocity():
    assert action(0.2, 0.4) == "left"
    assert action(0.2, -0.4) == "right"


def test_twist_converts_to_independent_wheel_speeds():
    left, right = wheel_targets(0.12, 0.5, 0.24)
    assert left == pytest.approx(0.06)
    assert right == pytest.approx(0.18)


def test_wheel_limit_preserves_curvature():
    left, right = limit_wheel_targets(0.2, 0.4, 0.22)
    assert right == pytest.approx(0.22)
    assert left == pytest.approx(0.11)


def test_feedforward_pwm_preserves_direction_and_minimum_duty():
    assert feedforward_pwm(0.0, 0.22, 0.28) == 0.0
    assert feedforward_pwm(0.01, 0.22, 0.28) >= 0.28
    assert feedforward_pwm(-0.01, 0.22, 0.28) <= -0.28


def test_pi_controller_resets_integral_for_stop():
    pwm, integral = pi_pwm(
        0.0, 0.1, 0.3, 0.1, 0.22, 0.28, 0.8, 0.2, 0.4
    )
    assert pwm == 0.0
    assert integral == 0.0
