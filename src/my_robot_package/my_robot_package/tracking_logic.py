"""Pure tracking control logic that can be tested without ROS or robot hardware."""

import math


def compute_motion(
    mode,
    target_visible,
    center_x,
    distance_m,
    target_distance_m,
    distance_deadband_m,
    center_deadband,
    follow_speed,
    turn_speed,
    blocked=False,
):
    """Return ``(linear_x, angular_z)`` for the current tracking observation."""
    if blocked or not target_visible or mode == "IDLE":
        return 0.0, 0.0

    horizontal_error = center_x - 0.5
    absolute_error = abs(horizontal_error)
    angular_z = 0.0
    if absolute_error > center_deadband:
        # Proportional visual steering: gentle near the centre, full turn at an edge.
        angular_z = -math.copysign(
            turn_speed * min(absolute_error / 0.5, 1.0), horizontal_error
        )

    linear_x = 0.0
    if mode == "FOLLOW" and distance_m > target_distance_m + distance_deadband_m:
        # Blend forward and turn only while the target remains safely in-frame.
        # Near an image edge, rotate first to avoid driving blindly sideways.
        if absolute_error < 0.35:
            steering_scale = max(0.0, 1.0 - absolute_error / 0.35)
            linear_x = follow_speed * steering_scale

    return linear_x, angular_z
