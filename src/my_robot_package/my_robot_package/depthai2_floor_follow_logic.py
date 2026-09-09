"""Pure control helpers for the bounded DepthAI 2 floor-follow test."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowLimits:
    """Motion limits for the deliberately short, low-speed floor test."""

    target_distance_m: float = 0.60
    distance_deadband_m: float = 0.00
    center_deadband: float = 0.10
    forward_center_limit: float = 0.15
    max_linear_mps: float = 0.05
    max_angular_rps: float = 0.15


def clamp(value, minimum, maximum):
    """Clamp a finite value to an inclusive range."""
    return max(minimum, min(maximum, value))


def bounded_follow_command(center_x, distance_m, limits, blocked=False):
    """Return a conservative ``(linear_x, angular_z)`` follow command.

    Positive image X is to the camera's right, while positive ROS yaw turns
    left, hence the negative steering sign.  Translation is forward-only and
    permitted only while the person remains near the image centre.
    """
    if blocked:
        return 0.0, 0.0
    if not math.isfinite(center_x) or not math.isfinite(distance_m):
        return 0.0, 0.0
    if not 0.0 <= center_x <= 1.0 or distance_m <= 0.0:
        return 0.0, 0.0

    error = center_x - 0.5
    absolute_error = abs(error)
    angular_z = 0.0
    if absolute_error > limits.center_deadband:
        angular_z = -math.copysign(
            limits.max_angular_rps
            * clamp(absolute_error / 0.35, 0.0, 1.0),
            error,
        )

    linear_x = 0.0
    far_threshold = limits.target_distance_m + limits.distance_deadband_m
    if distance_m > far_threshold and absolute_error < limits.forward_center_limit:
        distance_scale = clamp((distance_m - far_threshold) / 0.50, 0.35, 1.0)
        center_scale = clamp(
            1.0 - absolute_error / limits.forward_center_limit, 0.0, 1.0
        )
        linear_x = limits.max_linear_mps * distance_scale * center_scale

    return (
        clamp(linear_x, 0.0, limits.max_linear_mps),
        clamp(angular_z, -limits.max_angular_rps, limits.max_angular_rps),
    )


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def planar_distance(start_xy, current_xy):
    """Return Euclidean displacement between two planar poses."""
    return math.hypot(current_xy[0] - start_xy[0], current_xy[1] - start_xy[1])


def current_throttle_bits(value):
    """Return only current Raspberry Pi throttle/power flags (low nibble)."""
    return int(value) & 0xF
