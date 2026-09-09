"""Pure differential-drive and PWM helpers, independent of ROS and GPIO."""


def clamp(value, minimum, maximum):
    """Clamp *value* to an inclusive numeric range."""
    return max(minimum, min(maximum, value))


def wheel_targets(linear_x, angular_z, wheel_separation_m):
    """Convert a base twist into left and right wheel speeds in metres/second."""
    half_track = wheel_separation_m / 2.0
    return (
        linear_x - angular_z * half_track,
        linear_x + angular_z * half_track,
    )


def wheel_speeds(linear_x, angular_z, wheel_separation_m):
    """Convert measured base velocity into left and right wheel speeds."""
    return wheel_targets(linear_x, angular_z, wheel_separation_m)


def limit_wheel_targets(left_mps, right_mps, max_wheel_speed_mps):
    """Scale both targets together so curvature is preserved at speed limits."""
    if max_wheel_speed_mps <= 0.0:
        raise ValueError("max_wheel_speed_mps must be positive")
    peak = max(abs(left_mps), abs(right_mps))
    if peak <= max_wheel_speed_mps:
        return left_mps, right_mps
    scale = max_wheel_speed_mps / peak
    return left_mps * scale, right_mps * scale


def feedforward_pwm(target_mps, max_wheel_speed_mps, minimum_pwm):
    """Return signed open-loop PWM for a requested wheel speed."""
    if max_wheel_speed_mps <= 0.0:
        raise ValueError("max_wheel_speed_mps must be positive")
    if target_mps == 0.0:
        return 0.0

    normalized = clamp(abs(target_mps) / max_wheel_speed_mps, 0.0, 1.0)
    duty_cycle = minimum_pwm + (1.0 - minimum_pwm) * normalized
    return clamp(duty_cycle, 0.0, 1.0) * (1.0 if target_mps > 0.0 else -1.0)


def pi_pwm(
    target_mps,
    measured_mps,
    integral_error,
    dt,
    max_wheel_speed_mps,
    minimum_pwm,
    kp,
    ki,
    integral_limit,
):
    """Return ``(signed_pwm, new_integral)`` for one wheel PI controller."""
    if target_mps == 0.0:
        return 0.0, 0.0

    error = target_mps - measured_mps
    new_integral = clamp(
        integral_error + error * max(dt, 0.0),
        -abs(integral_limit),
        abs(integral_limit),
    )
    pwm = feedforward_pwm(target_mps, max_wheel_speed_mps, minimum_pwm)
    pwm += kp * error + ki * new_integral
    return clamp(pwm, -1.0, 1.0), new_integral


def select_drive_action(linear_x, angular_z, linear_deadband, turn_threshold):
    """Legacy binary-drive selector retained for old tests and diagnostics."""
    if abs(linear_x) < linear_deadband and abs(angular_z) < turn_threshold:
        return "stop"
    if abs(angular_z) >= turn_threshold:
        return "left" if angular_z > 0.0 else "right"
    if linear_x > 0.0:
        return "forward"
    if linear_x < 0.0:
        return "backward"
    return "stop"
