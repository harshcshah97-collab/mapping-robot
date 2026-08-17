import numpy as np

from my_robot_package.appearance_logic import (
    appearance_distance,
    appearance_signature,
)


def solid_bgr(color):
    frame = np.zeros((80, 60, 3), dtype=np.uint8)
    frame[:, :] = color
    return frame


def test_same_appearance_has_near_zero_distance():
    frame = solid_bgr((0, 0, 255))
    first = appearance_signature(frame, (0, 0, 60, 80))
    second = appearance_signature(frame.copy(), (0, 0, 60, 80))
    assert appearance_distance(first, second) < 0.01


def test_different_hues_have_high_distance():
    red = appearance_signature(solid_bgr((0, 0, 255)), (0, 0, 60, 80))
    green = appearance_signature(solid_bgr((0, 255, 0)), (0, 0, 60, 80))
    assert appearance_distance(red, green) > 0.8


def test_too_small_crop_is_rejected():
    assert appearance_signature(solid_bgr((255, 0, 0)), (0, 0, 4, 4)) is None
