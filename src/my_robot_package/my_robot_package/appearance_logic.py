"""Lightweight, non-biometric appearance signatures for target re-identification."""

import cv2
import numpy as np


HISTOGRAM_BINS = (24, 16)


def roi_bounds(roi, frame_shape):
    """Return a clipped ``(x1, y1, x2, y2)`` box for a DepthAI ROI."""
    height, width = frame_shape[:2]
    try:
        left = float(roi.x)
        top = float(roi.y)
        right = left + float(roi.width)
        bottom = top + float(roi.height)
    except AttributeError:
        top_left = roi.topLeft()
        bottom_right = roi.bottomRight()
        left, top = float(top_left.x), float(top_left.y)
        right, bottom = float(bottom_right.x), float(bottom_right.y)

    if max(abs(left), abs(top), abs(right), abs(bottom)) <= 1.5:
        left, right = left * width, right * width
        top, bottom = top * height, bottom * height

    x1 = max(0, min(width - 1, int(round(left))))
    y1 = max(0, min(height - 1, int(round(top))))
    x2 = max(x1 + 1, min(width, int(round(right))))
    y2 = max(y1 + 1, min(height, int(round(bottom))))
    return x1, y1, x2, y2


def appearance_signature(frame, bounds):
    """Create a normalized HSV hue/saturation histogram for a person's crop."""
    x1, y1, x2, y2 = bounds
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0 or crop.shape[0] < 8 or crop.shape[1] < 8:
        return None

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist(
        [hsv], [0, 1], None, list(HISTOGRAM_BINS), [0, 180, 0, 256]
    )
    cv2.normalize(histogram, histogram, alpha=1.0, norm_type=cv2.NORM_L1)
    return histogram.astype(np.float32).reshape(-1)


def appearance_distance(first, second):
    """Return Bhattacharyya distance (0 identical, 1 very different)."""
    if first is None or second is None:
        return float("inf")
    first_array = np.asarray(first, dtype=np.float32).reshape(-1, 1)
    second_array = np.asarray(second, dtype=np.float32).reshape(-1, 1)
    if first_array.shape != second_array.shape:
        return float("inf")
    return float(
        cv2.compareHist(first_array, second_array, cv2.HISTCMP_BHATTACHARYYA)
    )
