"""
Classical lane detection using HSV color thresholding.

Detects white lane markings by thresholding in HSV color space, then removes
orange barrel/pylon regions to avoid false positives. No ML model required.

Usage: this module is lazily imported by lane_detection_inference.py when
lane_detection_mode is set to 1 (classical).
"""
import cv2
import numpy as np


def _create_white_mask(img_hsv):
    """Create mask for white pixels (lane lines) in HSV."""
    sensitivity = 40
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])
    return cv2.inRange(img_hsv, lower_white, upper_white)


def _create_orange_mask(img_hsv):
    """Create mask for orange pixels (barrels/pylons) in HSV."""
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([50, 255, 255])
    return cv2.inRange(img_hsv, lower_orange, upper_orange)


def _get_mask(img, mask_function):
    """Downscale, apply mask function, upscale back."""
    h, w = img.shape[:2]
    small = cv2.resize(img, (w // 4, h // 4), interpolation=cv2.INTER_AREA)
    mask = mask_function(small)
    binary = np.where(mask > 0, 255, 0).astype(np.uint8)
    return cv2.resize(binary, (w, h), interpolation=cv2.INTER_AREA)


def _expand_barrels(img):
    """Expand barrel detections vertically by +/-30px to create exclusion zones."""
    if img.max() == 0:
        return img
    m_up = np.float64([[1, 0, 0], [0, 1, 30]])
    m_down = np.float64([[1, 0, 0], [0, 1, -30]])
    up = cv2.warpAffine(img, m_up, (img.shape[1], img.shape[0]))
    down = cv2.warpAffine(img, m_down, (img.shape[1], img.shape[0]))
    return img + up + down


def lane_detection(img):
    """
    Detect lane markings using HSV color thresholding.

    Args:
        img: BGR image (330x180)

    Returns:
        Binary mask (330x180) with 255 for lane pixels, 0 otherwise.
    """
    img = cv2.resize(img, (330, 180))
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    lanes = _get_mask(img_hsv, _create_white_mask)
    lanes = np.clip(lanes, 0, 1)

    barrels = _get_mask(img_hsv, _create_orange_mask)
    barrels = _expand_barrels(barrels)

    output = cv2.subtract(lanes, barrels)
    output = np.clip(output, 0, 1) * 255
    output[:50, :] = 0  # Remove sky false positives

    return output
