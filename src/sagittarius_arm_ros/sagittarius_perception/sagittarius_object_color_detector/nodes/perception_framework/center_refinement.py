#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


COLOR_ALIASES = {
    "red": ("red", "红", "紅"),
    "green": ("green", "绿", "綠"),
    "blue": ("blue", "蓝", "藍"),
}


def refine_detection_center(image_bgr, detection_box, target_text, mapper):
    """Refine a selected detection center with HSV contours when a known color exists.

    This keeps the semantic detector model-swappable: the backend picks the ROI, and
    a separate geometry refinement layer estimates a more precise center inside it.
    """

    if image_bgr is None or detection_box is None or mapper is None:
        return False

    color_name = _extract_color_name(target_text)
    if not color_name:
        return False

    hsv_range = mapper.get_hsv_range(color_name)
    if not hsv_range:
        return False

    image_height, image_width = image_bgr.shape[:2]
    x1, y1, x2, y2 = detection_box.bbox_xyxy
    x1 = int(round(max(0.0, min(float(image_width - 1), x1))))
    y1 = int(round(max(0.0, min(float(image_height - 1), y1))))
    x2 = int(round(max(0.0, min(float(image_width), x2))))
    y2 = int(round(max(0.0, min(float(image_height), y2))))
    if x2 <= x1 or y2 <= y1:
        return False

    roi = image_bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return False

    mask = _build_hsv_mask(roi, hsv_range)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    roi_area = float((x2 - x1) * (y2 - y1))
    if contour_area < max(25.0, roi_area * 0.003):
        return False

    moments = cv2.moments(contour)
    if moments["m00"] == 0.0:
        return False

    local_x = moments["m10"] / moments["m00"]
    local_y = moments["m01"] / moments["m00"]
    refined_center = (float(x1 + local_x), float(y1 + local_y))
    detection_box.metadata["refined_center"] = refined_center
    detection_box.metadata["refined_center_source"] = "hsv_contour"
    detection_box.metadata["refined_color"] = color_name
    detection_box.metadata["refined_contour_area"] = contour_area
    return True


def _extract_color_name(target_text):
    normalized = (target_text or "").strip().lower()
    if not normalized:
        return None
    for canonical, aliases in COLOR_ALIASES.items():
        for alias in aliases:
            if alias.lower() in normalized:
                return canonical
    return None


def _build_hsv_mask(image_bgr, hsv_range):
    image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hmin = float(hsv_range["hmin"]) / 2.0
    hmax = float(hsv_range["hmax"]) / 2.0
    smin = float(hsv_range["smin"])
    smax = float(hsv_range["smax"])
    vmin = float(hsv_range["vmin"])
    vmax = float(hsv_range["vmax"])

    if hmin > hmax:
        lower1 = np.array([0.0, smin, vmin], dtype=np.uint8)
        upper1 = np.array([hmax, smax, vmax], dtype=np.uint8)
        lower2 = np.array([hmin, smin, vmin], dtype=np.uint8)
        upper2 = np.array([180.0, smax, vmax], dtype=np.uint8)
        return cv2.add(
            cv2.inRange(image_hsv, lower1, upper1),
            cv2.inRange(image_hsv, lower2, upper2),
        )

    lower = np.array([hmin, smin, vmin], dtype=np.uint8)
    upper = np.array([hmax, smax, vmax], dtype=np.uint8)
    return cv2.inRange(image_hsv, lower, upper)
