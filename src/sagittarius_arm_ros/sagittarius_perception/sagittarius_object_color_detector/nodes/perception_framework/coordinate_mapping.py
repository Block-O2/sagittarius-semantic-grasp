#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml


class VisionPlaneMapper:
    """Maps image pixel centers to the calibrated Sagittarius grasp plane."""

    def __init__(self, vision_config_path: str):
        self.params, self.color_params = self._load_vision_config(vision_config_path)

    def map_pixel_center(self, center):
        pixel_x, pixel_y = center
        grasp_x = self.params["k1"] * pixel_y + self.params["b1"]
        grasp_y = self.params["k2"] * pixel_x + self.params["b2"]
        return grasp_x, grasp_y

    def is_degenerate(self, epsilon=1e-9):
        """Return True when pixel changes cannot affect mapped grasp XY."""
        return abs(self.params["k1"]) < epsilon and abs(self.params["k2"]) < epsilon

    def describe(self):
        return "x=k1*pixel_y+b1 (k1={k1:.6f}, b1={b1:.6f}), y=k2*pixel_x+b2 (k2={k2:.6f}, b2={b2:.6f})".format(
            **self.params
        )

    def get_hsv_range(self, color_name):
        color = self.color_params.get((color_name or "").strip().lower())
        if not color:
            return None
        return {
            "hmin": float(color["hmin"]),
            "hmax": float(color["hmax"]),
            "smin": float(color["smin"]),
            "smax": float(color["smax"]),
            "vmin": float(color["vmin"]),
            "vmax": float(color["vmax"]),
        }

    def _load_vision_config(self, filename):
        with open(filename, "r") as stream:
            content = yaml.safe_load(stream)
        linear_params = {
            "k1": float(content["LinearRegression"]["k1"]),
            "b1": float(content["LinearRegression"]["b1"]),
            "k2": float(content["LinearRegression"]["k2"]),
            "b2": float(content["LinearRegression"]["b2"]),
        }
        color_params = {}
        for color_name in ("red", "green", "blue"):
            if color_name not in content:
                continue
            hsv = content[color_name]
            if not all(
                key in hsv for key in ("hmin", "hmax", "smin", "smax", "vmin", "vmax")
            ):
                continue
            color_params[color_name] = hsv
        return linear_params, color_params
