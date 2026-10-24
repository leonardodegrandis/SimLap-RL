"""Exact two-straight, two-semicircle geometry and constant-time projection."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

from ..config import ROOT
from ..mathutils import clip
from .base import Projection, TrackPoint


@dataclass(frozen=True)
class StadiumTrack:
    straight_length_m: float = 120.0
    corner_radius_m: float = 30.0
    width_m: float = 8.0
    name: str = "oval"

    def __post_init__(self):
        values = (self.straight_length_m, self.corner_radius_m, self.width_m)
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Track dimensions must be finite and positive")
        if self.width_m >= 2 * self.corner_radius_m:
            raise ValueError("Track width must be less than the corner diameter")

    @property
    def length_m(self):
        return 2 * self.straight_length_m + 2 * math.pi * self.corner_radius_m

    def sample(self, s_m):
        length, radius = self.straight_length_m, self.corner_radius_m
        s = s_m % self.length_m
        if s < length:
            return TrackPoint(s, s, 0.0, 0.0, 0.0)
        if s < length + math.pi * radius:
            angle = (s - length) / radius - math.pi / 2
            return TrackPoint(s, length + radius * math.cos(angle), radius + radius * math.sin(angle), angle + math.pi / 2, 1 / radius)
        if s < 2 * length + math.pi * radius:
            return TrackPoint(s, 2 * length + math.pi * radius - s, 2 * radius, math.pi, 0.0)
        angle = (s - 2 * length - math.pi * radius) / radius + math.pi / 2
        return TrackPoint(s, radius * math.cos(angle), radius + radius * math.sin(angle), angle + math.pi / 2, 1 / radius)

    def project(self, x_m, y_m):
        length, radius = self.straight_length_m, self.corner_radius_m
        right_angle = clip(math.atan2(y_m - radius, x_m - length), -math.pi / 2, math.pi / 2)
        left_angle = clip(math.atan2(y_m - radius, x_m) % (2 * math.pi), math.pi / 2, 3 * math.pi / 2)
        candidates = [clip(x_m, 0, length), length + (right_angle + math.pi / 2) * radius,
                      length + math.pi * radius + clip(length - x_m, 0, length),
                      2 * length + math.pi * radius + (left_angle - math.pi / 2) * radius]
        point = min((self.sample(s) for s in candidates), key=lambda p: (p.x_m - x_m)**2 + (p.y_m - y_m)**2)
        lateral = -(x_m - point.x_m) * math.sin(point.heading_rad) + (y_m - point.y_m) * math.cos(point.heading_rad)
        return Projection(point, lateral)

    def to_dict(self):
        return dict(type="stadium", name=self.name, straight_length_m=self.straight_length_m,
                    corner_radius_m=self.corner_radius_m, width_m=self.width_m)


def load_track(path=None):
    raw = json.loads(Path(path or ROOT / "tracks/oval.json").read_text())
    if raw.pop("type", None) != "stadium":
        raise ValueError("Supported track type: stadium")
    try:
        return StadiumTrack(**raw)
    except TypeError as error:
        raise ValueError(f"Invalid track configuration: {error}") from error
