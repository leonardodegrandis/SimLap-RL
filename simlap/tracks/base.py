"""Geometry contract for future spline, imported-coordinate, or banked tracks."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class TrackPoint:
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_pm: float


@dataclass(frozen=True)
class Projection:
    point: TrackPoint
    lateral_m: float  # Positive to the left of the centerline.


class Track(Protocol):
    length_m: float
    width_m: float

    def sample(self, s_m: float) -> TrackPoint: ...
    def project(self, x_m: float, y_m: float) -> Projection: ...


def signed_progress(previous_s, current_s, length):
    """Unwrap locally; call at every physics step, not once per lap."""
    return (current_s - previous_s + length / 2) % length - length / 2
