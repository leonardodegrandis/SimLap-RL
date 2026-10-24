"""Saturating slip-angle tires with lateral-priority combined grip allocation."""

import math
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class TireForce:
    lateral_n: float
    longitudinal_capacity_n: float
    slip_rad: float
    capacity_n: float


class TireModel(Protocol):
    def evaluate(self, longitudinal_mps: float, lateral_mps: float,
                 normal_n: float, stiffness_nprad: float) -> TireForce: ...


class SaturatingTire:
    def __init__(self, config):
        self.c = config

    def evaluate(self, longitudinal_mps, lateral_mps, normal_n, stiffness_nprad):
        slip = math.atan2(lateral_mps, max(abs(longitudinal_mps), self.c.slip_regularization_mps))
        capacity = self.c.friction_coefficient * max(0, normal_n)
        lateral = -capacity * math.tanh(stiffness_nprad * slip / max(capacity, 1e-9))
        available = math.sqrt(max(0, capacity**2 - lateral**2))
        return TireForce(lateral, available, slip, capacity)


@dataclass(frozen=True)
class ContactForce:
    longitudinal_n: float
    lateral_n: float
    slip_ratio: float
    slip_rad: float
    capacity_n: float


class CombinedSlipTire:
    """Smooth friction circle in wheel coordinates; no camber thrust or ABS."""

    def __init__(self, config):
        self.c = config

    def evaluate(self, vx, vy, rolling_speed, normal_n, longitudinal_stiffness, lateral_stiffness):
        kappa = (rolling_speed - vx) / max(abs(vx), abs(rolling_speed), self.c.slip_regularization_mps)
        alpha = math.atan2(vy, max(abs(vx), self.c.slip_regularization_mps))
        fx, fy = longitudinal_stiffness * kappa, -lateral_stiffness * alpha
        demand = math.hypot(fx, fy)
        capacity = self.c.friction_coefficient * max(0, normal_n)
        scale = capacity * math.tanh(demand / max(capacity, 1e-12)) / max(demand, 1e-12)
        return ContactForce(fx * scale, fy * scale, kappa, alpha, capacity)
