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
