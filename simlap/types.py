"""Small, stable boundary objects for controllers and replaceable physics modules."""

import math
from dataclasses import astuple, dataclass, fields
from typing import Dict, Optional, Protocol


@dataclass(frozen=True)
class Control:
    throttle: float = 0.0
    brake: float = 0.0
    steer_rad: float = 0.0
    yaw_rate_radps: Optional[float] = None

    def __post_init__(self):
        values = (self.throttle, self.brake, self.steer_rad)
        if self.yaw_rate_radps is not None:
            values += (self.yaw_rate_radps,)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Controls must be finite")
        if not (0 <= self.throttle <= 1 and 0 <= self.brake <= 1):
            raise ValueError("Throttle and brake must be in [0, 1]")
        if self.yaw_rate_radps is not None and self.steer_rad != 0:
            raise ValueError("Specify steering OR yaw rate")


@dataclass(frozen=True)
class State:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    steer_rad: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    soc: float = 0.9
    chemical_energy_j: float = 0.0
    terminal_energy_j: float = 0.0
    battery_heat_j: float = 0.0
    motor_heat_j: float = 0.0
    inverter_heat_j: float = 0.0
    gearbox_heat_j: float = 0.0
    friction_brake_heat_j: float = 0.0
    distance_m: float = 0.0

    def shifted(self, derivative, dt):
        return type(self)(*(x + dt * dx for x, dx in zip(astuple(self), astuple(derivative))))


STATE_NAMES = tuple(f.name for f in fields(State))


@dataclass(frozen=True)
class Evaluation:
    derivative: State
    signals: Dict[str, float]


class DynamicsModel(Protocol):
    def evaluate(self, state: State, control: Control) -> Evaluation: ...


class Controller(Protocol):
    def __call__(self, time_s: float, state: State) -> Control: ...
