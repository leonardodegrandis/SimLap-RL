"""Strict, unit-explicit configuration; JSON overlays change only selected values."""

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import get_type_hints

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class Vehicle:
    mass_kg: float
    yaw_inertia_kgm2: float
    front_distance_m: float
    rear_distance_m: float
    cg_height_m: float
    wheel_radius_m: float
    drag_coefficient: float
    frontal_area_m2: float
    rolling_resistance: float
    max_lean_deg: float


@dataclass(frozen=True)
class Environment:
    gravity_mps2: float
    air_density_kgpm3: float
    headwind_mps: float


@dataclass(frozen=True)
class Tires:
    friction_coefficient: float
    front_stiffness_nprad: float
    rear_stiffness_nprad: float
    slip_regularization_mps: float


@dataclass(frozen=True)
class Motor:
    peak_power_kw: float
    nominal_factor: float
    rating: str
    map_rpm: list
    map_power_fraction: list
    max_torque_nm: float
    gear_ratio: float
    gear_efficiency: float
    copper_loss_w_per_nm2: float
    iron_loss_w_per_radps2: float
    idle_loss_w: float
    regen_max_torque_nm: float
    regen_max_power_kw: float
    regen_full_speed_mps: float


@dataclass(frozen=True)
class Inverter:
    efficiency: float
    idle_loss_w: float


@dataclass(frozen=True)
class Battery:
    capacity_kwh: float
    nominal_voltage_v: float
    resistance_ohm: float
    ocv_soc: list
    ocv_voltage_v: list
    max_discharge_a: float
    max_charge_a: float
    min_voltage_v: float
    max_voltage_v: float
    min_soc: float
    max_soc: float
    initial_soc: float
    auxiliary_power_w: float


@dataclass(frozen=True)
class Actuators:
    max_steer_deg: float
    steer_rate_degps: float
    steer_time_constant_s: float
    throttle_time_constant_s: float
    brake_time_constant_s: float
    brake_regularization_mps: float
    max_brake_force_n: float
    front_brake_fraction: float
    regen_enabled: bool
    yaw_rate_gain_s: float
    yaw_rate_max_radps: float
    yaw_rate_min_speed_mps: float


@dataclass(frozen=True)
class Simulation:
    dt_s: float
    control_dt_s: float
    load_transfer_iterations: int
    load_transfer_tolerance_n: float
    stop_speed_mps: float
    log_level: str
    dashboard_max_points: int


@dataclass(frozen=True)
class Config:
    vehicle: Vehicle
    environment: Environment
    tires: Tires
    motor: Motor
    inverter: Inverter
    battery: Battery
    actuators: Actuators
    simulation: Simulation

    def to_dict(self):
        return asdict(self)


def merge(base, overlay):
    for key, value in overlay.items():
        if key not in base:
            raise ValueError(f"Unknown configuration key: {key}")
        if isinstance(base[key], dict) and isinstance(value, dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path=None, overlays=()):
    with open(path or ROOT / "configs/motorcycle.json") as stream:
        raw = json.load(stream)
    for overlay in overlays:
        with open(overlay) as stream:
            merge(raw, json.load(stream))
    types = get_type_hints(Config)
    if set(raw) != set(types):
        raise ValueError("Config must contain exactly: " + ", ".join(types))
    try:
        config = Config(**{key: cls(**raw[key]) for key, cls in types.items()})
    except TypeError as error:
        raise ValueError(f"Invalid configuration: {error}") from error
    validate(config)
    return config


def validate(c):
    def require(condition, message):
        if not condition:
            raise ValueError(message)

    # Parameters are positive unless explicitly allowed to be zero or signed.
    zero_ok = {"cg_height_m", "drag_coefficient", "rolling_resistance", "resistance_ohm",
               "copper_loss_w_per_nm2", "iron_loss_w_per_radps2", "idle_loss_w",
               "regen_max_torque_nm", "regen_max_power_kw", "max_charge_a",
               "min_soc", "initial_soc", "auxiliary_power_w", "yaw_rate_gain_s",
               "front_brake_fraction"}
    for section in fields(c):
        obj = getattr(c, section.name)
        for field in fields(obj):
            value = getattr(obj, field.name)
            kind = get_type_hints(type(obj))[field.name]
            if kind in (float, int):
                require(type(value) in (int, float) and math.isfinite(value), f"{field.name}: expected finite number")
                if kind is int:
                    require(type(value) is int, f"{field.name}: expected integer")
                if field.name != "headwind_mps":
                    require(value >= 0 if field.name in zero_ok else value > 0, f"{field.name}: invalid range")
            elif kind is bool:
                require(type(value) is bool, f"{field.name}: expected boolean")
    m, b, a, s = c.motor, c.battery, c.actuators, c.simulation
    require(m.rating in ("peak", "nominal"), "motor.rating must be peak or nominal")
    for value in (m.nominal_factor, m.gear_efficiency, c.inverter.efficiency):
        require(0 < value <= 1, "Scaling and efficiencies must be in (0, 1]")
    require(0 <= a.front_brake_fraction <= 1, "front_brake_fraction must be in [0, 1]")
    require(0 <= b.min_soc < b.max_soc <= 1 and b.min_soc <= b.initial_soc <= b.max_soc, "Invalid SOC bounds")
    require(b.min_voltage_v < b.max_voltage_v, "Invalid voltage bounds")
    require(0 < c.vehicle.max_lean_deg < 90 and 0 < a.max_steer_deg < 80, "Invalid lean/steer angle")
    for xs, ys, name in ((m.map_rpm, m.map_power_fraction, "motor map"), (b.ocv_soc, b.ocv_voltage_v, "OCV map")):
        require(isinstance(xs, list) and isinstance(ys, list) and len(xs) == len(ys) >= 2, f"Invalid {name} dimensions")
        require(all(type(v) in (int, float) and math.isfinite(v) for v in xs + ys), f"Non-finite {name}")
        require(all(y > x for x, y in zip(xs, xs[1:])), f"{name} axis must increase")
    require(m.map_rpm[0] == 0 and m.map_power_fraction[0] == 0 and m.map_power_fraction[-1] == 0,
            "Power map must start at zero rpm/power and end at zero power (speed cutoff)")
    require(all(0 <= x <= 1 for x in m.map_power_fraction) and max(m.map_power_fraction) == 1, "Power map fractions must span 0..1")
    require(b.ocv_soc[0] == 0 and b.ocv_soc[-1] == 1 and all(v > 0 for v in b.ocv_voltage_v), "OCV map must cover SOC 0..1 with positive voltages")
    require(all(y >= x for x, y in zip(b.ocv_voltage_v, b.ocv_voltage_v[1:])), "OCV must not decrease with SOC")
    require(s.dt_s <= min(a.steer_time_constant_s, a.throttle_time_constant_s, a.brake_time_constant_s) / 4, "dt_s too large for actuator dynamics")
    require(s.control_dt_s >= s.dt_s and abs(s.control_dt_s / s.dt_s - round(s.control_dt_s / s.dt_s)) < 1e-8, "control_dt_s must be an integer multiple of dt_s")
    require(s.dashboard_max_points >= 2, "dashboard_max_points must be >= 2")
    require(s.log_level in ("DEBUG", "INFO", "WARNING", "ERROR"), "Invalid log_level")
