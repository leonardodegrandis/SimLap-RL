"""Mapped shaft power, separate losses, battery-limited drive and blended regen."""

import math
from typing import Protocol

from .battery import BatteryModel, ResistiveBattery
from ..mathutils import clip, interp


class PowertrainModel(Protocol):
    def prepare(self, speed_mps: float, soc: float, throttle: float, brake: float): ...
    def deliver(self, prepared, rear_capacity_n: float): ...


class ElectricPowertrain:
    def __init__(self, config, battery: BatteryModel = None):
        self.c = config
        self.battery = battery or ResistiveBattery(config.battery)

    def torque_envelope(self, rpm):
        c = self.c.motor
        scale = c.nominal_factor if c.rating == "nominal" else 1.0
        if rpm >= c.map_rpm[-1]:
            return 0.0
        if rpm <= c.map_rpm[1]:
            torque = c.peak_power_kw * 1000 * c.map_power_fraction[1] / (c.map_rpm[1] * math.pi / 30)
        else:
            power = c.peak_power_kw * 1000 * interp(rpm, c.map_rpm, c.map_power_fraction)
            torque = power / (rpm * math.pi / 30)
        return scale * min(c.max_torque_nm, torque)

    def electrical(self, torque, omega):
        """Return DC bus W, shaft W, motor heat W, inverter heat W."""
        if abs(torque) < 1e-12:
            return (0.0,) * 4  # Ideal disconnect when not producing torque.
        m, inv = self.c.motor, self.c.inverter
        shaft = torque * omega
        heat = m.idle_loss_w + m.copper_loss_w_per_nm2 * torque**2 + m.iron_loss_w_per_radps2 * omega**2
        ac = shaft + heat
        dc = (ac / inv.efficiency if ac >= 0 else ac * inv.efficiency) + inv.idle_loss_w
        return dc, shaft, heat, dc - ac

    def prepare(self, speed_mps, soc, throttle, brake):
        c, m = self.c, self.c.motor
        speed = max(0.0, speed_mps)
        omega = speed / c.vehicle.wheel_radius_m * m.gear_ratio
        discharge, charge = self.battery.power_limits(soc)
        auxiliary = min(c.battery.auxiliary_power_w, discharge)
        drive = self.torque_envelope(omega * 30 / math.pi) * throttle if brake < 1e-6 else 0.0
        rear_brake = c.actuators.max_brake_force_n * brake * (1 - c.actuators.front_brake_fraction)
        regen = min(m.regen_max_torque_nm, m.regen_max_power_kw * 1000 / max(omega, 1e-9),
                    rear_brake * m.gear_efficiency * c.vehicle.wheel_radius_m / m.gear_ratio)
        regen *= min(1.0, speed / m.regen_full_speed_mps) * c.actuators.regen_enabled
        if omega * 30 / math.pi >= m.map_rpm[-1]:
            regen = 0.0
        if m.copper_loss_w_per_nm2:
            regen = min(regen, omega / (2 * m.copper_loss_w_per_nm2))

        def limit(torque, bound, charging=False):
            def allowed(value):
                power = self.electrical(value, omega)[0] + auxiliary
                return power >= -bound if charging else power <= bound
            if allowed(torque):
                return torque
            lo, hi = 0.0, 1.0
            for _ in range(32):
                mid = (lo + hi) / 2
                if allowed(torque * mid):
                    lo = mid
                else:
                    hi = mid
            return torque * lo

        drive_limited = limit(drive, discharge)
        regen_limited = -limit(-regen, charge, charging=True)
        if self.electrical(-regen_limited, omega)[0] >= 0:
            regen_limited = 0.0
        return dict(speed=speed, omega=omega, soc=soc, auxiliary=auxiliary,
                    drive=drive_limited, regen=regen_limited, rear_brake=rear_brake,
                    drive_requested=drive, regen_requested=regen,
                    discharge_limit=discharge, charge_limit=charge)

    def deliver(self, prepared, rear_capacity_n):
        p, m, v = prepared, self.c.motor, self.c.vehicle
        drive_conversion = m.gear_ratio * m.gear_efficiency / v.wheel_radius_m
        regen_conversion = m.gear_ratio / (m.gear_efficiency * v.wheel_radius_m)
        drive = min(p["drive"], rear_capacity_n / drive_conversion)
        brake_force = min(p["rear_brake"], rear_capacity_n)
        regen = min(p["regen"], brake_force / regen_conversion)
        if self.electrical(-regen, p["omega"])[0] >= 0:
            regen = 0.0
        torque = drive - regen
        drive_force, regen_force = drive * drive_conversion, regen * regen_conversion
        friction_force = brake_force - regen_force
        dc, shaft, motor_heat, inverter_heat = self.electrical(torque, p["omega"])
        pack = self.battery.flow(dc + p["auxiliary"], p["soc"])
        wheel_power = (drive_force - regen_force) * p["speed"]
        gearbox_heat = shaft - wheel_power
        signals = dict(
            motor_rpm=p["omega"] * 30 / math.pi, motor_torque_nm=torque,
            requested_drive_torque_nm=p["drive_requested"], available_drive_torque_nm=p["drive"],
            motor_shaft_power_w=shaft, wheel_drive_power_w=wheel_power,
            motor_dc_power_w=dc, motor_loss_w=motor_heat, inverter_loss_w=inverter_heat,
            gearbox_loss_w=gearbox_heat, auxiliary_power_w=p["auxiliary"],
            auxiliary_unserved_w=self.c.battery.auxiliary_power_w - p["auxiliary"],
            battery_current_a=pack.current_a, battery_voltage_v=pack.terminal_voltage_v,
            battery_ocv_v=pack.ocv_v, battery_terminal_power_w=pack.terminal_power_w,
            battery_chemical_power_w=pack.chemical_power_w, battery_loss_w=pack.heat_w,
            battery_discharge_limit_w=p["discharge_limit"], battery_charge_limit_w=p["charge_limit"],
            drive_force_n=drive_force, regen_force_n=regen_force,
            rear_friction_brake_force_n=friction_force,
            power_balance_error_w=pack.chemical_power_w - (wheel_power + motor_heat + inverter_heat + gearbox_heat + p["auxiliary"] + pack.heat_w),
        )
        return drive_force - brake_force, pack, signals
