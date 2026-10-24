"""Rint pack: positive current/power discharges; negative values charge."""

import math
from dataclasses import dataclass
from typing import Protocol

from ..config import Battery
from ..mathutils import clip, interp


@dataclass(frozen=True)
class PackFlow:
    current_a: float
    terminal_voltage_v: float
    terminal_power_w: float
    chemical_power_w: float
    heat_w: float
    soc_rate: float
    ocv_v: float


class BatteryModel(Protocol):
    def power_limits(self, soc: float): ...
    def flow(self, power_w: float, soc: float) -> PackFlow: ...


class ResistiveBattery:
    def __init__(self, config: Battery):
        self.c = config
        self.capacity_coulomb = config.capacity_kwh * 3.6e6 / config.nominal_voltage_v

    def ocv(self, soc):
        return interp(soc, self.c.ocv_soc, self.c.ocv_voltage_v)

    def power_limits(self, soc):
        """Return maximum discharge W and maximum charge magnitude W."""
        c, voltage = self.c, self.ocv(soc)
        discharge, charge = c.max_discharge_a, c.max_charge_a
        if c.resistance_ohm:
            discharge = min(discharge, voltage / (2 * c.resistance_ohm),
                            max(0, (voltage - c.min_voltage_v) / c.resistance_ohm))
            charge = min(charge, max(0, (c.max_voltage_v - voltage) / c.resistance_ohm))
        else:
            discharge *= voltage >= c.min_voltage_v
            charge *= voltage <= c.max_voltage_v
        discharge *= soc > c.min_soc
        charge *= soc < c.max_soc
        return ((voltage - discharge * c.resistance_ohm) * discharge,
                (voltage + charge * c.resistance_ohm) * charge)

    def flow(self, power_w, soc):
        voltage, resistance = self.ocv(soc), self.c.resistance_ohm
        discharge, charge = self.power_limits(soc)
        power = clip(power_w, -charge, discharge)
        # Rationalized quadratic root avoids cancellation near zero power.
        current = 2 * power / (voltage + math.sqrt(max(0, voltage**2 - 4 * resistance * power)))
        return PackFlow(current, voltage - current * resistance, power,
                        voltage * current, current**2 * resistance,
                        -current / self.capacity_coulomb, voltage)
