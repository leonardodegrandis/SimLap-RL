"""Compatibility imports for the electrical component interfaces."""
from .physics.battery import BatteryModel, PackFlow, ResistiveBattery

__all__ = ["BatteryModel", "PackFlow", "ResistiveBattery"]
