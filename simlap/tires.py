"""Compatibility imports; implementations live in simlap.physics."""
from .physics.tires import SaturatingTire, TireForce, TireModel

__all__ = ["SaturatingTire", "TireForce", "TireModel"]
