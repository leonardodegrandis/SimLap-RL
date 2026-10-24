"""Explicit model selection; existing planar configurations retain their behavior."""

from .chassis import PlanarMotorcycle


def make_model(config):
    if config.dynamics.model == "planar":
        return PlanarMotorcycle(config)
    if config.dynamics.model == "roll_pitch":
        from .roll_pitch import RollPitchMotorcycle
        return RollPitchMotorcycle(config)
    raise ValueError(f"Unknown dynamics model: {config.dynamics.model}")
