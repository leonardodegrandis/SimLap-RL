"""Reproducible open-loop maneuvers; cornering begins from a solved trim."""

import json
import math

from .config import ROOT
from .mathutils import solve_linear
from .types import Control, State


def load_maneuvers(path=None):
    with open(path or ROOT / "configs/maneuvers.json") as stream:
        data = json.load(stream)
    required = {
        "acceleration": {"duration_s", "initial_speed_mps", "throttle"},
        "braking": {"duration_s", "initial_speed_mps", "brake"},
        "cornering": {"duration_s", "speed_mps", "radius_m", "trim_tolerance", "trim_iterations"},
    }
    if set(data) != set(required):
        raise ValueError("Maneuver config must define acceleration, braking and cornering")
    for name, keys in required.items():
        optional = {"roll_kp", "roll_kd"} if name == "cornering" else set()
        if not keys <= set(data[name]) or set(data[name]) - keys - optional:
            raise ValueError(f"Invalid keys for {name}: expected {sorted(keys)}")
        for key, value in data[name].items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{name}.{key} must be finite")
            if key in ("throttle", "brake"):
                valid = 0 <= value <= 1
            elif key in ("initial_speed_mps", "roll_kp", "roll_kd"):
                valid = value >= 0
            elif key == "radius_m":
                valid = value != 0
            else:
                valid = value > 0
            if not valid:
                raise ValueError(f"Invalid {name}.{key}")
    if type(data["cornering"]["trim_iterations"]) is not int:
        raise ValueError("trim_iterations must be an integer")
    return data


def cornering_trim(model, config, params):
    """Solve body-speed, lateral and yaw equilibrium at a requested CG path radius.

    Speed means total ground speed. Heading starts tangent to the circle after
    compensating sideslip. Controls remain constant during the subsequent run.
    """
    if hasattr(model, "cornering_trim"):
        return model.cornering_trim(params)
    speed, radius = params["speed_mps"], params["radius_m"]
    v, env = config.vehicle, config.environment
    lateral_accel = speed**2 / abs(radius)
    if lateral_accel >= min(config.tires.friction_coefficient * env.gravity_mps2,
                            env.gravity_mps2 * math.tan(math.radians(v.max_lean_deg))):
        raise ValueError("Corner requires more grip or lean than configured")
    yaw = speed / radius
    wheelbase = v.front_distance_m + v.rear_distance_m

    def evaluate(variables):
        vy, steer, throttle = variables
        if abs(vy) >= speed or abs(steer) >= math.radians(config.actuators.max_steer_deg) or not 0 <= throttle <= 1:
            raise ValueError("Corner trim exceeds speed, steering or power limits")
        vx = math.sqrt(speed**2 - vy**2)
        state = State(vx_mps=vx, vy_mps=vy, yaw_rate_radps=yaw, steer_rad=steer,
                      throttle=throttle, soc=config.battery.initial_soc,
                      heading_rad=-math.atan2(vy, vx))
        control = Control(throttle=throttle, steer_rad=steer)
        result = model.evaluate(state, control)
        d = result.derivative
        return [d.vx_mps, d.vy_mps, d.yaw_rate_radps], state, control, result.signals

    values = [0.0, math.atan(wheelbase / radius), 0.1]
    for iteration in range(params["trim_iterations"]):
        residual, state, control, signals = evaluate(values)
        norm = max(abs(x) for x in residual)
        if norm < params["trim_tolerance"]:
            return state, control, dict(trim_residual=norm, trim_iterations=iteration + 1,
                                       target_radius_m=radius, target_speed_mps=speed)
        columns = []
        for index, delta in enumerate((1e-5, 1e-6, 1e-6)):
            perturbed = values.copy()
            perturbed[index] += delta
            result = evaluate(perturbed)[0]
            columns.append([(b - a) / delta for a, b in zip(residual, result)])
        change = solve_linear(list(zip(*columns)), [-x for x in residual])
        for exponent in range(16):
            candidate = [x + dx * 0.5**exponent for x, dx in zip(values, change)]
            try:
                improved = max(abs(x) for x in evaluate(candidate)[0]) < norm
            except ValueError:
                improved = False
            if improved:
                values = candidate
                break
        else:
            raise ValueError("Corner trim failed: request a lower speed or larger radius")
    raise ValueError("Corner trim did not converge within trim_iterations")


def prepare(name, model, config, params):
    if name == "cornering":
        return cornering_trim(model, config, params)
    state = State(vx_mps=params["initial_speed_mps"], soc=config.battery.initial_soc)
    control = Control(throttle=params["throttle"]) if name == "acceleration" else Control(brake=params["brake"])
    return state, control, {}
