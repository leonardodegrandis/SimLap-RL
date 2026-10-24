"""Steady corner equilibrium for the reduced roll/pitch chassis."""

import math

from ..mathutils import solve_linear
from ..types import Control, RollPitchState


def roll_pitch_trim(model, config, params):
    v, d = config.vehicle, config.dynamics
    speed, radius = params["speed_mps"], params["radius_m"]
    yaw, gravity = speed / radius, config.environment.gravity_mps2
    if speed**2 / abs(radius) >= min(config.tires.friction_coefficient * gravity,
                                    gravity * math.tan(math.radians(v.max_lean_deg))):
        raise ValueError("Corner requires more grip or lean than configured")

    def evaluate(values):
        vy, steer, throttle, roll, pitch, heave, front_omega, rear_omega = values
        if (abs(vy) >= speed or abs(steer) >= math.radians(config.actuators.max_steer_deg)
                or not 0 <= throttle <= 1 or min(front_omega, rear_omega) < 0):
            raise ValueError("Trim exceeds speed, steering or power limits")
        vx = math.sqrt(speed**2 - vy**2)
        state = RollPitchState(vx_mps=vx, vy_mps=vy, yaw_rate_radps=yaw, steer_rad=steer, throttle=throttle,
                              soc=config.battery.initial_soc, heading_rad=-math.atan2(vy, vx), roll_rad=roll,
                              pitch_rad=pitch, heave_m=heave, front_wheel_radps=front_omega, rear_wheel_radps=rear_omega)
        control = Control(throttle=throttle, steer_rad=steer)
        result = model.evaluate(state, control)
        rate = result.derivative
        residual = [rate.vx_mps, rate.vy_mps, rate.yaw_rate_radps, rate.roll_rate_radps,
                    rate.pitch_rate_radps, rate.heave_rate_mps,
                    rate.front_wheel_radps * v.wheel_radius_m, rate.rear_wheel_radps * v.wheel_radius_m]
        if any(result.signals[key] for key in ("wheel_lift", "lean_limit_exceeded", "pitch_limit_exceeded", "suspension_limit_exceeded")):
            raise ValueError("Trim exceeds chassis model limits")
        return residual, state, control

    wheelbase = v.front_distance_m + v.rear_distance_m
    rear_fy = v.mass_kg * speed * yaw * v.front_distance_m / wheelbase
    vy = v.rear_distance_m * yaw - speed * rear_fy / config.tires.rear_stiffness_nprad
    omega = speed / v.wheel_radius_m
    values = [vy, math.atan(wheelbase / radius), 0.1, math.atan(speed * yaw / gravity), 0.0, 0.0, omega, omega]
    for iteration in range(params["trim_iterations"]):
        residual, state, control = evaluate(values)
        norm = max(map(abs, residual))
        if norm < params["trim_tolerance"]:
            return state, control, dict(trim_residual=norm, trim_iterations=iteration + 1,
                                       target_radius_m=radius, target_speed_mps=speed,
                                       trim_roll_deg=math.degrees(state.roll_rad), trim_pitch_deg=math.degrees(state.pitch_rad))
        columns = []
        for index in range(len(values)):
            delta = 1e-6 * max(1, abs(values[index]))
            candidate = values.copy()
            candidate[index] += delta
            columns.append([(b - a) / delta for a, b in zip(residual, evaluate(candidate)[0])])
        change = solve_linear(list(zip(*columns)), [-x for x in residual])
        for exponent in range(20):
            candidate = [x + dx * 0.5**exponent for x, dx in zip(values, change)]
            try:
                improved = max(map(abs, evaluate(candidate)[0])) < norm
            except ValueError:
                improved = False
            if improved:
                values = candidate
                break
        else:
            raise ValueError("Roll/pitch trim failed: request a lower speed or larger radius")
    raise ValueError("Roll/pitch trim did not converge within trim_iterations")
