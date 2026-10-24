"""Forward-only planar chassis; algebraic load transfer, no roll/pitch states."""

import math

from ..mathutils import clip
from .powertrain import ElectricPowertrain, PowertrainModel
from .tires import SaturatingTire, TireModel
from ..types import Control, Evaluation, State


class PlanarMotorcycle:
    def __init__(self, config, tires: TireModel = None, powertrain: PowertrainModel = None):
        self.c = config
        self.tires = tires or SaturatingTire(config.tires)
        self.powertrain = powertrain or ElectricPowertrain(config)

    def evaluate(self, state: State, control: Control):
        c, s = self.c, state
        v, a, env = c.vehicle, c.actuators, c.environment
        wheelbase = v.front_distance_m + v.rear_distance_m
        speed = max(0.0, s.vx_mps)
        steer = clip(s.steer_rad, -math.radians(a.max_steer_deg), math.radians(a.max_steer_deg))
        cs, sn = math.cos(steer), math.sin(steer)
        throttle, brake = clip(s.throttle, 0, 1), clip(s.brake, 0, 1)
        prepared = self.powertrain.prepare(speed, s.soc, throttle, brake)
        weight = v.mass_kg * env.gravity_mps2
        front_velocity_y = s.vy_mps + v.front_distance_m * s.yaw_rate_radps
        front_vx = speed * cs + front_velocity_y * sn
        front_vy = -speed * sn + front_velocity_y * cs
        rear_vy = s.vy_mps - v.rear_distance_m * s.yaw_rate_radps
        front_load = weight * v.rear_distance_m / wheelbase
        front_brake_request = a.max_brake_force_n * brake * a.front_brake_fraction
        residual = float("inf")
        for iteration in range(c.simulation.load_transfer_iterations):
            front = self.tires.evaluate(front_vx, front_vy, front_load, c.tires.front_stiffness_nprad)
            rear = self.tires.evaluate(speed, rear_vy, weight - front_load, c.tires.rear_stiffness_nprad)
            # Smooth friction braking at rest; no reverse or wheel-lock dynamics.
            brake_taper = math.tanh(speed / a.brake_regularization_mps)
            front_fx = -min(front_brake_request, front.longitudinal_capacity_n) * brake_taper
            rear_fx, pack, signals = self.powertrain.deliver(prepared, rear.longitudinal_capacity_n)
            if rear_fx < 0:
                # Regen already fades with speed; taper only the friction brake.
                friction = signals["rear_friction_brake_force_n"]
                rear_fx += friction * (1 - brake_taper)
                signals["rear_friction_brake_force_n"] *= brake_taper
            front_body_fx = front_fx * cs - front.lateral_n * sn
            front_body_fy = front_fx * sn + front.lateral_n * cs
            tire_fx = front_body_fx + rear_fx
            target_load = (weight * v.rear_distance_m - v.cg_height_m * tire_fx) / wheelbase
            residual = target_load - front_load
            if abs(residual) < c.simulation.load_transfer_tolerance_n:
                break
            if iteration + 1 < c.simulation.load_transfer_iterations:
                front_load = clip(front_load + 0.6 * residual, 0, weight)

        relative_air = speed + env.headwind_mps
        drag = 0.5 * env.air_density_kgpm3 * v.drag_coefficient * v.frontal_area_m2 * relative_air * abs(relative_air)
        rolling = v.rolling_resistance * weight * math.tanh(speed / a.brake_regularization_mps)
        ax = (tire_fx - drag - rolling) / v.mass_kg
        ay = (front_body_fy + rear.lateral_n) / v.mass_kg
        yaw_accel = (v.front_distance_m * front_body_fy - v.rear_distance_m * rear.lateral_n) / v.yaw_inertia_kgm2
        steer_target = control.steer_rad
        if control.yaw_rate_radps is not None:
            yaw_target = clip(control.yaw_rate_radps, -a.yaw_rate_max_radps, a.yaw_rate_max_radps)
            steer_target = math.atan(wheelbase * yaw_target / max(speed, a.yaw_rate_min_speed_mps))
            steer_target += a.yaw_rate_gain_s * (yaw_target - s.yaw_rate_radps)
        steer_target = clip(steer_target, -math.radians(a.max_steer_deg), math.radians(a.max_steer_deg))
        steer_rate = clip((steer_target - s.steer_rad) / a.steer_time_constant_s,
                          -math.radians(a.steer_rate_degps), math.radians(a.steer_rate_degps))
        friction_heat = -front_fx * max(0, front_vx) + signals["rear_friction_brake_force_n"] * speed
        derivative = State(
            x_m=speed * math.cos(s.heading_rad) - s.vy_mps * math.sin(s.heading_rad),
            y_m=speed * math.sin(s.heading_rad) + s.vy_mps * math.cos(s.heading_rad),
            heading_rad=s.yaw_rate_radps,
            vx_mps=ax + s.yaw_rate_radps * s.vy_mps,
            vy_mps=ay - s.yaw_rate_radps * speed,
            yaw_rate_radps=yaw_accel,
            steer_rad=steer_rate,
            throttle=((0 if control.brake > 0 else control.throttle) - s.throttle) / a.throttle_time_constant_s,
            brake=(control.brake - s.brake) / a.brake_time_constant_s,
            soc=pack.soc_rate,
            chemical_energy_j=pack.chemical_power_w,
            terminal_energy_j=pack.terminal_power_w,
            battery_heat_j=pack.heat_w,
            motor_heat_j=signals["motor_loss_w"],
            inverter_heat_j=signals["inverter_loss_w"],
            gearbox_heat_j=signals["gearbox_loss_w"],
            friction_brake_heat_j=friction_heat,
            distance_m=math.hypot(speed, s.vy_mps),
        )
        lean = math.atan2(ay, env.gravity_mps2)
        signals.update(
            speed_mps=math.hypot(speed, s.vy_mps), ax_mps2=ax, ay_mps2=ay,
            vx_derivative_mps2=derivative.vx_mps, vy_derivative_mps2=derivative.vy_mps,
            yaw_acceleration_radps2=yaw_accel, lean_estimate_deg=math.degrees(lean),
            lean_limit_exceeded=float(abs(math.degrees(lean)) > v.max_lean_deg),
            front_normal_n=front_load, rear_normal_n=weight - front_load,
            front_fx_n=front_fx, rear_fx_n=rear_fx, front_fy_n=front.lateral_n, rear_fy_n=rear.lateral_n,
            front_slip_rad=front.slip_rad, rear_slip_rad=rear.slip_rad,
            front_grip_usage=math.hypot(front_fx, front.lateral_n) / max(front.capacity_n, 1e-9),
            rear_grip_usage=math.hypot(rear_fx, rear.lateral_n) / max(rear.capacity_n, 1e-9),
            drag_force_n=drag, rolling_force_n=rolling, friction_brake_loss_w=friction_heat,
            tire_scrub_loss_w=-(front.lateral_n * front_vy + rear.lateral_n * rear_vy),
            steer_target_rad=steer_target, load_transfer_residual_n=residual,
            load_transfer_iterations=iteration + 1,
            wheel_lift=float(target_load <= 0 or target_load >= weight),
        )
        return Evaluation(derivative, signals)
