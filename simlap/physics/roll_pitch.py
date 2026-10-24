"""Reduced six-DOF sprung chassis plus wheel rotation; see physics/README.md.

Roll is positive leaning left; pitch and heave are positive nose/up. Pitch uses
small-angle suspension geometry. Steering is an actuator input, not a rider.
"""

import math
from dataclasses import asdict

from ..mathutils import clip
from ..types import Evaluation, RollPitchState
from .powertrain import ElectricPowertrain
from .tires import CombinedSlipTire


class RollPitchMotorcycle:
    def __init__(self, config, tires=None, powertrain=None):
        self.c = config
        self.tires = tires or CombinedSlipTire(config.tires)
        self.powertrain = powertrain or ElectricPowertrain(config)

    def initial_state(self, state):
        if isinstance(state, RollPitchState):
            return state
        omega = state.vx_mps / self.c.vehicle.wheel_radius_m
        return RollPitchState(**asdict(state), front_wheel_radps=omega, rear_wheel_radps=omega)

    def cornering_trim(self, params):
        from .trim import roll_pitch_trim
        return roll_pitch_trim(self, self.c, params)

    def contact_positions(self, state):
        v = self.c.vehicle
        lateral = -v.cg_height_m * math.sin(state.roll_rad)
        cs, sn = math.cos(state.heading_rad), math.sin(state.heading_rad)
        return [(state.x_m + x * cs - lateral * sn, state.y_m + x * sn + lateral * cs)
                for x in (v.front_distance_m, -v.rear_distance_m)]

    def evaluate(self, state, control):
        c, s = self.c, state
        v, d, a, env = c.vehicle, c.dynamics, c.actuators, c.environment
        front_arm, rear_arm, h, radius = v.front_distance_m, v.rear_distance_m, v.cg_height_m, v.wheel_radius_m
        wheelbase, weight = front_arm + rear_arm, v.mass_kg * env.gravity_mps2
        sin_roll, cos_roll = math.sin(s.roll_rad), math.cos(s.roll_rad)
        height = h * cos_roll + s.heave_m
        front_extension, rear_extension = s.heave_m + front_arm * s.pitch_rad, s.heave_m - rear_arm * s.pitch_rad
        front_rate, rear_rate = s.heave_rate_mps + front_arm * s.pitch_rate_radps, s.heave_rate_mps - rear_arm * s.pitch_rate_radps
        front_load = max(0, weight * rear_arm / wheelbase - d.front_spring_npm * front_extension - d.front_damper_nspm * front_rate)
        rear_load = max(0, weight * front_arm / wheelbase - d.rear_spring_npm * rear_extension - d.rear_damper_nspm * rear_rate)

        steer = clip(s.steer_rad, -math.radians(a.max_steer_deg), math.radians(a.max_steer_deg))
        cs, sn = math.cos(steer), math.sin(steer)
        contact_vx = s.vx_mps + height * s.pitch_rate_radps + h * sin_roll * s.yaw_rate_radps
        front_vy = s.vy_mps + front_arm * s.yaw_rate_radps - height * s.roll_rate_radps
        rear_vy = s.vy_mps - rear_arm * s.yaw_rate_radps - height * s.roll_rate_radps
        front_vx, front_vy = contact_vx * cs + front_vy * sn, -contact_vx * sn + front_vy * cs
        front = self.tires.evaluate(front_vx, front_vy, radius * s.front_wheel_radps, front_load,
                                    d.front_longitudinal_stiffness_n, c.tires.front_stiffness_nprad)
        rear = self.tires.evaluate(contact_vx, rear_vy, radius * s.rear_wheel_radps, rear_load,
                                   d.rear_longitudinal_stiffness_n, c.tires.rear_stiffness_nprad)
        # The existing electrical module now sees actual shaft speed, including wheelspin.
        prepared = self.powertrain.prepare(radius * s.rear_wheel_radps, s.soc, clip(s.throttle, 0, 1), clip(s.brake, 0, 1))
        _, pack, signals = self.powertrain.deliver(prepared, math.inf)
        taper_front = math.tanh(radius * s.front_wheel_radps / a.brake_regularization_mps)
        taper_rear = math.tanh(radius * s.rear_wheel_radps / a.brake_regularization_mps)
        front_brake = a.max_brake_force_n * clip(s.brake, 0, 1) * a.front_brake_fraction * radius * taper_front
        rear_brake = signals["rear_friction_brake_force_n"] * radius * taper_rear
        front_rolling = v.rolling_resistance * front_load * radius * taper_front
        rear_rolling = v.rolling_resistance * rear_load * radius * taper_rear
        drive_torque = signals["drive_force_n"] * radius
        regen_torque = signals["regen_force_n"] * radius
        front_omega_dot = (-front_brake - front_rolling - radius * front.longitudinal_n) / d.front_wheel_inertia_kgm2
        rear_omega_dot = (drive_torque - regen_torque - rear_brake - rear_rolling - radius * rear.longitudinal_n) / (
            d.rear_wheel_inertia_kgm2 + d.motor_inertia_kgm2 * c.motor.gear_ratio**2)

        front_fx = front.longitudinal_n * cs - front.lateral_n * sn
        front_fy = front.longitudinal_n * sn + front.lateral_n * cs
        tire_fx, tire_fy = front_fx + rear.longitudinal_n, front_fy + rear.lateral_n
        airspeed = s.vx_mps + env.headwind_mps
        drag = 0.5 * env.air_density_kgpm3 * v.drag_coefficient * v.frontal_area_m2 * airspeed * abs(airspeed)
        ax, ay = (tire_fx - drag) / v.mass_kg, tire_fy / v.mass_kg
        yaw_accel = (front_arm * front_fy - rear_arm * rear.lateral_n + h * sin_roll * tire_fx) / v.yaw_inertia_kgm2
        roll_moment = h * sin_roll * (front_load + rear_load) - height * tire_fy - d.roll_damping_nmsprad * s.roll_rate_radps
        pitch_moment = height * tire_fx + front_arm * front_load - rear_arm * rear_load
        roll_accel, pitch_accel = roll_moment / d.roll_inertia_kgm2, pitch_moment / d.pitch_inertia_kgm2
        heave_accel = (front_load + rear_load) / v.mass_kg - env.gravity_mps2 + h * cos_roll * s.roll_rate_radps**2 + h * sin_roll * roll_accel
        steer_target = control.steer_rad
        if control.yaw_rate_radps is not None:
            target = clip(control.yaw_rate_radps, -a.yaw_rate_max_radps, a.yaw_rate_max_radps)
            steer_target = math.atan(wheelbase * target / max(s.vx_mps, a.yaw_rate_min_speed_mps)) + a.yaw_rate_gain_s * (target - s.yaw_rate_radps)
        steer_target = clip(steer_target, -math.radians(a.max_steer_deg), math.radians(a.max_steer_deg))
        friction_heat = front_brake * s.front_wheel_radps + rear_brake * s.rear_wheel_radps
        tire_heat = (front.longitudinal_n * (radius * s.front_wheel_radps - front_vx) - front.lateral_n * front_vy
                     + rear.longitudinal_n * (radius * s.rear_wheel_radps - contact_vx) - rear.lateral_n * rear_vy)
        rolling_heat = front_rolling * s.front_wheel_radps + rear_rolling * s.rear_wheel_radps
        suspension_heat = d.front_damper_nspm * front_rate**2 + d.rear_damper_nspm * rear_rate**2
        roll_heat = d.roll_damping_nmsprad * s.roll_rate_radps**2
        derivative = RollPitchState(
            x_m=s.vx_mps * math.cos(s.heading_rad) - s.vy_mps * math.sin(s.heading_rad),
            y_m=s.vx_mps * math.sin(s.heading_rad) + s.vy_mps * math.cos(s.heading_rad),
            heading_rad=s.yaw_rate_radps, vx_mps=ax + s.yaw_rate_radps * s.vy_mps,
            vy_mps=ay - s.yaw_rate_radps * s.vx_mps, yaw_rate_radps=yaw_accel,
            steer_rad=clip((steer_target - s.steer_rad) / a.steer_time_constant_s,
                           -math.radians(a.steer_rate_degps), math.radians(a.steer_rate_degps)),
            throttle=((0 if control.brake > 0 else control.throttle) - s.throttle) / a.throttle_time_constant_s,
            brake=(control.brake - s.brake) / a.brake_time_constant_s,
            soc=pack.soc_rate, chemical_energy_j=pack.chemical_power_w, terminal_energy_j=pack.terminal_power_w,
            battery_heat_j=pack.heat_w, motor_heat_j=signals["motor_loss_w"], inverter_heat_j=signals["inverter_loss_w"],
            gearbox_heat_j=signals["gearbox_loss_w"], friction_brake_heat_j=friction_heat,
            distance_m=math.hypot(s.vx_mps, s.vy_mps), roll_rad=s.roll_rate_radps, roll_rate_radps=roll_accel,
            pitch_rad=s.pitch_rate_radps, pitch_rate_radps=pitch_accel, heave_m=s.heave_rate_mps, heave_rate_mps=heave_accel,
            front_wheel_radps=front_omega_dot, rear_wheel_radps=rear_omega_dot,
            tire_heat_j=tire_heat, rolling_heat_j=rolling_heat, suspension_heat_j=suspension_heat, roll_damper_heat_j=roll_heat)
        signals.update(
            speed_mps=math.hypot(s.vx_mps, s.vy_mps), ax_mps2=ax, ay_mps2=ay,
            vx_derivative_mps2=derivative.vx_mps, vy_derivative_mps2=derivative.vy_mps,
            yaw_acceleration_radps2=yaw_accel, roll_acceleration_radps2=roll_accel, pitch_acceleration_radps2=pitch_accel,
            heave_acceleration_mps2=heave_accel, roll_moment_nm=roll_moment, pitch_moment_nm=pitch_moment,
            roll_deg=math.degrees(s.roll_rad), pitch_deg=math.degrees(s.pitch_rad),
            lean_estimate_deg=math.degrees(math.atan2(ay, env.gravity_mps2)), cg_height_m=height,
            front_normal_n=front_load, rear_normal_n=rear_load,
            front_fx_n=front.longitudinal_n, front_fy_n=front.lateral_n, rear_fx_n=rear.longitudinal_n, rear_fy_n=rear.lateral_n,
            front_body_fx_n=front_fx, front_body_fy_n=front_fy,
            front_slip_ratio=front.slip_ratio, rear_slip_ratio=rear.slip_ratio,
            front_slip_rad=front.slip_rad, rear_slip_rad=rear.slip_rad,
            front_grip_usage=math.hypot(front.longitudinal_n, front.lateral_n) / max(front.capacity_n, 1e-9),
            rear_grip_usage=math.hypot(rear.longitudinal_n, rear.lateral_n) / max(rear.capacity_n, 1e-9),
            front_contact_vx_mps=front_vx, front_contact_vy_mps=front_vy,
            rear_contact_vx_mps=contact_vx, rear_contact_vy_mps=rear_vy,
            front_suspension_displacement_m=front_extension, rear_suspension_displacement_m=rear_extension,
            front_brake_torque_nm=front_brake, rear_brake_torque_nm=rear_brake,
            drive_torque_nm=drive_torque, regen_torque_nm=regen_torque,
            front_wheel_acceleration_radps2=front_omega_dot, rear_wheel_acceleration_radps2=rear_omega_dot,
            drag_force_n=drag, rolling_force_n=(front_rolling + rear_rolling) / radius,
            friction_brake_loss_w=friction_heat, tire_scrub_loss_w=tire_heat, rolling_loss_w=rolling_heat,
            suspension_loss_w=suspension_heat, roll_damper_loss_w=roll_heat,
            rear_friction_brake_force_n=rear_brake / radius, steer_target_rad=steer_target,
            load_transfer_residual_n=0.0, load_transfer_iterations=0,
            wheel_lift=float(min(front_load, rear_load) <= 0),
            lean_limit_exceeded=float(abs(math.degrees(s.roll_rad)) > v.max_lean_deg),
            pitch_limit_exceeded=float(abs(math.degrees(s.pitch_rad)) > d.max_pitch_deg),
            suspension_limit_exceeded=float(max(abs(front_extension), abs(rear_extension)) > d.max_suspension_displacement_m))
        return Evaluation(derivative, signals)
