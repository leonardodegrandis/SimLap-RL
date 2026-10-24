"""Reduced-model equilibria, combined grip, dissipation and timestep convergence."""

import csv
import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from simlap import Control, RollPitchState, Simulator, State, load_config
from simlap.maneuvers import cornering_trim, load_maneuvers
from simlap.physics.roll_pitch import RollPitchMotorcycle
from simlap.run import run_maneuver


class RollPitchTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config(overlays=["configs/roll_pitch.json"])
        self.model = RollPitchMotorcycle(self.c)

    def test_static_equilibrium_and_state_conversion(self):
        sim = Simulator(self.c)
        result = sim.model.evaluate(sim.state, Control())
        rate, q = result.derivative, result.signals
        self.assertAlmostEqual(q["front_normal_n"] + q["rear_normal_n"], self.c.vehicle.mass_kg * 9.81)
        for name in ("vx_mps", "vy_mps", "yaw_rate_radps", "roll_rate_radps", "pitch_rate_radps", "heave_rate_mps"):
            self.assertAlmostEqual(getattr(rate, name), 0, places=10)
        state = sim.reset(State(vx_mps=15))
        self.assertIsInstance(state, RollPitchState)
        self.assertAlmostEqual(state.front_wheel_radps * self.c.vehicle.wheel_radius_m, 15)

    def test_combined_grip_dissipation_and_motor_uses_wheel_speed(self):
        for vx, vy, omega, roll in ((20, 2, 90, .2), (20, -2, .1, -.2), (0, 0, 30, 0)):
            s = RollPitchState(vx_mps=vx, vy_mps=vy, front_wheel_radps=omega, rear_wheel_radps=omega,
                              throttle=.8, steer_rad=.03, roll_rad=roll)
            q = self.model.evaluate(s, Control(throttle=.8, steer_rad=.03)).signals
            for axle in ("front", "rear"):
                self.assertLessEqual(math.hypot(q[f"{axle}_fx_n"], q[f"{axle}_fy_n"]),
                                     self.c.tires.friction_coefficient * q[f"{axle}_normal_n"] + 1e-9)
            for name in ("tire_scrub_loss_w", "rolling_loss_w", "suspension_loss_w", "friction_brake_loss_w", "battery_loss_w"):
                self.assertGreaterEqual(q[name], 0)
            self.assertAlmostEqual(q["motor_rpm"], omega * self.c.motor.gear_ratio * 30 / math.pi)
            self.assertLess(abs(q["power_balance_error_w"]), 1e-6)
        unloaded = self.model.tires.evaluate(20, 2, 40, 0, 20000, 18000)
        self.assertEqual(unloaded.longitudinal_n, 0)
        self.assertEqual(unloaded.lateral_n, 0)

    def test_countersteering_and_unstable_roll_are_dynamic(self):
        base = self.model.initial_state(State(vx_mps=15))
        left_steer = self.model.evaluate(replace(base, steer_rad=.02), Control(steer_rad=.02))
        self.assertGreater(left_steer.signals["front_fy_n"], 0)
        self.assertLess(left_steer.derivative.roll_rate_radps, 0)
        leaning = self.model.evaluate(replace(base, roll_rad=.03), Control())
        self.assertGreater(leaning.derivative.roll_rate_radps, 0)
        self.assertEqual(leaning.signals["steer_target_rad"], 0)

    def test_acceleration_squat_brake_dive_and_lockup(self):
        for control in (Control(throttle=1), Control(brake=1)):
            sim = Simulator(self.c)
            sim.reset(State(vx_mps=15))
            state, q = sim.advance(control, .5)
            self.assertEqual(sim.status, "running")
            if control.throttle:
                self.assertGreater(state.pitch_rad, 0)
                self.assertGreater(q["rear_normal_n"], q["front_normal_n"])
                self.assertGreater(q["rear_slip_ratio"], 0)
            else:
                self.assertLess(state.pitch_rad, 0)
                self.assertGreater(q["front_normal_n"], q["rear_normal_n"])
                self.assertLess(q["rear_slip_ratio"], -.8)
            self.assertAlmostEqual(state.chemical_energy_j, state.terminal_energy_j + state.battery_heat_j, places=7)

    def test_left_right_trim_and_mirror_symmetry(self):
        states = []
        for radius in (50, -50):
            params = dict(load_maneuvers("configs/maneuvers_roll_pitch.json")["cornering"], radius_m=radius)
            state, control, extra = cornering_trim(self.model, self.c, params)
            self.assertLess(extra["trim_residual"], params["trim_tolerance"])
            self.assertGreater(state.roll_rad * radius, 0)
            sim = Simulator(self.c)
            sim.reset(state)
            final, _ = sim.advance(control, .5)
            self.assertLess(abs(math.hypot(final.x_m, final.y_m - radius) - abs(radius)), 1e-5)
            states.append(state)
        self.assertAlmostEqual(states[0].roll_rad, -states[1].roll_rad, places=8)
        self.assertAlmostEqual(states[0].throttle, states[1].throttle, places=8)

    def test_adaptive_matches_small_fixed_step(self):
        final = []
        for method, step in (("rk45", .001), ("rk4", .0001)):
            config = replace(self.c, simulation=replace(self.c.simulation, integrator=method, max_internal_step_s=step))
            sim = Simulator(config)
            sim.reset(State(vx_mps=15))
            state, _ = sim.advance(Control(brake=.5), .2)
            final.append(state)
        for name in ("vx_mps", "pitch_rad", "heave_m", "front_wheel_radps", "rear_wheel_radps", "soc"):
            self.assertAlmostEqual(getattr(final[0], name), getattr(final[1], name), delta=2e-6)

    def test_mechanical_power_balance_including_wheels_and_suspension(self):
        v, d = self.c.vehicle, self.c.dynamics
        state = RollPitchState(vx_mps=20, vy_mps=.2, yaw_rate_radps=.1, steer_rad=.03, brake=.5,
                               roll_rad=.2, roll_rate_radps=.12, pitch_rad=.002, pitch_rate_radps=.03,
                               heave_m=.001, heave_rate_mps=.01, front_wheel_radps=63, rear_wheel_radps=64)
        result = self.model.evaluate(state, Control(brake=.5, steer_rad=.03))
        rate, q = result.derivative, result.signals
        vz = state.heave_rate_mps - v.cg_height_m * math.sin(state.roll_rad) * state.roll_rate_radps
        az = rate.heave_rate_mps - v.cg_height_m * (math.cos(state.roll_rad) * state.roll_rate_radps**2
                                                  + math.sin(state.roll_rad) * rate.roll_rate_radps)
        kinetic = v.mass_kg * (state.vx_mps * rate.vx_mps + state.vy_mps * rate.vy_mps + vz * az)
        kinetic += v.yaw_inertia_kgm2 * state.yaw_rate_radps * rate.yaw_rate_radps
        kinetic += d.roll_inertia_kgm2 * state.roll_rate_radps * rate.roll_rate_radps
        kinetic += d.pitch_inertia_kgm2 * state.pitch_rate_radps * rate.pitch_rate_radps
        kinetic += d.front_wheel_inertia_kgm2 * state.front_wheel_radps * rate.front_wheel_radps
        kinetic += (d.rear_wheel_inertia_kgm2 + d.motor_inertia_kgm2 * self.c.motor.gear_ratio**2) * state.rear_wheel_radps * rate.rear_wheel_radps
        weight, wheelbase = v.mass_kg * self.c.environment.gravity_mps2, v.front_distance_m + v.rear_distance_m
        potential = weight * vz
        potential += (d.front_spring_npm * (state.heave_m + v.front_distance_m * state.pitch_rad) - weight * v.rear_distance_m / wheelbase) * (state.heave_rate_mps + v.front_distance_m * state.pitch_rate_radps)
        potential += (d.rear_spring_npm * (state.heave_m - v.rear_distance_m * state.pitch_rad) - weight * v.front_distance_m / wheelbase) * (state.heave_rate_mps - v.rear_distance_m * state.pitch_rate_radps)
        expected = q["wheel_drive_power_w"] - q["drag_force_n"] * state.vx_mps
        expected -= sum(q[key] for key in ("friction_brake_loss_w", "tire_scrub_loss_w", "rolling_loss_w", "suspension_loss_w", "roll_damper_loss_w"))
        self.assertAlmostEqual(kinetic + potential, expected, places=7)

    def test_limits_and_complete_telemetry_schema(self):
        sim = Simulator(self.c)
        sim.reset(RollPitchState(heave_m=.1))
        sim.step(Control())
        self.assertEqual(sim.status, "wheel_lift_model_limit")
        with tempfile.TemporaryDirectory() as folder:
            params = dict(load_maneuvers()["acceleration"], duration_s=.02)
            path, summary = run_maneuver("acceleration", self.c, params, folder)
            with (path / "telemetry.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(summary["samples"], 5)
            self.assertIn("roll_rad", rows[0])
            self.assertIn("front_slip_ratio", rows[0])
            self.assertEqual(float(rows[0]["integrator_rhs_evaluations"]), 0)
            self.assertGreater(float(rows[-1]["integrator_rhs_evaluations"]), 0)
            self.assertIn("solver=RK45", (path / "engine.log").read_text())

    def test_old_checkpoint_config_defaults_to_planar(self):
        raw = self.c.to_dict()
        raw.pop("dynamics")
        for key in ("integrator", "relative_tolerance", "absolute_tolerance", "max_internal_step_s", "min_internal_step_s", "max_internal_steps"):
            raw["simulation"].pop(key)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "old.json"
            path.write_text(json.dumps(raw))
            config = load_config(path)
        self.assertEqual(config.dynamics.model, "planar")
        self.assertEqual(config.simulation.integrator, "rk4")
