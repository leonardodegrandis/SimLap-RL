"""Physical invariants, limits and convergence; run with unittest discovery."""

import json
import math
import tempfile
import unittest
from dataclasses import dataclass, replace
from pathlib import Path

from simlap import Control, Simulator, State, load_config
from simlap.battery import ResistiveBattery
from simlap.config import validate
from simlap.dynamics import PlanarMotorcycle
from simlap.maneuvers import cornering_trim, load_maneuvers
from simlap.powertrain import ElectricPowertrain
from simlap.run import run_maneuver
from simlap.types import Evaluation


class PhysicsTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config()

    def test_pack_power_balance_and_limits(self):
        for resistance in (0, .035, .4):
            battery = ResistiveBattery(replace(self.c.battery, resistance_ohm=resistance))
            for request in (-1e6, -20000, 0, 20000, 1e6):
                flow = battery.flow(request, .9)
                self.assertAlmostEqual(flow.chemical_power_w, flow.terminal_power_w + flow.heat_w, places=8)
                self.assertAlmostEqual(flow.terminal_power_w, flow.current_a * flow.terminal_voltage_v, places=8)
                self.assertLessEqual(flow.current_a, self.c.battery.max_discharge_a + 1e-8)
                self.assertGreaterEqual(flow.current_a, -self.c.battery.max_charge_a - 1e-8)
                self.assertGreaterEqual(flow.terminal_voltage_v, self.c.battery.min_voltage_v - 1e-8)
                self.assertLessEqual(flow.terminal_voltage_v, self.c.battery.max_voltage_v + 1e-8)
        battery = ResistiveBattery(self.c.battery)
        self.assertEqual(battery.flow(10000, self.c.battery.min_soc).current_a, 0)
        self.assertEqual(battery.flow(-10000, self.c.battery.max_soc).current_a, 0)

    def test_peak_map_nominal_scaling_and_launch(self):
        peak = ElectricPowertrain(self.c)
        nominal = ElectricPowertrain(replace(self.c, motor=replace(self.c.motor, rating="nominal")))
        for rpm in (0, 500, 3000, 4500, 6000, 8000, 9000, 10000):
            torque = peak.torque_envelope(rpm)
            self.assertLessEqual(torque * rpm * math.pi / 30, 48000 + 1e-8)
            self.assertAlmostEqual(nominal.torque_envelope(rpm), torque * .75)
        self.assertAlmostEqual(peak.torque_envelope(4500) * 4500 * math.pi / 30, 48000)
        result = PlanarMotorcycle(self.c).evaluate(State(throttle=1), Control(throttle=1))
        self.assertGreater(result.derivative.vx_mps, 0)
        self.assertGreater(result.signals["battery_current_a"], 0)
        self.assertEqual(result.signals["motor_shaft_power_w"], 0)
        self.assertGreater(result.signals["motor_loss_w"], 0)

    def test_tire_limits_and_power_consistency(self):
        model = PlanarMotorcycle(self.c)
        for speed in (0, 1, 15, 35, 60):
            for throttle, brake in ((1, 0), (0, 1), (.5, .5)):
                for steer in (0, .03):
                    result = model.evaluate(State(vx_mps=speed, throttle=throttle, brake=brake, steer_rad=steer),
                                            Control(throttle=throttle, brake=brake, steer_rad=steer))
                    d = result.signals
                    self.assertLessEqual(d["front_grip_usage"], 1 + 1e-10)
                    self.assertLessEqual(d["rear_grip_usage"], 1 + 1e-10)
                    self.assertAlmostEqual(d["front_normal_n"] + d["rear_normal_n"], self.c.vehicle.mass_kg * 9.81)
                    self.assertLess(abs(d["power_balance_error_w"]), 1e-5)
                    for key in ("motor_loss_w", "inverter_loss_w", "gearbox_loss_w", "battery_loss_w", "friction_brake_loss_w"):
                        self.assertGreaterEqual(d[key], -1e-8)

    def test_regen_brake_blending_and_full_battery(self):
        state = State(vx_mps=20, brake=1)
        result = PlanarMotorcycle(self.c).evaluate(state, Control(brake=1)).signals
        self.assertLess(result["battery_terminal_power_w"], 0)
        self.assertGreater(result["regen_force_n"], 0)
        full = PlanarMotorcycle(self.c).evaluate(replace(state, soc=self.c.battery.max_soc), Control(brake=1)).signals
        self.assertGreaterEqual(full["battery_terminal_power_w"], -1e-8)
        self.assertAlmostEqual(full["rear_fx_n"], result["rear_fx_n"], places=7)
        disabled_config = replace(self.c, actuators=replace(self.c.actuators, regen_enabled=False))
        disabled = PlanarMotorcycle(disabled_config).evaluate(state, Control(brake=1)).signals
        self.assertEqual(disabled["regen_force_n"], 0)
        self.assertAlmostEqual(disabled["ax_mps2"], result["ax_mps2"], places=7)

    def test_empty_pack_and_battery_limited_torque(self):
        empty = PlanarMotorcycle(self.c).evaluate(State(vx_mps=20, throttle=1, soc=self.c.battery.min_soc), Control(throttle=1)).signals
        self.assertAlmostEqual(empty["motor_torque_nm"], 0, places=6)
        c = replace(self.c, battery=replace(self.c.battery, max_discharge_a=20))
        result = PlanarMotorcycle(c).evaluate(State(vx_mps=20, throttle=1), Control(throttle=1)).signals
        self.assertLessEqual(result["battery_current_a"], 20 + 1e-8)
        self.assertLess(result["available_drive_torque_nm"], result["requested_drive_torque_nm"])
        self.assertLess(abs(result["power_balance_error_w"]), 1e-5)

    def test_straight_line_and_timestep_convergence(self):
        final = []
        for dt in (.005, .0025):
            c = replace(self.c, simulation=replace(self.c.simulation, dt_s=dt))
            simulator = Simulator(c)
            state, _ = simulator.advance(Control(throttle=1), 5.0)
            self.assertEqual(state.y_m, 0)
            self.assertEqual(state.yaw_rate_radps, 0)
            self.assertAlmostEqual(state.chemical_energy_j, state.terminal_energy_j + state.battery_heat_j, places=6)
            final.append(state)
        self.assertLess(abs(final[0].vx_mps - final[1].vx_mps), .005)
        self.assertLess(abs(final[0].x_m - final[1].x_m), .02)

    def test_braking_stops_without_reverse(self):
        simulator = Simulator(self.c)
        simulator.reset(State(vx_mps=100 / 3.6))
        while simulator.time_s < 5 and simulator.state.vx_mps > self.c.simulation.stop_speed_mps:
            simulator.step(Control(brake=1))
        self.assertLess(simulator.state.vx_mps, self.c.simulation.stop_speed_mps)
        self.assertGreaterEqual(simulator.state.vx_mps, 0)
        self.assertLess(simulator.state.distance_m, 45)
        self.assertGreater(simulator.state.soc, .9)

    def test_corner_trim_holds_both_directions(self):
        for radius in (50, -50):
            params = dict(load_maneuvers()["cornering"], radius_m=radius)
            simulator = Simulator(self.c)
            state, control, metadata = cornering_trim(simulator.model, self.c, params)
            self.assertLess(metadata["trim_residual"], params["trim_tolerance"])
            simulator.reset(state)
            final, signals = simulator.advance(control, 3.0)
            self.assertLess(abs(math.hypot(final.x_m, final.y_m - radius) - abs(radius)), .001)
            self.assertLess(abs(math.hypot(final.vx_mps, final.vy_mps) - 15), .001)
            self.assertAlmostEqual(signals["lean_estimate_deg"], math.degrees(math.atan(signals["ay_mps2"] / 9.81)))
        with self.assertRaises(ValueError):
            cornering_trim(simulator.model, self.c, dict(params, radius_m=5))

    def test_yaw_command_is_actuated_not_teleported(self):
        simulator = Simulator(self.c)
        simulator.reset(State(vx_mps=15))
        state, signals = simulator.step(Control(yaw_rate_radps=.2))
        self.assertGreater(state.steer_rad, 0)
        self.assertLess(state.yaw_rate_radps, .2)
        with self.assertRaises(ValueError):
            Control(steer_rad=.1, yaw_rate_radps=.2)

    def test_chassis_mechanical_power_balance(self):
        state = State(vx_mps=20, vy_mps=.2, yaw_rate_radps=.1, steer_rad=.03, brake=.5)
        result = PlanarMotorcycle(self.c).evaluate(state, Control(brake=.5, steer_rad=.03))
        d, q, v = result.derivative, result.signals, self.c.vehicle
        kinetic_rate = v.mass_kg * (state.vx_mps * d.vx_mps + state.vy_mps * d.vy_mps)
        kinetic_rate += v.yaw_inertia_kgm2 * state.yaw_rate_radps * d.yaw_rate_radps
        expected = q["wheel_drive_power_w"] - q["friction_brake_loss_w"] - q["tire_scrub_loss_w"]
        expected -= (q["drag_force_n"] + q["rolling_force_n"]) * state.vx_mps
        self.assertAlmostEqual(kinetic_rate, expected, places=7)

    def test_soc_boundary_stops_without_losing_energy_accounting(self):
        simulator = Simulator(self.c)
        simulator.reset(State(vx_mps=20, throttle=1, soc=self.c.battery.min_soc + 1e-7))
        state, _ = simulator.advance(Control(throttle=1))
        self.assertEqual(simulator.status, "battery_soc_boundary")
        self.assertEqual(state.soc, self.c.battery.min_soc)
        self.assertLess(simulator.time_s, self.c.simulation.dt_s)
        self.assertAlmostEqual(state.chemical_energy_j, state.terminal_energy_j + state.battery_heat_j, places=8)
        with self.assertRaises(RuntimeError):
            simulator.step(Control())

    def test_configuration_rejects_typos_and_invalid_physics(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "overlay.json"
            path.write_text('{"motor": {"nominal_fator": 0.5}}')
            with self.assertRaises(ValueError):
                load_config(overlays=[path])
        for c in (replace(self.c, motor=replace(self.c.motor, nominal_factor=2)),
                  replace(self.c, battery=replace(self.c.battery, resistance_ohm=-1)),
                  replace(self.c, simulation=replace(self.c.simulation, dt_s=.1))):
            with self.assertRaises(ValueError):
                validate(c)

    def test_extended_state_and_replacement_model(self):
        @dataclass(frozen=True)
        class ThermalState(State):
            temperature_k: float = 293.15

        class HeatingModel:
            def evaluate(self, state, control):
                return Evaluation(ThermalState(soc=0, temperature_k=2), {})

        simulator = Simulator(self.c, model=HeatingModel())
        simulator.reset(ThermalState())
        state, _ = simulator.advance(Control(), 1)
        self.assertIsInstance(state, ThermalState)
        self.assertAlmostEqual(state.temperature_k, 295.15)

    def test_run_artifacts_and_initial_final_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            params = dict(load_maneuvers()["acceleration"], duration_s=.05)
            directory, summary = run_maneuver("acceleration", self.c, params, folder)
            self.assertEqual(summary["samples"], 11)
            data = json.loads((directory / "dashboard.json").read_text())
            self.assertEqual(data["rows"][0]["time_s"], 0)
            self.assertAlmostEqual(data["rows"][-1]["time_s"], .05)
            self.assertIn("solver=RK4", (directory / "engine.log").read_text())
            self.assertTrue((directory / "channels.json").exists())


if __name__ == "__main__":
    unittest.main()
