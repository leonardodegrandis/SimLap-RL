import copy
import importlib.util
import unittest
from dataclasses import replace

from simlap import load_config
from simlap.rl.config import load_settings
from simlap.tracks import load_track

HAS_RL = all(importlib.util.find_spec(name) for name in ("gymnasium", "numpy", "stable_baselines3"))
if HAS_RL:
    import numpy as np
    from simlap.rl.environment import MotorcycleEnv


@unittest.skipUnless(HAS_RL, "Install simlap[rl] to test the optional learning environment")
class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_settings()
        self.env = MotorcycleEnv(load_config(), load_track(), self.settings, evaluation=True)

    def test_seeded_reset_and_gymnasium_contract(self):
        from stable_baselines3.common.env_checker import check_env
        check_env(self.env, warn=True)
        training = MotorcycleEnv(load_config(), load_track(), self.settings)
        first, _ = training.reset(seed=42)
        training.step([0.5, 0.05])
        second, _ = training.reset(seed=42)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(len(first), 18 + 3 * len(self.settings["environment"]["preview_distances_m"]))
        self.assertTrue(training.observation_space.contains(first))

    def test_policy_controls_all_actuators_without_an_autopilot(self):
        self.env.reset()
        self.env.step([0.7, 0.2])
        control = self.env.last_control
        self.assertAlmostEqual(control.throttle, .7, places=6)
        self.assertEqual(control.brake, 0)
        self.assertAlmostEqual(control.steer_rad, .2 * self.env.max_steer, places=6)
        self.env.step([-.5, -.1])
        self.assertEqual(self.env.last_control.throttle, 0)
        self.assertAlmostEqual(self.env.last_control.brake, .5)
        self.assertLess(self.env.last_control.steer_rad, 0)
        with self.assertRaises(ValueError):
            self.env.step([2, 0])

    def test_roll_pitch_observations_and_direct_steering(self):
        from simlap.types import RollPitchState
        config = load_config(overlays=["configs/roll_pitch.json"])
        env = MotorcycleEnv(config, load_track(), self.settings, evaluation=True)
        obs, _ = env.reset()
        self.assertIsInstance(env.sim.state, RollPitchState)
        self.assertEqual(len(obs), len(self.env.observation_names) + 8)
        self.assertTrue(env.observation_space.contains(obs))
        obs, _, _, _, info = env.step([.4, .2])
        self.assertAlmostEqual(env.last_control.steer_rad, .2 * env.max_steer, places=6)
        self.assertLess(env.sim.state.roll_rad, 0)  # Initial countersteering, no balance autopilot.
        self.assertIn("front_normal_n", info["telemetry"])
        self.assertTrue(env.observation_space.contains(obs))

    def test_boundary_checked_inside_control_interval(self):
        self.env.reset(options={"lateral_m": -3.35, "speed_mps": 10})
        self.env.sim.state = replace(self.env.sim.state, vy_mps=-12)
        _, reward, terminated, truncated, info = self.env.step([0, 0])
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["termination_reason"], "off_track")
        self.assertLess(self.env.sim.time_s, self.env.vehicle.simulation.control_dt_s)
        self.assertLess(reward, -100)
        with self.assertRaises(RuntimeError):
            self.env.step([0, 0])

    def test_time_limit_truncates_and_completion_terminates(self):
        settings = copy.deepcopy(self.settings)
        settings["environment"]["max_episode_s"] = .02
        env = MotorcycleEnv(load_config(), load_track(), settings, evaluation=True)
        env.reset()
        _, _, terminated, truncated, info = env.step([0, 0])
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertIsNone(info["lap_time_s"])
        self.env.reset()
        self.env.progress = self.env.track.length_m - .01
        _, _, terminated, truncated, info = self.env.step([0, 0])
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info["is_success"])
        self.assertGreater(info["lap_time_s"], 0)

    def test_crossing_finish_does_not_award_an_unearned_lap(self):
        self.env.reset(options={"start_s_m": self.env.track.length_m - .01})
        _, _, terminated, _, info = self.env.step([0, 0])
        self.assertFalse(terminated)
        self.assertGreater(info["progress_m"], 0)
        self.assertLess(info["completion_fraction"], .01)

    def test_terminal_info_retains_state_for_training_logs(self):
        self.env.reset(options={"lateral_m": -3.35, "speed_mps": 10})
        self.env.sim.state = replace(self.env.sim.state, vy_mps=-12)
        _, _, _, _, info = self.env.step([.2, 0])
        last_y = self.env.sim.state.y_m
        self.env.reset()
        self.assertEqual(info["telemetry"]["y_m"], last_y)
        self.assertNotEqual(info["telemetry"]["y_m"], self.env.sim.state.y_m)
        self.assertEqual(set(info["reward_terms"]), {"progress", "time", "action_change", "edge", "terminal"})
