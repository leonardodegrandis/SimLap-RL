"""Gymnasium adapter: a policy controls longitudinal demand and steering."""

import math
from dataclasses import asdict

import gymnasium as gym
import numpy as np

from ..engine import Simulator
from ..mathutils import clip
from ..tracks.base import signed_progress
from ..types import Control, RollPitchState, State


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TrackRecorder:
    """Add track/action channels without coupling the physics engine to a track."""

    def __init__(self, env, recorder):
        self.env, self.recorder, self.logger = env, recorder, recorder.logger

    def record(self, time_s, state, control, signals, cpu_ms=0.0):
        self.recorder.record(time_s, state, control, dict(signals, **self.env.track_signals(state)), cpu_ms)


class MotorcycleEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, vehicle, track, settings, evaluation=False, model=None, recorder=None):
        super().__init__()
        self.vehicle, self.track, self.settings = vehicle, track, settings
        self.c, self.reward_config = settings["environment"], settings["reward"]
        self.evaluation = evaluation
        self.padding = self.c["vehicle_half_width_m"] + self.c["boundary_margin_m"]
        self.half_width = track.width_m / 2 - self.padding
        if self.half_width <= self.c["initial_lateral_m"]:
            raise ValueError("Track is too narrow for the vehicle margin and reset offsets")
        if self.c["max_policy_steer_deg"] > vehicle.actuators.max_steer_deg:
            raise ValueError("Policy steering range exceeds the physical steering limit")
        self.max_steer = math.radians(self.c["max_policy_steer_deg"])
        # Positive longitudinal action is TPS; negative is brake. No shared-pedal ambiguity.
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_names = ["vx", "vy", "yaw_rate", "steer", "throttle", "brake", "soc",
                                  "lateral_error", "heading_sin", "heading_cos", "left_clearance", "right_clearance",
                                  "curvature", "lap_progress", "elapsed_time", "stall_time", "previous_longitudinal", "previous_steer"]
        for distance in self.c["preview_distances_m"]:
            self.observation_names.extend(f"preview_{distance:g}m_{key}" for key in ("x", "y", "curvature"))
        self.recorder = recorder
        self.sim = Simulator(vehicle, model=model, recorder=TrackRecorder(self, recorder) if recorder else None)
        self.roll_pitch = isinstance(self.sim.state, RollPitchState)
        if self.roll_pitch:
            self.observation_names.extend(["roll", "roll_rate", "pitch", "pitch_rate", "heave", "heave_rate",
                                           "front_wheel_speed", "rear_wheel_speed"])
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(len(self.observation_names),), dtype=np.float32)
        self.done = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        unknown = set(options) - {"start_s_m", "speed_mps", "lateral_m", "heading_error_rad"}
        if unknown:
            raise ValueError(f"Unknown reset options: {sorted(unknown)}")
        randomize = not self.evaluation
        start = options.get("start_s_m", self.np_random.uniform(0, self.track.length_m) if randomize and self.c["random_start"] else 0.0)
        speed = options.get("speed_mps", self.np_random.uniform(*self.c["initial_speed_mps"]) if randomize else self.c["initial_speed_mps"][0])
        lateral = options.get("lateral_m", self.np_random.uniform(-self.c["initial_lateral_m"], self.c["initial_lateral_m"]) if randomize else 0.0)
        heading = options.get("heading_error_rad", self.np_random.uniform(-self.c["initial_heading_rad"], self.c["initial_heading_rad"]) if randomize else 0.0)
        if not all(math.isfinite(v) for v in (start, speed, lateral, heading)) or speed < 0 or abs(lateral) >= self.half_width:
            raise ValueError("Invalid reset state")
        point = self.track.sample(start)
        state = State(x_m=point.x_m - lateral * math.sin(point.heading_rad),
                      y_m=point.y_m + lateral * math.cos(point.heading_rad),
                      heading_rad=point.heading_rad + heading, vx_mps=speed,
                      soc=self.vehicle.battery.initial_soc)
        state = self.sim.reset(state)
        self.start_s = self.track.project(state.x_m, state.y_m).point.s_m
        self.last_s, self.progress, self.stall_time, self.episode_return = self.start_s, 0.0, 0.0, 0.0
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.reason, self.done = "running", False
        self.last_control = Control()
        self.signals = self.sim.model.evaluate(state, self.last_control).signals
        if self.clearance(state) < 0:
            raise ValueError("Initial vehicle footprint is outside the track")
        if self.recorder:
            self.recorder.logger.info("RL reset seed=%s start_s_m=%s state=%s", seed, self.start_s, state)
            self.sim.recorder.record(0.0, state, self.last_control, self.signals)
        return self.observation(), self.info()

    def clearance(self, state):
        """Approximate vehicle footprint: front/rear contacts and CG, each padded."""
        offsets = (-self.vehicle.vehicle.rear_distance_m, 0, self.vehicle.vehicle.front_distance_m)
        points = [(state.x_m + offset * math.cos(state.heading_rad), state.y_m + offset * math.sin(state.heading_rad))
                  for offset in offsets]
        if hasattr(self.sim.model, "contact_positions"):
            points.extend(self.sim.model.contact_positions(state))
        lateral = [abs(self.track.project(x, y).lateral_m) for x, y in points]
        return self.half_width - max(lateral)

    def track_signals(self, state):
        projection = self.track.project(state.x_m, state.y_m)
        p, half = projection.point, self.track.width_m / 2
        return dict(track_s_m=p.s_m, track_lateral_m=projection.lateral_m,
                    track_heading_error_rad=wrap_angle(state.heading_rad - p.heading_rad),
                    track_curvature_pm=p.curvature_pm, track_clearance_m=self.clearance(state),
                    track_center_x_m=p.x_m, track_center_y_m=p.y_m,
                    track_left_x_m=p.x_m - half * math.sin(p.heading_rad), track_left_y_m=p.y_m + half * math.cos(p.heading_rad),
                    track_right_x_m=p.x_m + half * math.sin(p.heading_rad), track_right_y_m=p.y_m - half * math.cos(p.heading_rad),
                    lap_progress_m=self.progress + signed_progress(self.last_s, p.s_m, self.track.length_m),
                    policy_longitudinal=float(self.previous_action[0]), policy_steer=float(self.previous_action[1]))

    def observation(self):
        s, c = self.sim.state, self.c
        p = self.track.project(s.x_m, s.y_m)
        angle = wrap_angle(s.heading_rad - p.point.heading_rad)
        obs = [s.vx_mps / c["speed_scale_mps"], s.vy_mps / c["lateral_speed_scale_mps"],
               s.yaw_rate_radps / c["yaw_rate_scale_radps"], s.steer_rad / self.max_steer,
               s.throttle, s.brake, s.soc, p.lateral_m / self.half_width,
               math.sin(angle), math.cos(angle), (self.half_width - p.lateral_m) / self.half_width,
               (self.half_width + p.lateral_m) / self.half_width, p.point.curvature_pm * c["curvature_scale_m"],
               self.progress / self.track.length_m, self.sim.time_s / c["max_episode_s"],
               self.stall_time / c["stall_timeout_s"], *self.previous_action]
        for distance in c["preview_distances_m"]:
            target = self.track.sample(p.point.s_m + distance)
            dx, dy = target.x_m - s.x_m, target.y_m - s.y_m
            obs.extend([(dx * math.cos(s.heading_rad) + dy * math.sin(s.heading_rad)) / distance,
                        (-dx * math.sin(s.heading_rad) + dy * math.cos(s.heading_rad)) / distance,
                        target.curvature_pm * c["curvature_scale_m"]])
        if self.roll_pitch:
            v, d = self.vehicle.vehicle, self.vehicle.dynamics
            obs.extend([s.roll_rad / math.radians(v.max_lean_deg), s.roll_rate_radps / c["yaw_rate_scale_radps"],
                        s.pitch_rad / math.radians(d.max_pitch_deg), s.pitch_rate_radps / c["yaw_rate_scale_radps"],
                        s.heave_m / d.max_suspension_displacement_m, s.heave_rate_mps / c["lateral_speed_scale_mps"],
                        s.front_wheel_radps * v.wheel_radius_m / c["speed_scale_mps"],
                        s.rear_wheel_radps * v.wheel_radius_m / c["speed_scale_mps"]])
        result = np.asarray(obs, dtype=np.float32)
        if not np.isfinite(result).all():
            raise FloatingPointError("Non-finite RL observation")
        return result

    def info(self):
        return dict(is_success=self.reason == "lap_completed", termination_reason=self.reason,
                    lap_time_s=self.sim.time_s if self.reason == "lap_completed" else None,
                    elapsed_s=self.sim.time_s, progress_m=self.progress,
                    completion_fraction=clip(self.progress / self.track.length_m, 0, 1),
                    episode_return=self.episode_return, track_clearance_m=self.clearance(self.sim.state),
                    telemetry=dict(time_s=self.sim.time_s, **asdict(self.sim.state), **self.signals,
                                   **self.track_signals(self.sim.state),
                                   **{f"command_{key}": value for key, value in asdict(self.last_control).items()}),
                    **{key: value for key, value in self.track_signals(self.sim.state).items() if key != "track_clearance_m"})

    def step(self, action):
        if self.done:
            raise RuntimeError("Episode ended; call reset() before step()")
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (2,) or not np.isfinite(action).all() or np.any(np.abs(action) > 1 + 1e-6):
            raise ValueError("Action must be [longitudinal, steer] in [-1, 1]")
        action = np.clip(action, -1, 1)
        change = float(np.sum((action - self.previous_action)**2))
        self.previous_action = action.copy()
        longitudinal, steer = map(float, action)
        self.last_control = Control(throttle=max(0, longitudinal), brake=max(0, -longitudinal), steer_rad=steer * self.max_steer)
        start_time, start_progress = self.sim.time_s, self.progress
        end = min(start_time + self.vehicle.simulation.control_dt_s, self.c["max_episode_s"])
        terminated = truncated = False
        while self.sim.time_s < end - 1e-10:
            old_distance = self.sim.state.distance_m
            dt = min(self.vehicle.simulation.dt_s, end - self.sim.time_s)
            old_time = self.sim.time_s
            state, self.signals = self.sim.step(self.last_control, dt)
            projection = self.track.project(state.x_m, state.y_m)
            delta = signed_progress(self.last_s, projection.point.s_m, self.track.length_m)
            # Reject branch jumps before they can earn progress or a spurious lap.
            valid_progress = abs(delta) <= 2 * (state.distance_m - old_distance) + 1e-3
            if valid_progress:
                self.progress += delta
            self.last_s = projection.point.s_m
            elapsed = self.sim.time_s - old_time
            self.stall_time = self.stall_time + elapsed if state.vx_mps < self.c["stall_speed_mps"] else 0.0
            if self.clearance(state) < 0:
                self.reason = "off_track"
            elif not valid_progress:
                self.reason = "invalid_progress"
            elif self.sim.status != "running":
                self.reason = self.sim.status
            elif self.progress < -self.c["backward_limit_m"]:
                self.reason = "wrong_direction"
            elif self.stall_time >= self.c["stall_timeout_s"]:
                self.reason = "stalled"
            elif self.progress >= self.track.length_m:
                self.reason = "lap_completed"
            if self.reason != "running":
                terminated = True
                break
        if not terminated and self.sim.time_s >= self.c["max_episode_s"] - 1e-10:
            self.reason, truncated = "time_limit", True
        dt = self.sim.time_s - start_time
        r = self.reward_config
        edge = max(0, 1 - self.clearance(self.sim.state) / self.half_width - r["edge_start_fraction"]) / (1 - r["edge_start_fraction"])
        terms = dict(progress=r["progress_per_m"] * (self.progress - start_progress), time=-r["time_per_s"] * dt,
                     action_change=-r["action_change"] * change, edge=-r["edge_per_s"] * edge**2 * dt,
                     terminal=r["lap_completed"] if self.reason == "lap_completed" else (-r["failure"] if terminated else 0.0))
        reward = float(sum(terms.values()))
        self.episode_return += reward
        self.done = terminated or truncated
        info = self.info()
        info["reward_terms"] = terms
        return self.observation(), reward, terminated, truncated, info
