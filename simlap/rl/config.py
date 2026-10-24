"""Validated RL settings, separated from vehicle and track parameters."""

import json
import math
from pathlib import Path

from ..config import ROOT, merge


def load_settings(path=None):
    defaults = json.loads((ROOT / "configs/rl/oval_sac.json").read_text())
    settings = merge(defaults, json.loads(Path(path).read_text())) if path else defaults
    positive = {
        "environment": ["max_episode_s", "stall_timeout_s", "stall_speed_mps", "backward_limit_m", "speed_scale_mps",
                        "lateral_speed_scale_mps", "yaw_rate_scale_radps", "curvature_scale_m", "max_policy_steer_deg"],
        "reward": ["progress_per_m", "time_per_s", "failure", "lap_completed"],
        "sac": ["learning_rate", "buffer_size", "batch_size", "gamma", "tau", "train_freq", "gradient_steps", "torch_threads"],
        "logging": ["eval_every_steps", "eval_episodes", "checkpoint_every_steps", "console_every_steps"],
    }
    def number(value, minimum=0, strict=False):
        return type(value) in (float, int) and math.isfinite(value) and (value > minimum if strict else value >= minimum)

    for section, keys in positive.items():
        for key in keys:
            if not number(settings[section][key], strict=True):
                raise ValueError(f"{section}.{key} must be finite and positive")
    integers = [(settings, "seed"), (settings, "total_timesteps")]
    integers += [(settings["sac"], key) for key in ("buffer_size", "learning_starts", "batch_size", "train_freq", "gradient_steps", "torch_threads")]
    integers += [(settings["logging"], key) for key in ("eval_every_steps", "eval_episodes", "checkpoint_every_steps", "console_every_steps")]
    if any(type(obj[key]) is not int or obj[key] < 0 for obj, key in integers) or settings["total_timesteps"] == 0:
        raise ValueError("Step counts, seed and thread counts must be valid integers")
    env, reward, sac = settings["environment"], settings["reward"], settings["sac"]
    for key in ("initial_lateral_m", "initial_heading_rad", "vehicle_half_width_m", "boundary_margin_m"):
        if not number(env[key]):
            raise ValueError(f"environment.{key} must be finite and nonnegative")
    for key in ("action_change", "edge_per_s"):
        if not number(reward[key]):
            raise ValueError(f"reward.{key} must be finite and nonnegative")
    speeds = env["initial_speed_mps"]
    if not isinstance(speeds, list) or len(speeds) != 2 or not all(number(v) for v in speeds) or speeds[0] > speeds[1]:
        raise ValueError("initial_speed_mps must be an ordered nonnegative [min, max]")
    preview = env["preview_distances_m"]
    if not isinstance(preview, list) or not preview or not all(number(v, strict=True) for v in preview) or any(a >= b for a, b in zip(preview, preview[1:])):
        raise ValueError("preview_distances_m must be strictly increasing and positive")
    if not 0 <= reward["edge_start_fraction"] < 1 or not 0 < sac["gamma"] <= 1 or not 0 < sac["tau"] <= 1:
        raise ValueError("Invalid edge threshold, gamma or tau")
    if type(env["random_start"]) is not bool or type(settings["logging"]["save_replay_buffer"]) is not bool:
        raise ValueError("random_start and save_replay_buffer must be boolean")
    if not isinstance(sac["net_arch"], list) or not sac["net_arch"] or any(type(v) is not int or v < 1 for v in sac["net_arch"]):
        raise ValueError("net_arch must contain positive layer sizes")
    if sac["batch_size"] > sac["buffer_size"]:
        raise ValueError("batch_size exceeds replay buffer capacity")
    return settings
