"""Self-contained policy snapshots with settings and optional replay state."""

import json
from pathlib import Path

from ..telemetry import write_json

ACTION_MAPPING = "v1: action[0]>=0 -> throttle; action[0]<0 -> brake; action[1] -> steering angle"


def save_checkpoint(model, directory, vehicle, track, settings, observation_names, replay=False, metrics=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    model.save(directory / "model")
    if replay:
        model.save_replay_buffer(directory / "replay_buffer.pkl")
    write_json(directory / "vehicle.json", vehicle.to_dict())
    write_json(directory / "track.json", track.to_dict())
    write_json(directory / "settings.json", settings)
    write_json(directory / "manifest.json", dict(training_steps=model.num_timesteps, observation_names=observation_names,
               action_mapping=ACTION_MAPPING, evaluation=metrics, replay_saved=replay))


def check_interface(directory, observation_names):
    manifest = json.loads((Path(directory) / "manifest.json").read_text())
    if manifest["observation_names"] != observation_names or manifest["action_mapping"] != ACTION_MAPPING:
        raise ValueError("Checkpoint observation/action interface differs from the current environment")
    return manifest
