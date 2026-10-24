"""Run a trained policy or benchmark and export a lap into the telemetry viewer."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from ..config import load_config
from ..controllers.baseline import GeometricBaseline
from ..telemetry import Recorder, write_json
from ..tracks import load_track
from ..tracks.export import export_track
from .checkpoints import check_interface
from .config import load_settings
from .environment import MotorcycleEnv


def main(maneuver=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--checkpoint", help="Directory containing model.zip and config snapshots")
    source.add_argument("--baseline", action="store_true", help="Conservative geometric benchmark, not an RL policy")
    parser.add_argument("--config", help="RL settings for baseline only")
    parser.add_argument("--vehicle-config")
    parser.add_argument("--track")
    parser.add_argument("--baseline-speed", type=float, default=9.0)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--output", default="results")
    args = parser.parse_args()
    checkpoint = Path(args.checkpoint) if args.checkpoint else None
    if checkpoint and any((args.config, args.vehicle_config, args.track)):
        parser.error("Checkpoint evaluation uses its saved vehicle, track and environment settings")
    vehicle = load_config(checkpoint / "vehicle.json" if checkpoint else args.vehicle_config)
    track = load_track(checkpoint / "track.json" if checkpoint else args.track)
    settings = load_settings(checkpoint / "settings.json" if checkpoint else args.config)
    name = maneuver or ("rl_policy" if checkpoint else "baseline_lap")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    directory = Path(args.output) / f"{stamp}_{name}"
    recorder = Recorder(directory, vehicle, name, dict(track=track.to_dict(), seed=args.seed, checkpoint=str(checkpoint) if checkpoint else None))
    env = MotorcycleEnv(vehicle, track, settings, evaluation=True, recorder=recorder)
    try:
        write_json(directory / "track.json", track.to_dict())
        write_json(directory / "rl_settings.json", settings)
        export_track(track, directory / "geometry")
        if checkpoint:
            import torch
            from stable_baselines3 import SAC
            torch.set_num_threads(settings["sac"]["torch_threads"])
            check_interface(checkpoint, env.observation_names)
            model = SAC.load(checkpoint / "model.zip", device=settings["sac"]["device"])
            policy = lambda observation: model.predict(observation, deterministic=True)[0]
        else:
            policy = GeometricBaseline(env, args.baseline_speed)
        observation, info = env.reset(seed=args.seed)
        while not env.done:
            observation, _, _, _, info = env.step(policy(observation))
        summary = recorder.finish(env.reason, {key: info[key] for key in ("is_success", "lap_time_s", "completion_fraction", "episode_return")})
        print(json.dumps(dict(directory=str(directory), **summary), indent=2))
    except Exception:
        recorder.logger.exception("Lap evaluation failed")
        raise
    finally:
        recorder.close()
        env.close()


if __name__ == "__main__":
    main()
