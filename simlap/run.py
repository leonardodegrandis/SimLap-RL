"""CLI shared by the three maneuver entry scripts."""

import argparse
import json
from datetime import datetime, timezone
from dataclasses import replace
from pathlib import Path

from .config import load_config
from .engine import Simulator
from .maneuvers import load_maneuvers, prepare
from .telemetry import Recorder


def run_maneuver(name, config, parameters, output="results"):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    directory = Path(output) / f"{stamp}_{name}"
    recorder = Recorder(directory, config, name, parameters)
    try:
        simulator = Simulator(config, recorder=recorder)
        recorder.logger.info("models chassis=%s tires=%s powertrain=%s battery=%s integrator=%s",
                             type(simulator.model).__name__, type(simulator.model.tires).__name__,
                             type(simulator.model.powertrain).__name__, type(simulator.model.powertrain.battery).__name__,
                             config.simulation.integrator.upper())
        state, control, extra = prepare(name, simulator.model, config, parameters)
        state = simulator.reset(state)
        trim_control, next_control = control, 0.0
        balance = name == "cornering" and hasattr(state, "roll_rad") and (parameters.get("roll_kp", 0) or parameters.get("roll_kd", 0))
        extra["roll_balance_controller"] = bool(balance)
        recorder.logger.info("initial state=%s control=%s trim=%s", state, control, extra)
        recorder.record(0.0, state, control, simulator.model.evaluate(state, control).signals)
        duration = parameters["duration_s"]
        reason = "duration_completed"
        while simulator.time_s < duration - 1e-10:
            if name == "braking" and simulator.state.vx_mps <= config.simulation.stop_speed_mps:
                reason = "stopped"
                break
            if balance and simulator.time_s >= next_control - 1e-10:
                steer = (trim_control.steer_rad + parameters.get("roll_kp", 0) * (simulator.state.roll_rad - state.roll_rad)
                         + parameters.get("roll_kd", 0) * simulator.state.roll_rate_radps)
                control = replace(trim_control, steer_rad=steer)
                next_control += config.simulation.control_dt_s
            simulator.step(control, min(config.simulation.dt_s, duration - simulator.time_s))
            if simulator.status != "running":
                reason = simulator.status
                break
        if name == "braking" and simulator.state.vx_mps <= config.simulation.stop_speed_mps:
            reason = "stopped"
        summary = recorder.finish(reason, extra)
        return directory, summary
    except Exception:
        recorder.logger.exception("Simulation failed")
        raise
    finally:
        recorder.close()


def main(maneuver=None):
    parser = argparse.ArgumentParser(description="Electric motorcycle maneuver simulator")
    if maneuver is None:
        parser.add_argument("maneuver", choices=("acceleration", "braking", "cornering", "all"))
    parser.add_argument("--config", help="Complete vehicle JSON configuration")
    parser.add_argument("--override", action="append", default=[], help="JSON overlay; repeatable")
    parser.add_argument("--maneuvers", help="Maneuver JSON configuration")
    parser.add_argument("--output", default="results", help="Output directory")
    args = parser.parse_args()
    try:
        config = load_config(args.config, args.override)
        maneuvers = load_maneuvers(args.maneuvers)
        selected = maneuver or args.maneuver
        names = maneuvers if selected == "all" else [selected]
        failed = False
        for name in names:
            directory, summary = run_maneuver(name, config, maneuvers[name], args.output)
            print(json.dumps(dict(directory=str(directory), **summary), indent=2))
            failed |= summary["reason"] not in ("duration_completed", "stopped")
        if failed:
            raise SystemExit(1)
    except (ValueError, OSError) as error:
        parser.exit(2, f"Simulation error: {error}\n")


if __name__ == "__main__":
    main()
