"""Full-resolution CSV, engine trace, reproducibility metadata and UI samples."""

import csv
import hashlib
import json
import logging
import math
import platform
import subprocess
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from .config import ROOT


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def provenance():
    """Identify the code and runtime behind a simulation or training run."""
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, stderr=subprocess.DEVNULL, text=True).strip()
    except (subprocess.SubprocessError, OSError):
        commit = None
    sources = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted((ROOT / "simlap").rglob("*.py"))}
    return dict(python=platform.python_version(), platform=platform.platform(), git_commit=commit, source_sha256=sources)


class Recorder:
    def __init__(self, directory, config, maneuver, parameters):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.config = config
        self.rows = []
        self.csv_file = (self.directory / "telemetry.csv").open("w", newline="")
        self.writer = None
        self.logger = logging.getLogger(f"simlap.{self.directory.name}")
        self.logger.setLevel(config.simulation.log_level)
        self.logger.propagate = False
        self.handler = logging.FileHandler(self.directory / "engine.log")
        self.handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        self.logger.addHandler(self.handler)
        resolved = config.to_dict()
        write_json(self.directory / "config.json", resolved)
        self.metadata = dict(maneuver=maneuver, parameters=parameters,
                             created_utc=datetime.now(timezone.utc).isoformat(),
                             **provenance(),
                             config_sha256=hashlib.sha256(json.dumps(resolved, sort_keys=True).encode()).hexdigest())
        write_json(self.directory / "metadata.json", self.metadata)
        self.logger.info("start maneuver=%s parameters=%s config_sha256=%s", maneuver, parameters, self.metadata["config_sha256"])

    def record(self, time_s, state, control, signals, cpu_ms=0.0):
        solver = dict(integrator_accepted_steps=0, integrator_rejected_steps=0,
                      integrator_rhs_evaluations=0, integrator_error_norm=0.0)
        row = dict(time_s=time_s, **asdict(state), **dict(solver, **signals), step_cpu_ms=cpu_ms)
        row.update({f"command_{key}": value for key, value in asdict(control).items()})
        if not all(value is None or math.isfinite(value) for value in row.values()):
            raise FloatingPointError("Non-finite telemetry")
        if self.writer is None:
            self.writer = csv.DictWriter(self.csv_file, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.rows.append(row)
        if signals.get("load_transfer_iterations", 0) >= self.config.simulation.load_transfer_iterations and abs(signals.get("load_transfer_residual_n", 0)) > self.config.simulation.load_transfer_tolerance_n:
            self.logger.warning("t=%.6f load-transfer did not converge residual_n=%.6f", time_s, signals["load_transfer_residual_n"])

    def finish(self, reason, extra=None):
        rows, first, last = self.rows, self.rows[0], self.rows[-1]
        summary = dict(
            run_id=self.directory.name, maneuver=self.metadata["maneuver"], reason=reason,
            duration_s=last["time_s"], samples=len(rows), distance_m=last["distance_m"] - first["distance_m"],
            max_speed_kph=max(row["speed_mps"] for row in rows) * 3.6,
            final_speed_kph=last["speed_mps"] * 3.6,
            peak_shaft_power_kw=max(row["motor_shaft_power_w"] for row in rows) / 1000,
            peak_discharge_kw=max(row["battery_terminal_power_w"] for row in rows) / 1000,
            peak_regen_kw=max(0, -min(row["battery_terminal_power_w"] for row in rows)) / 1000,
            initial_soc=first["soc"], final_soc=last["soc"],
            chemical_energy_wh=(last["chemical_energy_j"] - first["chemical_energy_j"]) / 3600,
            terminal_energy_wh=(last["terminal_energy_j"] - first["terminal_energy_j"]) / 3600,
            battery_heat_wh=(last["battery_heat_j"] - first["battery_heat_j"]) / 3600,
            max_power_balance_error_w=max(abs(row["power_balance_error_w"]) for row in rows),
            max_load_transfer_residual_n=max(abs(row["load_transfer_residual_n"]) for row in rows),
            engine_cpu_s=sum(row["step_cpu_ms"] for row in rows) / 1000,
        )
        target = 100 / 3.6
        summary["zero_to_100_s"] = None
        if first["speed_mps"] < 0.1 and self.metadata["maneuver"] == "acceleration":
            for left, right in zip(rows, rows[1:]):
                if left["speed_mps"] < target <= right["speed_mps"]:
                    fraction = (target - left["speed_mps"]) / (right["speed_mps"] - left["speed_mps"])
                    summary["zero_to_100_s"] = left["time_s"] + fraction * (right["time_s"] - left["time_s"])
                    break
        summary["stopping_distance_m"] = summary["distance_m"] if reason == "stopped" else None
        if self.metadata["maneuver"] == "cornering":
            radius = self.metadata["parameters"]["radius_m"]
            summary["max_radius_error_m"] = max(abs(math.hypot(row["x_m"], row["y_m"] - radius) - abs(radius)) for row in rows)
            summary["max_speed_error_mps"] = max(abs(row["speed_mps"] - self.metadata["parameters"]["speed_mps"]) for row in rows)
        summary.update(extra or {})
        write_json(self.directory / "summary.json", summary)
        limit = self.config.simulation.dashboard_max_points
        indices = sorted({round(i * (len(rows) - 1) / (min(limit, len(rows)) - 1)) for i in range(min(limit, len(rows)))}) if len(rows) > 1 else [0]
        write_json(self.directory / "dashboard.json", dict(summary=summary, metadata=self.metadata,
                   config=self.config.to_dict(), columns=list(first), rows=[rows[i] for i in indices]))
        descriptions = {
            "ax_mps2": "Body-frame longitudinal acceleration (force / mass), not dvx/dt",
            "ay_mps2": "Body-frame lateral acceleration (force / mass), not dvy/dt",
            "lean_estimate_deg": "Quasi-steady atan(ay/g); distinct from actual roll_deg in the roll/pitch model",
            "front_fx_n": "Front tire longitudinal force in the steered wheel frame",
            "front_fy_n": "Front tire lateral force in the steered wheel frame",
            "roll_rad": "Actual roll angle, positive leaning left",
            "pitch_rad": "Actual pitch angle, positive nose up",
            "heave_m": "Suspension heave from static equilibrium; CG height = h*cos(roll) + heave",
            "front_slip_ratio": "(wheel rolling speed - contact longitudinal speed) / regularized speed",
            "integrator_error_norm": "Largest accepted normalized embedded error; zero for fixed RK4",
            "chemical_energy_j": "Signed integral of OCV * current; positive means consumed",
            "battery_terminal_power_w": "Positive discharge, negative charging, includes auxiliaries",
            "power_balance_error_w": "Chemical power minus wheel power, all electrical/driveline losses and auxiliaries",
            "step_cpu_ms": "Wall time of physics step, excludes recording and file I/O",
            "command_yaw_rate_radps": "Null when using direct steering; otherwise adapter target",
        }
        write_json(self.directory / "channels.json", {key: descriptions.get(key, "SI units indicated by suffix; dimensionless if no suffix") for key in first})
        self.logger.info("finish reason=%s summary=%s", reason, summary)
        return summary

    def close(self):
        self.csv_file.close()
        self.handler.close()
        self.logger.removeHandler(self.handler)
