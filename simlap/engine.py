"""Fixed output/control intervals with selectable internal Runge–Kutta solvers."""

import math
import time
from dataclasses import astuple, replace

from .physics.factory import make_model
from .integration import SolverStats, integrate
from .types import Control, DynamicsModel, State


class Simulator:
    def __init__(self, config, model: DynamicsModel = None, recorder=None):
        self.config = config
        self.model = model or make_model(config)
        self.recorder = recorder
        self.reset()

    def reset(self, state=None):
        state = state or State(soc=self.config.battery.initial_soc)
        if hasattr(self.model, "initial_state"):
            state = self.model.initial_state(state)
        if not all(math.isfinite(x) for x in astuple(state)) or state.vx_mps < 0:
            raise ValueError("Initial state must be finite and forward-moving")
        if not self.config.battery.min_soc <= state.soc <= self.config.battery.max_soc:
            raise ValueError("Initial SOC is outside configured bounds")
        if not 0 <= state.throttle <= 1 or not 0 <= state.brake <= 1:
            raise ValueError("Initial actuators must be in [0, 1]")
        if abs(state.steer_rad) > math.radians(self.config.actuators.max_steer_deg):
            raise ValueError("Initial steering exceeds actuator limit")
        self.state, self.time_s, self.steps = state, 0.0, 0
        self.status = "running"
        self.solver_stats = SolverStats()
        return self.state

    def _integrate(self, state, control, dt):
        result, stats = integrate(lambda value: self.model.evaluate(value, control).derivative,
                                  state, dt, self.config.simulation)
        self.solver_stats.accepted += stats.accepted
        self.solver_stats.rejected += stats.rejected
        self.solver_stats.evaluations += stats.evaluations
        self.solver_stats.max_accepted_error = max(self.solver_stats.max_accepted_error, stats.max_accepted_error)
        return result

    def step(self, control: Control, dt_s=None):
        if self.status != "running":
            raise RuntimeError(f"Simulation terminated: {self.status}; reset before stepping")
        dt = self.config.simulation.dt_s if dt_s is None else dt_s
        if not math.isfinite(dt) or not 0 < dt <= self.config.simulation.dt_s * (1 + 1e-9):
            raise ValueError("step dt must be positive and no larger than configured physics dt")
        start = time.perf_counter()
        self.solver_stats = SolverStats()
        state = self._integrate(self.state, control, dt)
        b = self.config.battery
        if state.soc < b.min_soc or state.soc > b.max_soc:
            # Locate the SOC boundary instead of discarding charge by clipping a full step.
            boundary = b.min_soc if state.soc < b.min_soc else b.max_soc
            lo, hi = 0.0, dt
            for _ in range(40):
                mid = (lo + hi) / 2
                candidate = self._integrate(self.state, control, mid)
                if b.min_soc <= candidate.soc <= b.max_soc:
                    lo = mid
                else:
                    hi = mid
            dt = hi
            state = replace(self._integrate(self.state, control, dt), soc=boundary)
            self.status = "battery_soc_boundary"
        state = replace(state, vx_mps=max(0, state.vx_mps))
        if not all(math.isfinite(x) for x in astuple(state)):
            raise FloatingPointError("Non-finite state; reduce timestep or inspect model parameters")
        self.state = state
        self.time_s += dt
        self.steps += 1
        evaluation = self.model.evaluate(state, control)
        if evaluation.signals.get("wheel_lift", 0):
            self.status = "wheel_lift_model_limit"
        if evaluation.signals.get("lean_limit_exceeded", 0):
            self.status = "lean_model_limit"
        if evaluation.signals.get("pitch_limit_exceeded", 0):
            self.status = "pitch_model_limit"
        if evaluation.signals.get("suspension_limit_exceeded", 0):
            self.status = "suspension_model_limit"
        evaluation.signals.update(integrator_accepted_steps=self.solver_stats.accepted,
                                  integrator_rejected_steps=self.solver_stats.rejected,
                                  integrator_rhs_evaluations=self.solver_stats.evaluations,
                                  integrator_error_norm=self.solver_stats.max_accepted_error)
        elapsed = (time.perf_counter() - start) * 1000
        if self.recorder:
            self.recorder.record(self.time_s, state, control, evaluation.signals, elapsed)
            self.recorder.logger.debug("step=%d t=%.6f dt=%.6f solver=%s accepted=%d rejected=%d evaluations=%d error=%.3g cpu_ms=%.3f status=%s",
                                       self.steps, self.time_s, dt, self.config.simulation.integrator.upper(),
                                       self.solver_stats.accepted, self.solver_stats.rejected, self.solver_stats.evaluations,
                                       self.solver_stats.max_accepted_error, elapsed, self.status)
        return state, evaluation.signals

    def advance(self, control: Control, duration_s=None):
        """Hold one action for a control interval, retaining every physics-step log."""
        duration = self.config.simulation.control_dt_s if duration_s is None else duration_s
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration_s must be positive and finite")
        end = self.time_s + duration
        signals = {}
        while self.time_s < end - 1e-12 and self.status == "running":
            _, signals = self.step(control, min(self.config.simulation.dt_s, end - self.time_s))
        return self.state, signals
