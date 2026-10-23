"""Fixed-step RK4 driver; controllers and storage stay outside physics modules."""

import math
import time
from dataclasses import astuple, replace

from .dynamics import PlanarMotorcycle
from .types import Control, DynamicsModel, State


class Simulator:
    def __init__(self, config, model: DynamicsModel = None, recorder=None):
        self.config = config
        self.model = model or PlanarMotorcycle(config)
        self.recorder = recorder
        self.reset()

    def reset(self, state=None):
        state = state or State(soc=self.config.battery.initial_soc)
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
        return self.state

    def _integrate(self, state, control, dt):
        evaluate = self.model.evaluate
        k1 = evaluate(state, control).derivative
        k2 = evaluate(state.shifted(k1, dt / 2), control).derivative
        k3 = evaluate(state.shifted(k2, dt / 2), control).derivative
        k4 = evaluate(state.shifted(k3, dt), control).derivative
        average = type(state)(*((a + 2*b + 2*c + d) / 6 for a, b, c, d in
                          zip(astuple(k1), astuple(k2), astuple(k3), astuple(k4))))
        return state.shifted(average, dt)

    def step(self, control: Control, dt_s=None):
        if self.status != "running":
            raise RuntimeError(f"Simulation terminated: {self.status}; reset before stepping")
        dt = self.config.simulation.dt_s if dt_s is None else dt_s
        if not math.isfinite(dt) or not 0 < dt <= self.config.simulation.dt_s * (1 + 1e-9):
            raise ValueError("step dt must be positive and no larger than configured physics dt")
        start = time.perf_counter()
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
        elapsed = (time.perf_counter() - start) * 1000
        if self.recorder:
            self.recorder.record(self.time_s, state, control, evaluation.signals, elapsed)
            self.recorder.logger.debug("step=%d t=%.6f dt=%.6f solver=RK4 stages=4 cpu_ms=%.3f status=%s",
                                       self.steps, self.time_s, dt, elapsed, self.status)
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
