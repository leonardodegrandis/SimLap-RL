"""Pure-Python RK4 and error-controlled Dormand–Prince 5(4) integration."""

import math
from dataclasses import dataclass

from .types import state_values


@dataclass
class SolverStats:
    accepted: int = 0
    rejected: int = 0
    evaluations: int = 0
    max_accepted_error: float = 0.0


def combine(state, stages, weights, step):
    vectors = [state_values(stage) for stage in stages]
    return type(state)(*(value + step * sum(w * vector[i] for w, vector in zip(weights, vectors))
                         for i, value in enumerate(state_values(state))))


def rk4_step(rhs, state, step):
    k1 = rhs(state)
    k2 = rhs(state.shifted(k1, step / 2))
    k3 = rhs(state.shifted(k2, step / 2))
    k4 = rhs(state.shifted(k3, step))
    return combine(state, [k1, k2, k3, k4], [1/6, 1/3, 1/3, 1/6], step)


# Dormand–Prince tableau. These coefficients define the method, not model parameters.
DP_A = [[], [1/5], [3/40, 9/40], [44/45, -56/15, 32/9],
        [19372/6561, -25360/2187, 64448/6561, -212/729],
        [9017/3168, -355/33, 46732/5247, 49/176, -5103/18656],
        [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84]]
DP_B4 = [5179/57600, 0, 7571/16695, 393/640, -92097/339200, 187/2100, 1/40]


def integrate(rhs, state, duration, settings):
    """Reach exactly the requested output time; controls are held by the caller."""
    if duration <= 0:
        return state, SolverStats()
    stats, elapsed = SolverStats(), 0.0
    step = min(duration, settings.max_internal_step_s)
    for _ in range(settings.max_internal_steps):
        step = min(step, duration - elapsed)
        if settings.integrator == "rk4":
            result = rk4_step(rhs, state, step)
            stats.evaluations += 4
            error = 0.0
        else:
            stages = [rhs(state)]
            for weights in DP_A[1:]:
                stages.append(rhs(combine(state, stages, weights, step)))
            result = combine(state, stages, DP_A[-1], step)
            lower = combine(state, stages, DP_B4, step)
            stats.evaluations += 7
            errors = [(a - b) / (settings.absolute_tolerance + settings.relative_tolerance * max(abs(old), abs(a)))
                      for old, a, b in zip(state_values(state), state_values(result), state_values(lower))]
            error = math.sqrt(sum(e * e for e in errors) / len(errors))
        if not all(math.isfinite(x) for x in state_values(result)) or not math.isfinite(error):
            raise FloatingPointError("Non-finite integration stage; inspect parameters or reduce the maximum internal step")
        if error <= 1:
            state = result
            elapsed += step
            stats.accepted += 1
            stats.max_accepted_error = max(stats.max_accepted_error, error)
            if duration - elapsed <= max(1e-15, duration * 1e-13):
                return state, stats
        else:
            stats.rejected += 1
            if step <= settings.min_internal_step_s * (1 + 1e-12):
                raise RuntimeError("RK45 cannot meet tolerances at min_internal_step_s")
        if settings.integrator == "rk45":
            factor = 5.0 if error == 0 else max(0.1, min(5.0, 0.9 * error**(-0.2)))
            step = min(settings.max_internal_step_s, max(settings.min_internal_step_s, step * factor))
    raise RuntimeError("Integrator exceeded max_internal_steps; inspect stiffness and tolerances")
