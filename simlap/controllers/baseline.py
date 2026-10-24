"""Conservative benchmark, never called by the RL environment or policy."""

import math

from ..mathutils import clip


class GeometricBaseline:
    def __init__(self, env, speed_mps=9.0):
        if speed_mps <= 0:
            raise ValueError("Baseline speed must be positive")
        self.env, self.speed_mps = env, speed_mps

    def __call__(self, observation):
        env, state = self.env, self.env.sim.state
        position = env.track.project(state.x_m, state.y_m)
        lookahead = 4.0 + 0.5 * state.vx_mps
        point = env.track.sample(position.point.s_m + lookahead)
        dx, dy = point.x_m - state.x_m, point.y_m - state.y_m
        lateral = -dx * math.sin(state.heading_rad) + dy * math.cos(state.heading_rad)
        wheelbase = env.vehicle.vehicle.front_distance_m + env.vehicle.vehicle.rear_distance_m
        steer = math.atan2(2 * wheelbase * lateral, dx**2 + dy**2)
        demand = clip(0.25 * (self.speed_mps - state.vx_mps) + 0.035, -1, 1)
        return [demand, clip(steer / env.max_steer, -1, 1)]
