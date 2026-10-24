"""Deterministic evaluations with explicit success/lap-time metrics."""

import statistics


def evaluate(model, env, episodes=3, seed=1000):
    results = []
    for index in range(episodes):
        low, high = env.c["initial_speed_mps"]
        speed = low + (high - low) * index / max(1, episodes - 1)
        observation, info = env.reset(seed=seed + index, options={"start_s_m": 0.0, "speed_mps": speed})
        while not env.done:
            action, _ = model.predict(observation, deterministic=True)
            observation, _, _, _, info = env.step(action)
        results.append({key: info[key] for key in ("is_success", "lap_time_s", "elapsed_s", "completion_fraction", "episode_return", "termination_reason")})
        results[-1]["initial_speed_mps"] = speed
    laps = [r["lap_time_s"] for r in results if r["is_success"]]
    return dict(success_rate=len(laps) / episodes, mean_lap_time_s=statistics.mean(laps) if laps else None,
                mean_completion=statistics.mean(r["completion_fraction"] for r in results), episodes=results)


def rank(metrics):
    """Prefer finishing reliably; compare lap times only among equal success rates."""
    return (metrics["success_rate"], -metrics["mean_lap_time_s"] if metrics["mean_lap_time_s"] is not None else metrics["mean_completion"])
