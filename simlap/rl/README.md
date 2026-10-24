# Training a motorcycle policy

The policy learns **throttle, braking and steering**. The environment has no autopilot, speed reference, racing-line tracker or imitation demonstrations. It exposes the motorcycle state and local road geometry, including preview points and curvature ahead.

## Install and run

From the repository root (Python 3.9+):

```sh
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[rl]'

# Check the complete pipeline, including gradient updates and saving/loading.
.venv/bin/python scripts/train.py --smoke

# Start a new training run with the default 200,000 control transitions.
.venv/bin/python scripts/train.py
```

Each run gets a unique folder under `training_runs/`. The default algorithm is **SAC**, with a 128×128 MLP actor, twin critics, a replay buffer and continuous actions. Training uses one environment on CPU by default. The vehicle physics step is 5 ms; one policy action is held for the configured control interval (default 50 ms). There are no physics shortcuts in training. Runtime depends heavily on the vehicle model and episode lengths; 200,000 steps is a starting budget, not a convergence guarantee.

`--smoke` uses a **2-second episode horizon** and 256 transitions. It verifies the plumbing and learning updates; its checkpoints are not trained lap-driving policies. Start a fresh normal run for lap training, rather than resuming a smoke checkpoint.

Configuration options:

```sh
.venv/bin/python scripts/train.py --config configs/rl/oval_sac.json --track tracks/oval.json --steps 200000
.venv/bin/python scripts/train.py --vehicle-override configs/nominal.json
# A new run with dynamic roll/pitch, suspension and wheel slip:
.venv/bin/python scripts/train.py --vehicle-override configs/roll_pitch.json
```

`--config` accepts a partial JSON overlay on the default RL settings. Vehicle, track and learning configuration are separate. Set optimizer parameters, network sizes, observation scales, reset distributions, rewards, episode limits and checkpoint/evaluation frequency in `configs/rl/oval_sac.json`.

## Actions and observations

The neural network emits two normalized values in `[-1,1]`:

| Output | Mapping |
| --- | --- |
| `action[0] >= 0` | TPS = action; brake = 0 |
| `action[0] < 0` | TPS = 0; brake = −action |
| `action[1]` | Steering angle request = action × `max_policy_steer_deg` |

This signed longitudinal action learns both pedals while excluding simultaneous throttle and braking. Actual steering and pedals still pass through the physical actuator dynamics. The default policy steering range is ±12°; the physical limit remains configured separately. Increasing the range changes the learning problem and should start a new run.

Observations include body velocities, yaw rate, actual actuators, battery SOC, track-relative lateral/heading errors, left/right clearance, local curvature, episode progress/time/stall time, and the previous action. For each configured preview distance, the observation includes centerline coordinates in the motorcycle frame and curvature. The default preview reaches 80 m. This is knowledge of road geometry, **not a desired time-indexed trajectory or target speed**. The policy is free to choose its line inside the road.

Inputs are scaled using configured physical scales; no external running-normalization state is required. The exact ordered observation names and action convention are saved with checkpoints and verified when loading.

The [roll/pitch chassis](../physics/README.md) adds eight observations: roll/pitch/heave and their rates, plus front/rear wheel speeds. The policy must learn balance and countersteering; it receives no automatic roll stabilization. Start a new run for this model; planar policies have a different observation interface. The existing SAC settings are not tuned or proven to solve this harder task. Old checkpoints continue to use their saved planar configuration.

## Episode and reward

Training resets at random positions around the track, with initial speed sampled from 3–8 m/s by default and small lateral/heading perturbations. These are rolling starts; standing-start learning can be enabled by changing the speed range. Random starts expose both corners early. An episode requires one **complete lap measured from its own start**, not merely crossing the global start line.

The reward is:

```
progress_per_m * signed_progress
- time_per_s * elapsed_seconds
- action_change * squared_action_change
- edge_per_s * edge_cost * elapsed_seconds
+ lap_completed_bonus OR failure_penalty
```

For completed laps, total progress is nearly fixed, so the time term favors faster completion. The small action-change and edge terms regularize the task; the discounted RL objective is a surrogate, not a mathematical guarantee of minimum lap time. There is **no target-speed or centerline-tracking reward**. Signed progress is unwrapped at every physics step, preventing start-line and reverse/forward accumulation errors.

Track limits are checked at **every physics step**, including between policy actions. CG and front/rear contact positions must stay within the track after allowing for the configured half-width and safety margin. This is an approximate footprint, not full body/lean collision geometry.

Leaving the track, excessive lean, wheel lift, battery limits, prolonged stalling, significant backward travel and invalid projection jumps terminate the episode with a failure cost. Successful lap completion terminates with a bonus. The episode time limit is a Gymnasium **truncation**, allowing SAC to bootstrap correctly. Success is checked after track and model limits, so a crash at the finish cannot earn a completed lap.

## Evaluate and inspect

```sh
.venv/bin/python scripts/evaluate.py --checkpoint training_runs/YOUR_RUN/best
# The same policy evaluation, named lap_time in the maneuver list:
.venv/bin/python scripts/lap_time.py --checkpoint training_runs/YOUR_RUN/best
.venv/bin/python -m simlap.dashboard
```

For the lap-time maneuver, refresh the dashboard page and select the newest `lap_time` run. The run header shows **Lap time** for a completed lap; a failed attempt shows **Lap not completed** and its completion percentage. The full metrics are also under **Files, configuration and run metrics**. This is one deterministic rolling-start lap, using the checkpoint's saved track/physics and the lower configured starting speed; it is not the three-speed training evaluation average. Every physics-step sample is recorded for plotting speed, controls, forces, track position and lap progress.

Checkpoint evaluation uses the saved vehicle, track, observation and action configuration. A telemetry run appears in `results/` as `rl_policy`, with full physics-step CSV and engine logging, track error/clearance/progress channels and requested actions. In the dashboard, select this run, plot `y_m` against `x_m`, and enable Equal XY for the driven path. Compare TPS, brake, steering, speed and track clearance in time plots. The run also contains a `geometry/geometry.svg` track plan.

Periodic training evaluations are deterministic, start from the same track position and use a fixed sweep over the configured initial-speed range. The best policy is selected by **success rate first**, then completed-lap time; before any successes, mean progress breaks ties. `best` means best observed checkpoint, and can still be a failing policy. Inspect `summary.json` or `best/manifest.json` rather than assuming it completes a lap.

To verify the physical task independently:

```sh
.venv/bin/python scripts/evaluate.py --baseline
```

This runs a separate conservative geometric controller at approximately 9 m/s. It is only a benchmark; the RL environment never calls it or uses its actions. Do not report benchmark laps as learned-policy performance.

## Checkpointing and logs

Press **Ctrl+C** once to save a final checkpoint and stop. Resume with an additional training budget:

```sh
.venv/bin/python scripts/train.py --resume training_runs/YOUR_RUN/final --steps 100000
```

Resuming loads model/optimizer state and the replay buffer when present, with saved vehicle/track/settings. It starts fresh environment episodes; it is not a bit-for-bit restoration of environment/RNG state. Without a replay buffer, the trainer collects fresh warmup transitions. Checkpoints are local Python/PyTorch artifacts; load your own trusted checkpoints.

| Output | Contents |
| --- | --- |
| `transitions.csv` | Every policy step: all endpoint physical telemetry, track state, actual/requested controls, action, reward components, termination flags and episode index |
| `episodes.csv` | Return, duration, completion, lap time, success and termination reason |
| `optimizer/progress.csv`, `optimizer/progress.json` | SAC losses, entropy coefficient and optimizer metrics |
| `evaluations.jsonl` | Fixed evaluation suite results over training |
| `training.log` | Training progress, evaluations, interruptions and exceptions |
| `vehicle.json`, `track.json`, `settings.json` | Resolved configuration snapshots |
| `versions.json`, `provenance.json` | Dependency versions, code hashes, Python/platform and Git revision |
| `best/` | Best evaluated policy, configuration and evaluation metrics |
| `checkpoints/step_*/` | Periodic policy snapshots, optional replay buffers and configuration |
| `final/` | Last policy, optional replay buffer and configuration; also saved on interruption |
| `summary.json` | Training/update counts and final/best evaluation results |

Training logs all control-rate endpoints; storing every RK4 physics step for hundreds of thousands of actions would create much larger files. Evaluation uses the full existing physics recorder. Replay buffers can be large; disable `logging.save_replay_buffer` or increase checkpoint intervals to reduce storage.

## Architecture

```
tracks/                  Geometry definitions and generated coordinates
configs/rl/              Learning/environment settings
simlap/physics/          Chassis, tires, battery and powertrain
simlap/tracks/           Geometry interfaces, projection and export
simlap/controllers/      Independent benchmark controllers
simlap/rl/environment.py Gymnasium adapter and task/reward definition
simlap/rl/train.py       SAC training, logging and periodic evaluation
simlap/rl/evaluation.py  Deterministic evaluation suite and model ranking
simlap/rl/evaluate.py    Standalone policy/benchmark telemetry runs
simlap/rl/checkpoints.py Configuration-aware policy snapshots
```

The physics engine remains usable without RL dependencies. Old physical-component imports are preserved by small compatibility modules. A new chassis can be injected through `MotorcycleEnv(..., model=...)`; a new track implements the `Track` interface. A richer model will generally require revising observation/state mapping and retraining.

API references: [Gymnasium environment contract](https://gymnasium.farama.org/api/env/), [SAC implementation](https://stable-baselines3.readthedocs.io/en/v2.6.0/modules/sac.html), [termination versus truncation](https://farama.org/Gymnasium-Terminated-Truncated-Step-API).
