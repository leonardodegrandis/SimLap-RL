# SimLap

A small electric motorcycle simulator with modular physics, reproducible maneuver runs, a local dashboard, and optional reinforcement learning. Python **3.9+**. The core physics has no third-party dependencies; learning adds Gymnasium, PyTorch and Stable-Baselines3.

For **throttle/brake/steering training on the oval**, see the [RL guide](simlap/rl/README.md) and [track geometry guide](tracks/README.md). Quick start after installing the optional dependencies:

```sh
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[rl]'
.venv/bin/python scripts/train.py
```

RL settings are in `configs/rl/oval_sac.json`. Track dimensions are in `tracks/oval.json`. The policy receives local road geometry, not a target speed or racing trajectory.

For **dynamic roll, pitch, suspension and wheel/tire slip**, see the [advanced physics guide](simlap/physics/README.md). Run its three example maneuvers with:

```sh
.venv/bin/python -m simlap.run all --override configs/roll_pitch.json --maneuvers configs/maneuvers_roll_pitch.json
```

This selects the new chassis and adaptive RK45. The default remains planar so existing training/checkpoints retain their model.

```sh
python3 -m venv .venv
.venv/bin/python -m simlap.run all
.venv/bin/python -m simlap.dashboard
```

Open **http://127.0.0.1:8000**. The dashboard reads completed runs from `results/`; use Refresh after running another simulation. It plots full-resolution CSV telemetry in configurable panels. It has no external assets, CDN, or Python dependencies.

### Telemetry workspace

- Filter the channel list, select channels, choose a destination plot, and click **Add selected**. Double-click a channel to add it directly. All logged channels are listed, including channels with no numeric samples.
- Use **+ Plot** to create panels. Rename them, move them up/down, choose 1–3 columns, or make a panel full width. Narrow screens stack panels automatically.
- Each plot's legend controls trace visibility, color, left/right Y axis, destination panel, and removal. It shows values at the shared cursor. Units are displayed from channel suffixes; unknown units are marked `—`. Mixed units on one Y axis are explicitly flagged.
- Choose any logged **X** channel. For a trajectory, use `x_m` as X, add `y_m`, and enable **Equal XY**. Equal scaling requires matching axis units and no right-axis traces.
- Click a time plot to position the shared cursor. Drag horizontally to zoom, double-click to fit, or set the time interval numerically. Playback and the slider use the same cursor. XY clicks select the sample nearest the clicked X value; repeated X values can be ambiguous.
- Select a comparison run for dashed overlays; solid lines are the selected run. Runs align at simulation time zero. Separate legend columns show the nearest sample for each run, or `—` outside its time span.
- Layout, channel assignments, axes, colors, visibility and panel order are saved in browser storage. **Reset layout** restores defaults. Full CSV data is loaded into browser memory; very long runs may require a future windowed-data backend.

Individual entry points:

```sh
.venv/bin/python scripts/acceleration.py
.venv/bin/python scripts/braking.py
.venv/bin/python scripts/cornering.py
.venv/bin/python scripts/lap_time.py --checkpoint training_runs/YOUR_RUN/best
.venv/bin/python -m simlap.run all --override configs/nominal.json
.venv/bin/python -m unittest discover -s tests -v
```

`lap_time.py` runs one deterministic lap with the saved policy, physics and track. Select its `lap_time` run in the dashboard to see the lap time in the header and plot the full lap telemetry. Incomplete attempts show their completion percentage instead of a lap time.

## Configuration

Edit [configs/motorcycle.json](configs/motorcycle.json) for the full model and [configs/maneuvers.json](configs/maneuvers.json) for maneuvers. Units appear in parameter names. Mass includes rider. All numbers are **assumptions for development**, not fitted measurements.

| Section | Parameters |
| --- | --- |
| `vehicle` | Mass, yaw inertia, CG location/height, wheel radius, Cd, frontal area, rolling resistance, lean limit |
| `environment` | Gravity, air density, longitudinal headwind relative to the bike |
| `tires` | Friction coefficient, front/rear cornering stiffness, low-speed slip regularization |
| `motor` | Peak shaft power, normalized RPM/power map, nominal multiplier, torque limit, ratio, gearbox efficiency, copper/iron/idle losses, regen limits |
| `inverter` | Conversion efficiency and energized idle loss |
| `battery` | Rated kWh and voltage, SOC/OCV curve, pack resistance, current/voltage/SOC limits, initial SOC, auxiliaries |
| `actuators` | Response times, steering angle/rate limits, brake force/bias, regen switch, low-speed braking regularization, yaw-rate adapter |
| `dynamics` | Chassis selection, roll/pitch and wheel inertias, springs/dampers, longitudinal tire stiffness, pitch/travel limits |
| `simulation` | Physics/control steps, RK4/RK45 method, internal step bounds/tolerances, load-transfer solver settings, logging |

The motor map peaks at **48 kW mechanical shaft power**. Battery power is larger during motoring because of losses. `motor.rating: "nominal"` multiplies the entire torque/power envelope by `motor.nominal_factor` (default 0.75, giving a 36 kW ceiling). This is a selectable map, not a thermally enforced continuous rating. Both map axes are editable. The first power-map segment provides finite launch torque; the final zero-power point sets the speed cutoff. Torque is additionally capped by `max_torque_nm` and pack limits. The planar model also limits torque by tire grip; the roll/pitch model allows wheelspin.

Use `--config path.json` to replace the complete config, or repeat `--override path.json` to merge partial changes in order. Unknown keys and invalid ranges are rejected. For example:

```json
{
  "motor": {"rating": "nominal", "nominal_factor": 0.65},
  "battery": {"capacity_kwh": 10.0, "resistance_ohm": 0.025},
  "tires": {"friction_coefficient": 0.9}
}
```

`--maneuvers path.json` replaces the maneuver configuration; `--output directory` changes the run destination. Point the dashboard to that directory with `--results directory`.

## Model and assumptions

The default chassis is a **forward-only planar 3-DOF single-track model**: longitudinal, lateral and yaw motion. The description below covers this baseline; the selectable [roll/pitch chassis](simlap/physics/README.md) adds suspension and wheel dynamics with a different tire law. Positive steering/yaw/lateral motion is left; angles are radians unless marked otherwise. Both use SI units.

```
Control → actuator dynamics / optional yaw-rate adapter
        → motor map + electrical limits
        → tire forces ↔ algebraic longitudinal load transfer
        → chassis derivatives + battery/energy derivatives
        → RK4 or adaptive RK45 → next state + telemetry
```

- Chassis: `dvx/dt = Fx/m + r*vy`, `dvy/dt = Fy/m - r*vx`, `dr/dt = Mz/Iz`. Front tire forces are rotated into the body frame. Aerodynamic drag acts at the CG. Rolling resistance is a longitudinal resistive force.
- Tires: `Fy = -μFz tanh(Cα/(μFz))`. Remaining longitudinal force is limited to `sqrt((μFz)² - Fy²)`. This **lateral-priority allocation assumes ideal traction control and ABS**; wheel speed, longitudinal slip and lockup are not simulated. Near zero speed, slip angles and friction forces are regularized.
- Load transfer: solve the front normal load `Fzf = (m*g*b - h*Fx_tires)/(a+b)` with damped fixed-point iterations. Residuals are recorded and non-convergence is logged. Wheel lift terminates the maneuver because pitch/wheelie dynamics are outside this model.
- Lean: `atan(ay/g)` is an equilibrium estimate. Exceeding the configured lean limit terminates the maneuver. Roll dynamics, balancing, countersteering, camber thrust, suspension, banking and rider motion are not modeled.
- Motor: mapped shaft torque, copper loss proportional to torque squared, iron loss proportional to shaft speed squared, and energized idle loss. Launch torque therefore consumes electrical power even at zero shaft speed. Zero torque assumes ideal electrical disconnect; persistent coasting iron/bearing losses are not represented.
- Inverter: separate bidirectional efficiency and energized idle loss. Gearbox losses are separate and positive in both drive and regen.
- Battery: `Vterminal = OCV(SOC) - I*R`, `Pterminal = Vterminal*I`, `Pheat = I²R`, `dSOC/dt = -I/Q`. Current comes from the stable low-current quadratic root. Available power respects current, voltage, SOC and maximum-power limits. Requested motor torque is reduced **before** evaluating delivered power.
- `capacity_kwh / nominal_voltage_v` defines rated amp-hours. Actual chemical energy depends on the integral of the configured OCV curve, so it need not equal the nameplate kWh exactly. SOC is coulomb-counted; chemical and terminal energies are integrated separately. Positive current/energy means discharge/consumption; negative means charging/recovery. Heat is always nonnegative.
- Braking uses a fixed configurable front/rear bias. Rear regen replaces rear friction braking within the requested rear brake force; when charge acceptance or regen capability decreases, friction braking fills the remainder. Regen fades at low speed. Any brake command takes priority over throttle.
- No temperatures or thermal derating yet. Per-component heat **rates and accumulated joules** are available for future thermal modules. Losses are not inferred temperatures.

The engine defaults to RK4 with a 5 ms output step. RK45 and internal substeps are selectable independently of the 50 ms default control interval. See the [solver settings](simlap/physics/README.md#integration-and-telemetry). It terminates at an encountered SOC boundary, locating the event within the final step. A run starting at maximum SOC can still brake using friction and regen that only supplies auxiliaries. Braking stops at the configured speed threshold (0.05 m/s), so reported stopping distance is to that threshold, not mathematical zero.

## Maneuvers

- **Acceleration:** starts at the configured speed with relaxed actuators and applies constant throttle. Reports interpolated 0–100 km/h time if reached.
- **Braking:** starts at the configured speed with relaxed actuators and applies constant braking. Includes brake response delay; reports stopping distance only if the stop threshold is reached.
- **Steady cornering:** solves lateral velocity, steering and throttle for zero body-velocity and yaw-rate derivatives, at a requested **total ground speed** and signed CG path radius. Negative radius turns right. It initializes the actuators at trim and holds the commands constant. The initial world velocity is aligned with +X; the reference circle center is `(0, radius)`. Infeasible/non-converged trims raise an error. Radius and speed drift quantify how well the simulation holds the equilibrium. Battery limitations can cause a later departure from the initial trim.

Duration completion is distinct from hitting a model limit. The default example outputs are synthetic; validate against measurements before using lap times or performance predictions.

With the roll/pitch chassis, corner trim also solves suspension, lean and wheel-speed equilibrium. `configs/maneuvers_roll_pitch.json` enables an explicit roll PD controller for the cornering test. Its gains are configurable; RL never uses this controller. See the [balance conventions and limitations](simlap/physics/README.md#balance-and-maneuver-control).

## Logs

Every run gets a unique directory under `results/`:

| File | Content |
| --- | --- |
| `telemetry.csv` | Initial sample and every accepted physics step: all states, commands, actual actuators, forces, slips, grip usage, loads, lean estimate, torque, RPM, electrical flows, losses, energy totals, numerical diagnostics and step timing |
| `engine.log` | Initialization, component names, trim, output steps with internal solver statistics at DEBUG, termination summary and exception tracebacks |
| `config.json` | Exact resolved model configuration |
| `metadata.json` | Maneuver parameters, timestamps, Python/platform, Git commit if available, source-file hashes and config hash |
| `summary.json` | Full-resolution performance metrics and numerical checks |
| `channels.json` | Signal conventions and units |
| `dashboard.json` | Sampled preview plus metadata/config; the telemetry server replaces its preview rows with full CSV data |

Physics signals in a row are evaluated at that row's timestamp. `ax_mps2` and `ay_mps2` are force/mass accelerations, **not** derivatives of the rotating-frame velocity components; both derivatives are also logged. Energy counters use Runge–Kutta stage values. `step_cpu_ms` measures elapsed compute time for integration and endpoint evaluation, excluding log I/O. DEBUG records output steps with internal accepted/rejected counts, not every intermediate RK stage. Failed simulations retain their CSV and traceback; only completed artifact sets appear in the dashboard.

Full telemetry is retained in memory while a maneuver runs to calculate metrics and build dashboard samples. For long training sessions, use the engine without a recorder or replace `Recorder` with a streaming implementation.

## Extending the simulator

| Module | Responsibility / replacement point |
| --- | --- |
| `types.py` | Immutable `Control`, `State`, `Evaluation`; controller and dynamics interfaces |
| `engine.py`, `integration.py` | Reset, step/advance, RK4/RK45, boundary handling, optional recorder |
| `physics/chassis.py`, `physics/roll_pitch.py` | Planar and roll/pitch chassis, actuator dynamics and component coupling |
| `physics/factory.py`, `physics/trim.py` | Model selection and advanced steady-corner trim |
| `physics/tires.py` | Planar saturating and wheel-speed combined-slip tires |
| `physics/powertrain.py` | `PowertrainModel` interface and mapped electric implementation |
| `physics/battery.py` | `BatteryModel` interface and resistive pack |
| `tracks/` | Geometry, projection, centerline/boundary export |
| `controllers/` | Independent benchmark controllers |
| `rl/` | Gymnasium environment, SAC training, evaluation, checkpoints and logging |
| `maneuvers.py` | Maneuver initialization and steady-state trim |
| `telemetry.py` | Artifacts and summaries |
| `dashboard.py` + `dashboard/` | Local read-only result browser |

Inject components explicitly; no registry or plugin framework is required:

```python
from simlap import Control, Simulator, load_config
from simlap.physics.battery import ResistiveBattery
from simlap.physics.chassis import PlanarMotorcycle
from simlap.physics.powertrain import ElectricPowertrain

config = load_config()
battery = ResistiveBattery(config.battery)  # replace with your implementation
powertrain = ElectricPowertrain(config, battery=battery)
model = PlanarMotorcycle(config, powertrain=powertrain)
sim = Simulator(config, model=model)

# Future policy output; no reference path or trajectory is an engine dependency.
state, signals = sim.advance(Control(throttle=0.5, brake=0.0, steer_rad=0.02))
# Alternatively request yaw rate; an adapter generates a rate-limited steer target.
state, signals = sim.advance(Control(throttle=0.5, yaw_rate_radps=0.1))
```

`step()` advances one physics step; `advance()` holds an action for one control interval. Reset after termination. Throttle/brake are in `[0,1]`; steering is radians; yaw-rate commands are rad/s. Steering and yaw-rate commands are mutually exclusive. The yaw-rate adapter is geometric feedforward plus proportional feedback, **not** a guaranteed tracking controller.

To add RC battery states or temperatures, subclass the frozen `State` or `RollPitchState` dataclass and provide a dynamics model returning matching derivative states. Both integrators preserve the concrete state type, and the recorder automatically includes its fields. Keep `evaluate(state, control)` pure: do not mutate hidden battery/thermal states during RK calls. Extra states must be integrated by the engine. A radically different state layout or reverse-motion model should replace the engine's small motorcycle-specific boundary checks as well. New component parameters should get typed config fields and validation.

Tests cover electrical energy conservation, voltage/current/SOC limits, launch losses, nominal scaling, combined tire grip, regen blending, braking, timestep convergence, left/right trim stability, yaw actuation, artifacts and extended-state integration.

## Technical references

The baseline follows standard [planar single-track equations](https://www.mathworks.com/help/vdynblks/ref/vehiclebody3dof.html), [mapped motor/loss modeling](https://www.mathworks.com/help/autoblks/ref/mappedmotor.html), and [equivalent-circuit battery modeling](https://www.mathworks.com/help/simscape-battery/ref/batteryequivalentcircuit.html). The implementation is intentionally simpler than these reference products. A fuller [motorcycle lateral model](https://www.ibisc.univ-evry.fr/~harioui/publications/TIV_2019_vf.pdf) illustrates additional steering and gyroscopic dynamics beyond this simulator.
