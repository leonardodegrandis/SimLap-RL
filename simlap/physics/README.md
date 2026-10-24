# Roll, pitch and tire dynamics

Select the new model without changing the planar configuration used by existing policies:

```sh
.venv/bin/python -m simlap.run all \
  --override configs/roll_pitch.json \
  --maneuvers configs/maneuvers_roll_pitch.json
.venv/bin/python -m simlap.dashboard
```

The individual acceleration/braking/cornering scripts accept the same arguments. Refresh the dashboard to see the new runs. All parameters are in `configs/motorcycle.json`; the overlay selects `dynamics.model = "roll_pitch"` and RK45. Mass includes rider; the inertias, suspension and tire stiffnesses are development assumptions, **not measured or calibrated motorcycle data**.

## Scope and state

This is a **reduced six-DOF sprung-body model plus two wheel rotations**, not a full multibody motorcycle. It integrates longitudinal/lateral motion, yaw, roll, pitch and suspension heave. The added states are:

| State | Convention |
| --- | --- |
| `roll_rad`, `roll_rate_radps` | Positive leaning left |
| `pitch_rad`, `pitch_rate_radps` | Positive nose up |
| `heave_m`, `heave_rate_mps` | Upward suspension displacement from static equilibrium |
| `front_wheel_radps`, `rear_wheel_radps` | Forward wheel rotation; rear includes reflected motor inertia |
| `tire_heat_j`, `rolling_heat_j`, `suspension_heat_j`, `roll_damper_heat_j` | Integrated dissipated energy |

World `x_m`, `y_m` describe the CG. Actual CG height is `H = h*cos(roll) + heave`; suspension heave excludes the geometric CG drop from lean. `RollPitchState` extends the existing immutable state. Passing a plain `State` to `Simulator.reset()` initializes wheel speeds to rolling speed and returns the converted state. Pass `RollPitchState` explicitly to set slip, lean or suspension perturbations.

## Forces and moments

Let `a,b` be the distances from CG to the front/rear contact, `L=a+b`, and `h` the upright CG height. Spring extensions and normal loads use a small-pitch approximation:

```
df = heave + a*pitch              dr = heave - b*pitch
Fzf = max(0, m*g*b/L - kf*df - cf*df_dot)
Fzr = max(0, m*g*a/L - kr*dr - cr*dr_dot)
```

These are preloaded linear vertical suspension equivalents with no unsprung mass or suspension motion ratios. Positive displacement means extension. Normal loads can sum to more or less than `m*g` during vertical acceleration. Longitudinal load transfer emerges from pitch/suspension dynamics; it is no longer an algebraic fixed-point solve.

Contact velocities include yaw, roll and pitch rates, and are rotated into the steered front-wheel frame. In each wheel frame:

```
kappa = (R*omega - Vx) / max(abs(Vx), abs(R*omega), regularization_speed)
alpha = atan2(Vy, max(abs(Vx), regularization_speed))
raw_force = [Cx*kappa, -Cy*alpha]
force = raw_force * mu*Fz*tanh(norm(raw_force)/(mu*Fz)) / norm(raw_force)
```

The implementation handles zero force/load safely. Both components share the friction circle; wheelspin, braking slip and lockup emerge from the wheel equations. There is **no ABS or traction controller**. The smooth saturating law has no calibrated post-peak falloff, camber thrust, relaxation length or temperature dependence. Locked-wheel stopping performance therefore must not be interpreted as a validated prediction. Replace `CombinedSlipTire` to introduce a measured tire law.

With `Fx,Fy` the summed body-frame tire forces, the rotational and vertical equations are:

```
Iz * yaw_ddot   = a*Fyf_body - b*Fyr + h*sin(roll)*Fx
Iroll*roll_ddot = h*sin(roll)*(Fzf+Fzr) - H*Fy - roll_damping*roll_dot
Ipitch*pitch_ddot = H*Fx + a*Fzf - b*Fzr
heave_ddot = (Fzf+Fzr)/m - g + h*cos(roll)*roll_dot² + h*sin(roll)*roll_ddot
```

Body translation retains `vx_dot = (Fx-drag)/m + yaw_rate*vy`, `vy_dot = Fy/m - yaw_rate*vx`. Wheel torque balances integrate drive/regen torque, friction-brake torque, rolling resistance and `-R*Fx_tire`. The rear effective inertia is `Irear + Imotor*gear_ratio²`. Electric power uses **actual rear wheel speed**, so spinning the rear wheel changes motor RPM and electrical demand. Brake/rolling torques are regularized near zero rotation; the brake configuration's force parameter becomes an equivalent axle torque demand via wheel radius.

Rolling resistance acts on the wheels; it is not also subtracted as a chassis force. Tire dissipation includes longitudinal slip and lateral scrub. The tests verify the reduced equations' instantaneous mechanical power balance, including wheel and vertical kinetic energy, gravitational/preloaded-spring potential energy, and dissipative terms. Electrical and mechanical heat counters remain separate, ready for future thermal models.

The reduced equations omit full Euler/gyroscopic coupling, steering-axis trail/caster dynamics, wheel torque reactions on the frame, tire crown/contact migration, aerodynamic moments, rider motion and road banking/bumps. Inertias are constant; steering is a commanded angle with actuator lag. Pitch uses small-angle geometry with a configurable operating limit. Wheel lift, excessive lean/pitch and excessive suspension travel terminate the simulation; airborne motion, wheelies and suspension stops are not simulated. Forward-only motion and the engine's near-stop handling are retained.

## Balance and maneuver control

Roll is a dynamic state, not `atan(ay/g)` tracking. A positive front steering angle initially creates a left force and a **right roll acceleration**; a lean disturbance can grow and the motorcycle can fall. There is no hidden rider inside the model. The yaw-rate adapter only requests steering; it does not guarantee balance.

Corner initialization solves eight equilibrium conditions for lateral speed, steering, throttle, roll, pitch, heave and wheel speeds. The supplied advanced maneuver configuration explicitly enables a simple validation balance controller:

```
steer_request = trim_steer + roll_kp*(roll - trim_roll) + roll_kd*roll_rate
```

It runs at `control_dt_s`, holds trim throttle, and uses the configured steering actuator limits. It is intended to hold the solved constant-radius condition, not drive a circuit or track a speed. Set both gains to zero for an open-loop trim experiment; roundoff/perturbations may then grow. Requested steering is logged and `roll_balance_controller` appears in the run summary. Acceleration/braking start exactly upright and straight; symmetry keeps roll at zero, which does not demonstrate disturbance stability.

## Integration and telemetry

The original solver was already RK4. `simlap/integration.py` now separates integration from dynamics and supports:

| Setting | Behavior |
| --- | --- |
| `integrator: "rk4"` | Fixed fourth-order Runge–Kutta, subdivided by `max_internal_step_s` |
| `integrator: "rk45"` | Dormand–Prince fifth-order steps with an embedded fourth-order error estimate |
| `relative_tolerance`, `absolute_tolerance` | RMS scaled local error target for RK45; scalar tolerances across all state channels |
| `max_internal_step_s` | Largest internal step; advanced example uses 1 ms |
| `min_internal_step_s`, `max_internal_steps` | Failure limits; unmet tolerances raise an error |

The default telemetry/physics interval is 5 ms and control interval is 50 ms. Internal stages and rejected attempts do not mutate model state or produce telemetry rows. The solver reaches each output endpoint; its accepted/rejected counts, RHS evaluation count and maximum accepted normalized error are logged for every output step, including work used to locate an SOC boundary. The error norm is zero for RK4. Other chassis limits are checked at output endpoints, not root-localized. Tight suspension/tire settings can be stiff: explicit RK45 controls error but is not an implicit stiff solver. Reduce internal steps or add an implicit solver when extending the model.

In the dashboard, select any of `roll_deg`, `pitch_deg`, `front_normal_n`, `rear_normal_n`, `front_fx_n`, `front_fy_n`, `rear_fx_n`, `rear_fy_n`, slip ratios, wheel speeds, suspension displacement or solver channels. Front `fx/fy` are in the **wheel frame**; `front_body_fx_n/front_body_fy_n` are rotated body components. `lean_estimate_deg` remains a quasi-steady diagnostic and can differ from actual roll. All states and endpoint signals are in CSV; the existing plot colors, legends and panel assignments work for the new channels.

## RL and extension

```sh
# Starts a new policy; does not resume or alter existing planar training.
.venv/bin/python scripts/train.py --vehicle-override configs/roll_pitch.json
```

The RL environment automatically adds eight observations for roll/pitch/heave, their rates and wheel speeds; it also checks the laterally shifted contact points against track boundaries. It never enables the maneuver balance controller. The policy must learn balance as well as throttle, braking and steering. This is substantially harder than the planar task and the existing SAC settings are only a starting point. Old checkpoints remain planar; their observation interface cannot be reused as a trained roll/pitch policy.

`physics/factory.py` selects the chassis; explicit `Simulator(..., model=...)` injection still works. Tires and powertrain can be injected into either chassis; the combined-slip tire has a separate contract from the planar tire because it needs wheel speed and longitudinal stiffness. New thermal/electrical states should extend the state dataclass and be returned in its derivative. Keep RHS evaluation pure for rejected RK stages. No additional core dependencies are required.

References for context: [motorcycle roll/yaw modeling](https://www.scitepress.org/PublishedPapers/2021/103868/103868.pdf), [SciPy's RK45 method description](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html). This reduced implementation is not a reproduction or validation of the cited motorcycle model.
