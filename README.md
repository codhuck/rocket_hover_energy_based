# Project 1 — Planar TVC Rocket with Lyapunov Hover Control

![Visualisation preview](figures/rocket_visualization_preview.png)

## Overview
This repository implements a **planar thrust-vector-controlled rocket** that stabilizes to a fixed hover point using a **Lyapunov-inspired cascade controller**. The repository is arranged to satisfy the course project rules: the system dynamics, controller, simulation loop, and visualisation are split into separate modules; the outputs include reproducible plots, a real-time animation, explicit run commands, and a short results summary.

The default hover target is
- **position:** `(x_d, y_d) = (0.0, 8.0) m`
- **pitch:** `phi = 0 rad`
- **velocity:** `vx = 0, vy = 0`
- **angular rate:** `omega = 0`

## Quick Start
Create an environment and install the dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Regenerate all figures, the summary file, and the corrected real-time MP4 animation:

```bash
bash run_project.sh
```

Equivalent direct command:

```bash
python -m src.main --config configs/default.yaml --output-root .
```

Generated outputs:
- `figures/state_trajectories.png`
- `figures/attitude_and_gimbal.png`
- `figures/control_and_error.png`
- `figures/planar_trajectory.png`
- `figures/rocket_visualization_preview.png`
- `figures/summary.json`
- `animations/rocket_hover_realtime.mp4`

## Repository Structure
```text
project_1_lyapunov_control_planar_tvc_rocket/
├── README.md
├── P1_TVC_Lyapunov_Derivation.md
├── requirements.txt
├── run_project.sh
├── configs/
│   └── default.yaml
├── src/
│   ├── __init__.py
│   ├── controller.py
│   ├── main.py
│   ├── simulation.py
│   ├── system.py
│   └── visualization.py
├── figures/
│   ├── attitude_and_gimbal.png
│   ├── control_and_error.png
│   ├── planar_trajectory.png
│   ├── rocket_visualization_preview.png
│   ├── state_trajectories.png
│   └── summary.json
└── animations/
    └── rocket_hover_realtime.mp4
```

## 1. Problem Definition
The control task is to stabilize a planar rocket to a stationary hover point under gravity, bounded gimbal actuation, bounded throttle, translational drag, and fuel depletion.

### Method class
The method belongs to **Lyapunov-based nonlinear control** and uses a two-layer structure:
1. an **outer position loop** computes the desired thrust direction and thrust magnitude;
2. an **inner attitude loop** computes the nozzle gimbal command that drives the rocket pitch toward the desired pitch.

### Context and assumptions
- The rocket is planar, so the state contains one attitude angle and one angular rate.
- The control moment arm `l_cp` and the inertia `J_const` are frozen at midpoint mass for Project 1.
- Rotational aerodynamic damping is neglected.
- Fuel depletion is included explicitly, so the hover throttle slowly decreases during the run.
- The attitude loop is tuned faster than the position loop.

## 2. System Description
### State, control, and notation
The state is

\[
q = [x, y, \phi, \dot x, \dot y, \dot \phi, m]^T,
\]

where
- `x, y` are inertial horizontal and vertical positions in meters,
- `phi` is the pitch angle from the vertical axis in radians,
- `vx = dot x` and `vy = dot y` are translational velocities in meters per second,
- `omega = dot phi` is the angular rate in radians per second,
- `m` is the instantaneous mass in kilograms.

The control input is

\[
u = [\alpha, \delta]^T,
\]

where
- `alpha in [0, 1]` is the throttle command,
- `delta` is the nozzle gimbal angle with `|delta| <= delta_max`.

### Nonlinear dynamics used in the code
With thrust `F = alpha F_max`, translational drag coefficient `beta_drag`, and midpoint approximations for `J_const` and `l_cp`, the implemented dynamics are

\[
\dot m = -\alpha \dot m_{max},
\]
\[
\ddot x = \frac{\alpha F_{max}}{m}\sin(\phi + \delta) - \frac{\beta_{drag}}{m}\dot x\sqrt{\dot x^2 + \dot y^2},
\]
\[
\ddot y = \frac{\alpha F_{max}}{m}\cos(\phi + \delta) - g - \frac{\beta_{drag}}{m}\dot y\sqrt{\dot x^2 + \dot y^2},
\]
\[
\ddot \phi = -\frac{\alpha F_{max} l_{cp}}{J_{const}}\sin(\delta).
\]

The code computes `F_max = m_dot_max * v_e` from the mass-flow rate and effective exhaust velocity, then computes midpoint-mass values of `l_cp` and `J_const` from the dry structure and fuel geometry.

### Constraints
- `alpha_min <= alpha <= 1`
- `|delta| <= delta_max`
- `m >= m_dry`

When the dry mass is reached, the thrust is set to zero and further fuel depletion stops.

## 3. Mathematical Specification
### Outer-loop virtual thrust law
Define the position errors

\[
e_x = x - x_d, \qquad e_y = y - y_d.
\]

The outer loop commands the desired specific thrust components

\[
A_x = -k_{px} e_x - k_{dx} \dot x,
\]
\[
A_y = g - k_{py} e_y - k_{dy} \dot y.
\]

From these terms,

\[
T_{des} = \sqrt{A_x^2 + A_y^2}, \qquad \phi_{des} = \operatorname{atan2}(A_x, A_y),
\]
\[
\alpha = \operatorname{clip}\left(\frac{m T_{des}}{F_{max}}, \alpha_{min}, 1\right).
\]

The desired pitch is clipped to `phi_des_limit_deg` from the configuration file.

### Inner-loop Lyapunov attitude law
The inner loop uses the wrapped attitude error

\[
e_\phi = \operatorname{wrap}(\phi - \phi_{des}).
\]

The gimbal command is computed from

\[
\sin(\delta) = \frac{J_{const}}{\alpha F_{max} l_{cp}}\left(k_\phi e_\phi + k_\omega \dot \phi\right),
\]

followed by an `arcsin` and a hard saturation to `[-delta_max, delta_max]`.

### Stability interpretation
- The **outer loop** makes the translational error behave like a damped second-order system.
- The **inner loop** drives the rocket pitch toward the commanded pitch and damps angular motion.
- Translational drag is not cancelled, so it contributes extra dissipation.

The code intentionally implements the simplified Project 1 model, not a fully parameter-varying rocket. A longer derivation note is included in `P1_TVC_Lyapunov_Derivation.md`.

## 4. Method Description
### Control pipeline
At every integration step the code performs the following pipeline:
1. read the current state `(x, y, phi, vx, vy, omega, mass)`;
2. compute `e_x`, `e_y`, `A_x`, and `A_y`;
3. compute the desired pitch `phi_des` and desired thrust magnitude `T_des`;
4. compute the throttle `alpha`;
5. wrap the attitude error `e_phi = wrap(phi - phi_des)`;
6. compute the gimbal angle `delta` from the inner-loop law;
7. propagate the nonlinear dynamics with `solve_ivp`;
8. post-process the state history into plots, an animation, and summary metrics.

### Why the controller works in practice
The outer loop steers the thrust vector toward the hover point while damping velocity. The inner loop is tuned sufficiently faster than the position loop, so the commanded pitch is tracked without large gimbal excursions.

## 5. Experimental Setup
### Initial condition
- `x(0) = -2.0 m`
- `y(0) = 1.0 m`
- `phi(0) = 12.0 deg`
- `vx(0) = 0.5 m/s`
- `vy(0) = 0.0 m/s`
- `omega(0) = -5.0 deg/s`
- `m(0) = 1.60 kg`

### Target state
- `x_d = 0.0 m`
- `y_d = 8.0 m`
- `phi_d = 0.0 rad`
- `vx_d = vy_d = omega_d = 0`

### Physical parameters
- `g = 9.81 m/s^2`
- `beta_drag = 0.08`
- `m_dry = 1.05 kg`
- `m_dot_max = 0.02 kg/s`
- `v_e = 1200 m/s`
- `delta_max = 15 deg`
- `alpha_min = 0.20`
- `F_max = 24.0 N`
- `l_cp = 0.6091 m` at midpoint mass
- `J_const = 0.1183 kg m^2` at midpoint mass

### Controller gains
- `k_px = 0.45`
- `k_dx = 1.30`
- `k_py = 0.80`
- `k_dy = 1.80`
- `k_phi = 18.0`
- `k_omega = 7.0`
- `phi_des_limit_deg = 22.0`

### Numerical setup
- final simulation time: `20.0 s`
- sample count: `1201`
- integrator: `scipy.integrate.solve_ivp`
- integrator max step: `0.02 s`

## 6. Reproducibility
The repository includes exact commands, a dependency file, and a deterministic default configuration.

### Reproduce the final outputs
From the repository root, run:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m src.main --config configs/default.yaml --output-root .
```

### What each module does
- `src/system.py` defines the rocket parameters and nonlinear right-hand side;
- `src/controller.py` implements the Lyapunov hover controller;
- `src/simulation.py` integrates the system and computes metrics;
- `src/visualization.py` generates all figures and the corrected real-time MP4 animation;
- `src/main.py` runs the full pipeline from config to outputs.

## 7. Results Summary
The default experiment converges to the hover target and stays comfortably inside the gimbal limit.

### Quantitative results
- final position: `(-3.91e-05, 7.999994) m`
- final pitch: `-5.9e-05 deg`
- final speed: `2.03e-05 m/s`
- final position error: `3.91e-05 m`
- maximum position error during the run: `7.280 m`
- maximum absolute pitch: `12.000 deg`
- maximum absolute gimbal command: `1.330 deg`
- fuel used: `0.242 kg`
- minimum mass reached: `1.358 kg`

### What works
- The rocket removes the horizontal offset and reaches the target altitude.
- The attitude loop remains fast compared with the translational motion.
- The gimbal demand stays small compared with the available `15 deg` authority.
- The exported MP4 now matches the physical simulation time instead of playing too fast.

### What remains limited
- The inertia and control moment arm are frozen at midpoint mass.
- Rotational drag, actuator dynamics, and sensor noise are omitted.
- The proof is quasi-static with respect to mass variation.
- No baseline comparison is included because this is a Project 1 repository.

## 8. Figures and Interpretation
### State trajectories
![State trajectories](figures/state_trajectories.png)
The position histories show that the rocket reaches the hover target and removes the initial offsets without persistent oscillation.

### Attitude and gimbal command
![Attitude and gimbal](figures/attitude_and_gimbal.png)
The desired pitch and actual pitch quickly align, while the gimbal command remains far below the hard `15 deg` limit. The conclusion is that the inner loop has sufficient authority for the default case.

### Control effort and hover error
![Control and error](figures/control_and_error.png)
Throttle, speed, and position error all decay toward their hover values. The conclusion is that the closed-loop system settles smoothly instead of chattering around the target.

### Planar trajectory
![Planar trajectory](figures/planar_trajectory.png)
The planar path bends smoothly toward the hover point. The conclusion is that the rocket reaches the target without aggressive overshoot.

## 9. Animation
The project includes a corrected real-time animation:
- `animations/rocket_hover_realtime.mp4`

The animation shows the rocket body, the hover target, the path, the current orientation, the current time, the throttle command, the gimbal angle, and the instantaneous position error.

## 10. Theory and Implementation Match
This repository intentionally implements the **simplified Project 1 model**. Specifically:
- `J_const` and `l_cp` are frozen at midpoint mass;
- the controller is the two-layer Lyapunov hover law described above;
- the README, code, plots, and animation describe the same model and assumptions.

## 11. Possible Extensions
- update `J(m)` and `l_cp(m)` online instead of freezing them;
- add actuator lag or rate limits;
- add disturbances and robustness experiments;
- include a simple baseline controller for an optional Project 1 comparison;
- extend the planar setup from hover stabilization to trajectory tracking.

## 12. Notes on AI Use
AI assistance was used to help scaffold code, restructure files, and polish the documentation and visualisation. The final equations, parameters, and interpretation of the plots should still be checked by the team before submission.
