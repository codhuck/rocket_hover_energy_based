# Project 1 — Planar TVC Rocket: Attitude Stabilization with a Lyapunov Controller

![Visualization preview](figures/rocket_visualization_preview.png)

## Overview
This repository contains the **first project iteration** for a planar thrust-vector-controlled (TVC) rocket. The implemented objective is intentionally narrow:

\[
\phi(t) \to 0, \qquad \dot\phi(t) \to 0,
\]

that is, the rocket pitch angle is stabilized to the upright configuration by a **Lyapunov-based nonlinear attitude controller**.

This version is intentionally simplified and should be understood as a **model-verification and controller-verification stage**, not a full hover-tracking solution. In particular, the implemented project:
- does **not** regulate position,
- does **not** include aerodynamic drag,
- does **not** include variable mass,
- does **not** include adaptation or outer-loop guidance.

That simplification is deliberate. For a first project, a smaller but internally consistent implementation is stronger than an overcomplicated unfinished model.

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
    └── rocket_attitude_realtime.mp4
```

The code is separated into logical parts:
- `system.py` contains the plant model,
- `controller.py` contains the control law,
- `simulation.py` runs the closed-loop simulation,
- `visualization.py` produces plots and animation,
- `main.py` is the entry point.

## 1. Problem Definition
### Control task
The control objective is **attitude stabilization only**:

\[
\phi(t) \to 0, \qquad \dot\phi(t) \to 0.
\]

The rocket is simulated in planar translation as well, but translational motion is **not** part of the feedback objective in this first project.

### Plant / environment
The plant is a rigid planar rocket with:
- a single gimballed thrust vector,
- one pitch angle and one pitch rate,
- constant mass,
- constant inertia,
- constant control moment arm.

### Assumptions and context
The implemented model uses the following assumptions:
- total mass is constant,
- moment of inertia is constant,
- aerodynamic drag is neglected,
- the control moment arm is constant,
- throttle is fixed at the hover value,
- only the gimbal angle is used for feedback control.

These assumptions are intentional and define the scope of the first project iteration.

### Method class
The controller belongs to the class of **Lyapunov-based nonlinear control methods**.

## 2. System Description
### State variables
The state vector is

\[
q = [x,\; y,\; \phi,\; \dot x,\; \dot y,\; \dot\phi]^T.
\]

Where:
- `x` is horizontal position in meters,
- `y` is vertical position in meters,
- `\phi` is pitch angle from the inertial vertical axis in radians,
- `\dot x` and `\dot y` are translational velocities in m/s,
- `\dot\phi` is angular velocity in rad/s.

### Control inputs
The control input is written as

\[
u = [\alpha,\; \delta]^T,
\]

where:
- `\alpha` is throttle,
- `\delta` is the nozzle gimbal angle.

In the implemented project:
- `\alpha` is **not optimized online**,
- `\alpha` is fixed to the hover value
  \[
  \alpha_{hover} = \frac{mg}{F_{max}},
  \]
- the actual feedback action is the gimbal command `\delta`.

### Parameters
The main physical parameters are:
- `m` — total rocket mass,
- `J_const` — constant pitch inertia,
- `l_cp` — nozzle-to-center-of-mass control moment arm,
- `F_max` — maximum thrust,
- `g` — gravity,
- `\delta_max` — gimbal saturation limit.

### Constraints
The control bounds are:

\[
0 \le \alpha \le 1,
\qquad
|\delta| \le \delta_{max}.
\]

### Equations of motion
With constant mass, no drag, and thrust magnitude

\[
F = \alpha F_{max},
\]

the nonlinear planar dynamics implemented in `src/system.py` are

\[
\ddot x = \frac{F}{m}\sin(\phi + \delta),
\]
\[
\ddot y = \frac{F}{m}\cos(\phi + \delta) - g,
\]
\[
\ddot\phi = -\frac{F l_{cp}}{J_{const}}\sin(\delta).
\]

The translational coordinates remain in the model so that the simulation and animation show how the vehicle moves while the attitude is being recovered.

## 3. Mathematical Specification
### Notation used in the controller
The target attitude is the upright orientation:

\[
\phi_{target} = 0.
\]

The wrapped angle error is defined as

\[
e_\phi = \operatorname{wrap}(\phi - \phi_{target}).
\]

The angular rate is

\[
\omega = \dot\phi.
\]

### Lyapunov candidate
The controller is based on the quadratic Lyapunov function

\[
V(e_\phi, \omega) = \frac{1}{2}k_\phi e_\phi^2 + \frac{1}{2}\omega^2,
\]

with gains

\[
k_\phi > 0, \qquad k_\omega > 0.
\]

### Control law
The gimbal command is computed by enforcing a damping-like angular response:

\[
\sin(\delta)
=
\frac{J_{const}}{\alpha F_{max} l_{cp}}
\left(k_\phi e_\phi + k_\omega \omega\right).
\]

The implemented controller then applies:
1. clipping of `sin(\delta)` to `[-1, 1]`,
2. `\delta = \arcsin(\sin\delta)`,
3. saturation of `\delta` to `[-\delta_{max}, \delta_{max}]`.

### Interpretation
When the available gimbal authority is sufficient and saturation does not dominate the motion, this law damps angular velocity and drives the pitch angle toward the upright equilibrium.

## 4. Method Description
### Derivation idea
The rotational subsystem has the form

\[
\ddot\phi = -\frac{\alpha F_{max} l_{cp}}{J_{const}}\sin(\delta).
\]

The controller uses the gimbal angle to shape this rotational acceleration so that the Lyapunov function decreases along the closed-loop trajectory. This is the reason the controller depends directly on both the angle error `e_\phi` and the angular rate `\omega`.

### Approximations and simplifications
This project intentionally uses the following simplifications:
- translational control is omitted,
- drag is omitted,
- mass depletion is omitted,
- parameter adaptation is omitted.

The purpose is to validate the sign conventions, actuation direction, stabilization behavior, and code structure before moving to a more complete rocket model.

## 5. Algorithm Listing
At every controller evaluation step:

1. read the current state `(x, y, \phi, \dot x, \dot y, \omega)`,
2. compute the wrapped error `e_\phi = wrap(\phi - \phi_target)`,
3. set the throttle to the constant hover value `\alpha = mg / F_max`,
4. compute `\sin(\delta)` from the Lyapunov attitude law,
5. clip `\sin(\delta)` to `[-1, 1]`,
6. compute `\delta = arcsin(\sin\delta)`,
7. clamp `\delta` to the gimbal limit,
8. propagate the full nonlinear planar dynamics,
9. save the state history, controls, plots, preview image, summary file, and animation.

## 6. Experimental Setup
### Initial condition
The default experiment in `configs/default.yaml` uses:
- `x(0) = 0.0 m`,
- `y(0) = 8.0 m`,
- `\phi(0) = 20.0 deg`,
- `\dot x(0) = 0.0 m/s`,
- `\dot y(0) = 0.0 m/s`,
- `\omega(0) = -8.0 deg/s`.

### Physical parameters
The default rocket parameters are:
- `g = 9.81 m/s^2`,
- `m = 1.60 kg`,
- `F_max = 24.0 N`,
- `J_const = 0.1183 kg·m^2`,
- `l_cp = 0.6091 m`,
- `\delta_max = 15 deg`.

This gives the constant hover throttle

\[
\alpha_{hover} = \frac{mg}{F_{max}} \approx 0.654.
\]

### Controller gains
The default controller gains are:
- `k_phi = 18.0`,
- `k_omega = 7.0`,
- `phi_target = 0 deg`.

### Numerical setup
The default simulation uses:
- final time `t_final = 12.0 s`,
- `sample_count = 721`,
- `max_step = 0.02 s`,
- `scipy.integrate.solve_ivp` as the numerical integrator.

## 7. Reproducibility
### Dependencies
Install dependencies from:

```bash
pip install -r requirements.txt
```

### Exact commands to run the project
From the repository root:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
bash run_project.sh
```

Equivalent direct command:

```bash
python -m src.main --config configs/default.yaml --output-root .
```

### Produced outputs
Running the project generates:
- `figures/state_trajectories.png`,
- `figures/attitude_and_gimbal.png`,
- `figures/control_and_error.png`,
- `figures/planar_trajectory.png`,
- `figures/rocket_visualization_preview.png`,
- `figures/summary.json`,
- `animations/rocket_attitude_realtime.mp4`.

## 8. Results Summary
### What works
- The pitch angle converges to the upright equilibrium.
- The angular velocity converges toward zero.
- The gimbal command remains bounded by the imposed limit.
- The generated plots are readable and directly connected to the implemented controller.
- The animation shows the rocket body, thrust direction, gimbal deflection, time, and trajectory.

### What does not work or remains limited
- Position is not controlled.
- The rocket can drift significantly in `x` and `y` while the pitch is recovering.
- Constant hover throttle only balances gravity near the nominal upright condition.
- Without drag, translational motion is not naturally damped.

### Interpretation of the main outputs
- **`state_trajectories.png`** shows that translation evolves even though only attitude is controlled; the important conclusion is that angular stabilization alone does not imply position stabilization.
- **`attitude_and_gimbal.png`** shows the decay of `\phi(t)` toward the target and the bounded gimbal response used to achieve it.
- **`control_and_error.png`** shows that the attitude error decreases while throttle stays constant, which is consistent with the chosen simplified design.
- **`planar_trajectory.png`** visualizes the drift that remains when no outer-loop position controller is present.
- **`rocket_attitude_realtime.mp4`** makes the stabilization physically interpretable by showing the body orientation, nozzle deflection, thrust plume, and camera-following trajectory.

## 9. Consistency Note
This repository intentionally represents a **simplified Project 1 implementation**. The implemented code, the README, the generated plots, and the animation all describe the same model:
- constant mass,
- no drag,
- fixed hover throttle,
- Lyapunov-based attitude stabilization through the gimbal angle.

Any future extension to hover control, position tracking, variable mass, or aerodynamic effects should be documented as a separate next iteration rather than silently mixed into this version.