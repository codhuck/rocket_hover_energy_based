# Project 3 — Putch Tracking via Backstepping.

![Adaptive controller]()
Control law synthesis for rocket pitch tracking using backstepping, with stability analysis under discontinuous thrust.

## Overview

This project addresses the problem of tracking a prescribed pitch program during the active phase of a rocket's flight. The setting is planar motion in the vertical plane, accounting for aerodynamic forces and moments as well as the nozzle actuator dynamics. The control law is constructed via backstepping — a recursive procedure that steps through the cascade of subsystems (pitch angle → pitch rate → nozzle deflection) and at each level builds a virtual control.

Special attention is given to the case when the engine thrust undergoes discontinuities at known time instants (stage separation, engine shutdown/ignition). It is shown that thrust discontinuities do not break the closed-loop system: the Lyapunov function remains continuous through the switching instants, and the exponential tracking error bound holds globally over the entire powered phase.

## Plant Model

The plant is a rocket controlled in pitch via nozzle deflection. The full state vector is seven-dimensional (position, velocity, pitch angle, pitch rate, nozzle angle), but only three variables enter the feedback loop: pitch angle, pitch rate, and nozzle deflection angle. The remaining variables are used to compute aerodynamic quantities (angle of attack, dynamic pressure).

The nozzle actuator is modeled as a first-order lag. Linearization of the nozzle deflection angle is justified by its smallness and provides the strict-feedback structure of the equations.

## Structure of the Theoretical Chapter

1. **Tracking problem.** General formulation for nonlinear systems; transition to a finite-time setting natural for the powered flight phase.

2. **Feedforward (open-loop control).** Construction of the control signal by model inversion — from the pitch program to the actuator command. Justification of why open-loop control is fundamentally insufficient: the integrator chain in the error dynamics provides no decay of deviations.

3. **Feedback (closed-loop control).** The feedback principle introduction and its usage for such systems.

4. **Backstepping.** Reduction of the model to strict-feedback form. Three-step recursive synthesis procedure with derivation of the control law and a stability proof via a composite Lyapunov function.

5. **Stability under discontinuous thrust.** Analysis of how thrust jumps affect the closed loop. Discussion of relevant stability notions: exponential convergence with a prescribed rate, ultimate boundedness under disturbances.

## Assumptions

- Model parameters (mass, moments of inertia, aerodynamic coefficients, actuator time constant) are known.
- The full state vector is known from DE of motion. 
- The desired pitch program is sufficiently smooth (three continuous derivatives).
- The required nozzle deflection lies strictly inside the admissible range.

## Quick Start

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

Generated outputs:

- `figures/state_trajectories.png`
- `figures/attitude_and_gimbal.png`
- `figures/parameter_estimation.png`
- `figures/lyapunov_function.png`
- `figures/comparison_baseline_vs_adaptive.png`
- `figures/phase_portrait.png`
- `figures/planar_trajectory.png`
- `figures/summary.json`
- `animations/rocket_attitude_realtime.gif`

### Configuration

## Repository Structure

```text
project_2_adaptive_control_planar_tvc_rocket/
├── README.md
├── README-derivation-adaptive.md
├── README-Aerodynamics.md
├── requirements.txt
├── run_project.sh
├── configs/
│   └── default.yaml
├── src/
│   ├── __init__.py
│   ├── system.py
│   ├── controller.py
│   ├── simulation.py
│   ├── visualization.py
│   └── main.py
├── figures/
│   └── BF_Sys.png
└── animations/
```

---

# System Description and Problem Definition

## 1. System Description

### 1.1 Coordinate Frame and State Variables

We consider planar motion of a rocket in the vertical plane. The pitch angle $\vartheta$ is measured from the inertial horizontal axis: $\vartheta = 0$ corresponds to horizontal flight, $\vartheta = \pi/2$ to vertical.

The full state vector is

$$
q = (x_1, x_2, x_3, x_4, x_5, x_6, x_7)^\top \in \mathbb{R}^7,
$$

where:

| State | Meaning |
|-------|---------|
| $x_1 = x$ | Horizontal position of center of mass |
| $x_2 = y$ | Vertical position (altitude) |
| $x_3 = \dot{x}$ | Horizontal velocity |
| $x_4 = \dot{y}$ | Vertical velocity |
| $x_5 = \vartheta$ | Pitch angle |
| $x_6 = \dot{\vartheta}$ | Pitch rate |
| $x_7 = \delta$ | Physical nozzle deflection angle |

The single control input is $u = \delta_{\text{cmd}}$, the commanded nozzle deflection sent to the actuator.

### 1.2 Auxiliary Quantities

Several aerodynamic quantities are computed from the translational state:

$$
V = \sqrt{x_3^2 + x_4^2}, \qquad q_\infty = \tfrac{1}{2}\cdot \rho(x_2)\cdot V^2, \qquad \alpha = x_5 - \text{atan2}(x_4,\cdot  x_3),
$$

where $V$ is the airspeed, $q_\infty$ is the dynamic pressure, $\rho(x_2)$ is the altitude-dependent air density, and $\alpha$ is the angle of attack.

### 1.3 Equations of Translational Motion

The translational dynamics are driven by thrust, gravity, and aerodynamic forces:

$$
\dot{x}_1 = x_3,
$$

$$
\dot{x}_2 = x_4,
$$

$$
\dot{x}_3 = \frac{1}{m}\bigl[F\cos(x_5 + x_7) - C_x\cdot q_\infty S_m \cos x_5 - C_y(\alpha)\cdot q_\infty S_m \sin x_5\bigr],
$$

$$
\dot{x}_4 = \frac{1}{m}\bigl[F\sin(x_5 + x_7) - mg - C_x\cdot q_\infty S_m \sin x_5 + C_y(\alpha)\cdot q_\infty S_m \cos x_5\bigr],
$$

where $F$ is the total thrust, $m$ is the mass, $g$ is gravitational acceleration, $S_m$ is the cross-sectional (midsection) area, and $C_x$, $C_y(\alpha)$ are aerodynamic coefficients.

### 1.4 Equation of Rotational Motion

The pitch dynamics are governed by the aerodynamic moment and the thrust moment arm:

$$
\dot{x}_5 = x_6,
$$

$$
\dot{x}_6 = \frac{1}{J}\bigl[-m g\cdot l_{cp}\cdot x_7 + q_\infty S_m\cdot l\cdot C_{m\alpha}\cdot \alpha\bigr],
$$

where $J$ is the moment of inertia about the lateral axis, $l$ is the reference length, $l_{cp}$ is the thrust moment arm (distance from center of mass to the nozzle pivot), and $C_{m\alpha}$ is the pitch moment coefficient. The linearization $\sin\delta \approx \delta$ is used, justified by $|\delta| \leq \delta_{\max} \leq 0.25$ rad.

### 1.5 Nozzle Actuator Dynamics

The nozzle actuator is modeled as a first-order lag with time constant $\tau_\delta$ (on the order of 10–50 ms for an electric actuator):

$$
\dot{x}_7 = \frac{1}{\tau_\delta}\bigl(-x_7 + u\bigr),
$$

where $u = \delta_{\text{cmd}}$ is the commanded nozzle angle — the only control input to the system.

### 1.6 Compact Form

Collecting all seven equations:

$$
\dot{q} = f(q, u,  t), \qquad q \in \mathbb{R}^7, \quad u \in \mathbb{R}.
$$

The explicit time dependence arises from parameters that vary during flight: mass $m(t)$ decreases due to fuel burn, thrust $F(t)$ follows the engine program (and may have discontinuities at stage separation), and air density $\rho$ changes with altitude.

### 1.7 Model Parameters

| Symbol | Meaning |
|--------|---------|
| $F$ | Total engine thrust |
| $m$ | Mass (decreasing with fuel burn) |
| $J$ | Moment of inertia about the lateral axis |
| $l$ | Reference length |
| $l_{cp}$ | Thrust moment arm from center of mass |
| $S_m$ | Cross-sectional area |
| $\rho$ | Air density (function of altitude) |
| $g$ | Gravitational acceleration |
| $\tau_\delta$ | Actuator time constant |
| $C_x, C_y(\alpha), C_{m\alpha}$ | Aerodynamic coefficients |

---

## 2. Problem Definition

### 2.1 Pitch Program

A prescribed pitch program $\vartheta_d(t) \in C^3([0, T])$ is given for the powered flight phase $[0, T]$. The program defines the desired orientation of the rocket at every instant and is known in advance together with its derivatives $\dot{\vartheta}_d$, $\ddot{\vartheta}_d$, $\dddot{\vartheta}_d$. The duration $T$ is on the order of tens of seconds to a few minutes.

### 2.2 Control Objective

Design a control law $u = \delta_{\text{cmd}}(q, t)$ such that the pitch angle $x_5(t) = \vartheta(t)$ tracks the prescribed program $\vartheta_d(t)$ with:

1. **Exponential convergence** at a prescribed rate $\lambda > 0$:

$$
|\vartheta(t) - \vartheta_d(t)| \leq C\cdot e^{-\lambda t}\cdot |\vartheta(0) - \vartheta_d(0)| + \mu, \qquad t \in [0, T],
$$

where $C > 0$ is a constant and $\mu \geq 0$ accounts for disturbances and model uncertainties.

2. **Steady-state accuracy** by a settling time $T_1 \leq T$:

$$
|\vartheta(t) - \vartheta_d(t)| \leq \varepsilon, \qquad t \in [T_1, T],
$$

where $\varepsilon > 0$ is the allowable pitch error and $T_1 \approx 5/\lambda$.

### 2.3 Cascade Structure

The key structural observation is that the control input $u$ enters the system *only* at the lowest level — the actuator equation for $\dot{x}_7$. It does not appear in the translational or rotational equations directly. This gives the pitch-relevant subsystem a strict-feedback (cascade) form:

$$
\dot{x}_5 = x_6,
$$

$$
\dot{x}_6 = f_2(x_3, x_4, x_5) + g_2\cdot x_7,
$$

$$
\dot{x}_7 = -\frac{1}{\tau_\delta}\cdot x_7 + \frac{1}{\tau_\delta}\cdot u,
$$

where

$$
f_2 = \frac{q_\infty S_m\cdot lC_{m\alpha}\cdot \alpha}{J}, \qquad g_2 = -\frac{m g\cdot l_{cp}}{J}.
$$

The translational variables $(x_1, x_2, x_3, x_4)$ do not enter the feedback loop but are used to compute $\alpha$, $V$, $q_\infty$ for aerodynamic compensation inside the controller.

### 2.4 What Makes This Problem Nontrivial

- **Three levels of indirection.** The control input $u$ acts on the nozzle angle $x_7$, which affects the pitch rate $x_6$, which in turn drives the pitch angle $x_5$. A naive single-loop controller ignoring this cascade leads to poor performance or instability.


- **Discontinuous thrust.** The thrust $F(t)$ may jump at known instants (stage separation, engine shutdown/ignition). Although $F$ does not enter the rotational equation directly, it affects the translational state, which feeds into $f_2$ through $\alpha$ and $q_\infty$. Stability of the closed loop through these switching instants must be established.

- **Finite flight time.** The powered phase has finite duration $T$, making classical asymptotic stability ($t \to \infty$) an unnatural requirement. Instead, performance specifications are stated in terms of exponential convergence rate and steady-state accuracy on $[0, T]$.

### 2.5 Assumptions

1. The nozzle deflection is small: $|x_7| \leq \delta_{\max} \leq 0.25$ rad.
2. The full state vector $q$ is measurable.
3. All model parameters ($m$, $J$, $l$, $l_{cp}$, $S_m$, $F$, $\tau_\delta$, $C_x$, $C_y$, $C_{m\alpha}$) are known.
4. The pitch program $\vartheta_d(t)$ has continuous derivatives up to third order.
5. The required nozzle deflection $\delta^{\text{des}}(t)$ lies strictly inside $[-\delta_{\max}, \delta_{\max}]$ with margin.
6. The coefficient $g_2 \neq 0$ (equivalently, $l_{cp} \neq 0$).

## 3. Control Law

For full derivations and formal proofs see [`README-derivation-adaptive.md`](README-derivation-adaptive.md).

### Extended Lyapunov function


### Control law (Certainty Equivalence)


Projection confines the estimate to physically meaningful bounds and does not violate $\dot V \leq 0$.

### Stability summary

| Claim | Status | Method |
|-------|--------|--------|

---

## 4. Algorithm

Pipeline executed at each integration timestep:


---

## 5. Experimental Setup

### Initial conditions

| Variable | Value | Notes |
|----------|-------|-------|
### Target state

$$
\vartheta^* = 30\ \text{deg}, \qquad \dot\vartheta^* = 0\ \text{rad/s}
$$

Any constant angle can be set via `theta_target_deg` in `configs/default.yaml`.

### Physical parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| $g$ | 9.81 m/s² | Gravitational acceleration |
| $m$ | 13000 kg | Rocket mass (constant) |
| $F_{\max}$ | 191763 N $= 1.5\cdot mg$ | Maximum thrust |
| $F$ | $1.5\cdot mg = 191763$ N | Fixed thrust (throttle = 1.0) |
| $J$ | 318196 kg·m² | Moment of inertia about CoM |
| $l_{cp}$ | 10.3 m | CoM-to-nozzle distance |
| $l$ | 18.0 m | Reference rocket length |
| $\delta_{\max}$ | 15 deg | Nozzle deflection limit |
| $\rho$ | 1.225 kg/m³ | Air density |
| $S_m$ | 3.14 m² | Reference cross-sectional area |
| $C_{m\alpha}^{\text{true}}$ | 1.054 rad⁻¹ | Ground truth (simulator only, not known to controller) |
| $C_x$ | 0.358 | Axial drag coefficient (known) |
| $C_{y\alpha}$ | 0.05403 deg⁻¹ | Normal force slope (linear regime, known) |


### Numerical setup

| Setting | Value |
|---------|-------|

---

## 6. Reproducibility

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m src.main --config configs/default.yaml --output-root .
```

### Module descriptions

| File | Role |
|------|------|
| `src/system.py` | Rocket parameters, aerodynamic model ($X_b$, $Y_b$, $M_b^z$), nonlinear RHS |
| `src/controller.py` | `AdaptiveCEController` (CE law + adaptation + projection) and `BaselineLyapunovController` (P1 law, no aero compensation) |
| `src/simulation.py` | `simulate_adaptive()` and `simulate_baseline()` returning full state + estimate history |
| `src/visualization.py` | All figures including parameter estimation, Lyapunov function, and comparison plot |
| `src/main.py` | Runs both controllers, writes `figures/summary.json` |

---

