# Project 2 — Planar TVC Rocket with Adaptive Lyapunov Attitude Control

![Adaptive controller](animations/rocket_attitude_adaptive.gif)

## Overview

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

## 1. Problem Definition


### Method class

### Context and assumptions

---

## 2. System Description

### State and control

$$
q = []^T, 
$$

$\vartheta$ is the pitch angle from the inertial vertical to $X_b$, positive rightward. $\delta$ is the nozzle deflection from $X_b$.

![Body-fixed coordinate frame](figures/BF_Sys.png)

### Notation table

| Symbol | Meaning | Units |
|--------|---------|-------|


### Auxiliary quantities

$$
v = \sqrt{\dot x^2 + \dot y^2}, \qquad
q_\infty = \tfrac{1}{2}\rho v^2, \qquad
\alpha = \vartheta - \mathrm{atan2}(\dot x,\ \dot y)
$$

At zero airspeed ($v = 0$), we set $\alpha = \vartheta$ by convention (no aerodynamic forces at zero speed).

### Aerodynamic forces and moment (body frame)

For the full aerodynamic model, coordinate frame definition, and the body-to-inertial rotation matrix see [`README-Aerodynamics.md`](README-Aerodynamics.md).

### Equations of motion

**Translational (inertial frame):**

$$

**Rotational:**

$$
J\ddot\vartheta = -F\cdot l_{cp}\sin\delta + Y(\alpha)\cdot C_{m\alpha}
$$

---

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
| $F_{\max}$ | 191763 N $= 1.5\,mg$ | Maximum thrust |
| $F$ | $1.5\,mg = 191763$ N | Fixed thrust (throttle = 1.0) |
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

## 7. Results Summary

| Metric | Lyapunov-based | Adaptive |
|--------|--------------|-------------|


### Limitations


---

## 8. Figures and Interpretation

---

### P1 Baseline — Lyapunov attitude controller (no aerodynamic compensation)

#### Animation

![P1 Baseline animation](animations/rocket_attitude_realtime.gif)

#### State trajectories

![State trajectories](figures/baseline/state_trajectories.png)

Full state over time: position $(x, y)$, pitch $\vartheta$, velocities $(\dot x, \dot y)$, angular rate $\dot\vartheta$.

#### Attitude and nozzle deflection

![Attitude and nozzle deflection](figures/baseline/attitude_and_gimbal.png)

Pitch angle $\vartheta(t)$ and nozzle deflection $\delta(t)$. Without aerodynamic compensation, the baseline fails to reach the target — the uncompensated pitching moment $Y(\alpha)\cdot C_{m\alpha}$ acts as a persistent disturbance causing increasing error (`final_theta = 26.5 deg` vs target `30 deg`).

#### Lyapunov function

![Lyapunov function](figures/baseline/lyapunov_function.png)

$V(t) = \frac{1}{2}k_\vartheta e_\vartheta^2 + \frac{1}{2}\dot\vartheta^2$

#### Control and error

![Control and error](figures/baseline/control_and_error.png)

#### Planar trajectory

![Planar trajectory](figures/baseline/planar_trajectory.png)

---

