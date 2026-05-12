# Planar TVC Rocket — Full-System Backstepping Landing Control

![Backstepping landing](outputs/backstepping/animations/backstepping_landing.gif)

## Overview

This repository implements a **planar thrust-vector-controlled (TVC) rocket** with a full-system backstepping landing controller. A single composite Lyapunov function covers position, attitude, and the nozzle actuator simultaneously, yielding a provably stable landing law without time-scale separation assumptions.

The control objective is **full landing**: bring the rocket from an arbitrary initial position and attitude to the landing pad at the origin with near-zero velocity and attitude.

---

## Quick Start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m src.main --config configs/backstepping.yaml --output-root outputs/backstepping
```

With animation:

```bash
python -m src.main --config configs/backstepping.yaml --output-root outputs/backstepping --animate
```

Generated outputs:

- `outputs/backstepping/figures/position_velocity.png`
- `outputs/backstepping/figures/attitude_and_gimbal.png`
- `outputs/backstepping/figures/backstepping_errors.png`
- `outputs/backstepping/figures/lyapunov_and_throttle.png`
- `outputs/backstepping/figures/coupling_terms.png`
- `outputs/backstepping/figures/planar_trajectory.png`
- `outputs/backstepping/figures/summary.json`
- `outputs/backstepping/animations/backstepping_landing.gif`

### Configuration

All parameters are set in `configs/backstepping.yaml`. Key parameters:

| Parameter | Location | Effect |
|-----------|----------|--------|
| `x`, `y`, `theta_deg`, `vx`, `vy` | `initial_state` | Starting position, attitude, velocity |
| `k_px`, `k_dx`, `k_py`, `k_dy` | `controller` | Position loop gains |
| `k_phi`, `k_omega` | `controller` | Attitude loop gains |
| `k_delta` | `controller` | Nozzle actuator gain |
| `alpha1_max` | `controller` | Virtual rate saturation [rad/s] |
| `t_final` | `experiment` | Simulation duration [s] |

---

## Repository Structure

```text
rocket_hover_energy_based/
├── README (5).md
├── README-derivation-backstepping-full (1).md
├── README-Aerodynamics (1).md
├── requirements.txt
├── configs/
│   └── backstepping.yaml
├── src/
│   ├── __init__.py
│   ├── system.py
│   ├── controller.py
│   ├── simulation.py
│   ├── visualization.py
│   └── main.py
└── outputs/
    └── backstepping/
        ├── figures/
        └── animations/
```

---

## 1. Problem Definition

The control task is to land a planar TVC rocket at the origin from an arbitrary initial condition, with nozzle deflection limited to $|\delta| \leq \delta_{\max}$ and throttle $\sigma \in [\sigma_{\min}, 1]$. All physical parameters including $C_{m\alpha}$ are **known to the controller**.

**Control objective:**

$$
(x,\, y,\, \dot x,\, \dot y,\, \vartheta,\, \dot\vartheta,\, \delta) \;\to\; (0,\, 0,\, 0,\, 0,\, 0,\, 0,\, 0) \quad \text{as } t \to \infty
$$

### Method

**Full-system backstepping** with a composite Lyapunov function:

$$
V = \tfrac{1}{2}k_{px}e_x^2 + \tfrac{1}{2}\dot x^2 + \tfrac{1}{2}k_{py}e_y^2 + \tfrac{1}{2}\dot y^2 + \tfrac{1}{2}z_\vartheta^2 + \tfrac{1}{2}z_\omega^2 + \tfrac{1}{2}z_\delta^2
$$

Four sequential backstepping steps produce virtual controls $\vartheta^*$, $\alpha_1$, $\alpha_2$ and the real nozzle command $\delta_{\mathrm{cmd}}$. Stability under actuator saturation is proved via ISS (Section 10.6 of `README-derivation-backstepping-full (1).md`).

### Assumptions

1. **Constant mass**: $\dot m = 0$, $\dot J = 0$, $\dot l_{cp} = 0$.
2. **All parameters known**: $C_{m\alpha}$, $C_x$, $C_{y\alpha}$, $J$, $l_{cp}$, $\tau_\delta$.
3. **Low-speed regime**: $V \leq 100$ m/s, linear aerodynamic coefficients apply.
4. **Small nozzle deflection**: $|\delta| \leq 0.262$ rad, enabling the linearisation $\sin(\vartheta+\delta) \approx \sin\vartheta + \delta\cos\vartheta$.
5. **First-order nozzle actuator**: $\tau_\delta \dot\delta = -\delta + \delta_{\mathrm{cmd}}$.
6. **Full state measurement**: all 10 ODE states measurable.

---

## 2. System Description

### State vector

$$
q = [x,\ y,\ \dot x,\ \dot y,\ \vartheta,\ \dot\vartheta,\ \delta,\ \alpha_2^f,\ \dot\vartheta^*_{\mathrm{filt}},\ \dot\alpha_{1,\mathrm{filt}}]^T \in \mathbb{R}^{10}
$$

Control inputs: $\sigma \in [\sigma_{\min}, 1]$ (throttle) and $\delta_{\mathrm{cmd}} \in [-\delta_{\max,\mathrm{cmd}}, \delta_{\max,\mathrm{cmd}}]$.

![Body-fixed coordinate frame](figures/BF_Sys.png)

### Notation

| Symbol | Meaning | Units |
|--------|---------|-------|
| $e_x = x - x_d$ | Horizontal position error | m |
| $e_y = y$ | Altitude error | m |
| $\vartheta$ | Pitch angle from vertical, positive rightward | rad |
| $\vartheta^*$ | Desired pitch (Step 1 virtual control) | rad |
| $z_\vartheta = \vartheta - \vartheta^*$ | Pitch error | rad |
| $\alpha_1$ | Desired angular rate (Step 2 virtual control) | rad/s |
| $z_\omega = \dot\vartheta - \alpha_1$ | Rate error | rad/s |
| $\alpha_2$ | Desired nozzle angle (Step 3 virtual control) | rad |
| $\alpha_2^f$ | Command-filtered $\alpha_2$ | rad |
| $z_\delta = \delta - \alpha_2^f$ | Nozzle error | rad |
| $\sigma$ | Throttle $\in [\sigma_{\min}, 1]$ | — |
| $F = \sigma F_{\max}$ | Thrust | N |
| $g_2 = F l_{cp}/J$ | Rotational control gain | s⁻² |
| $f_2$ | Aerodynamic pitching moment / $J$ | rad/s² |

### Equations of motion

**Translational (small-$\delta$ linearisation, A4):**

$$
m\ddot x = F\sin\vartheta + F\delta\cos\vartheta + X_b\sin\vartheta + Y_b\cos\vartheta
$$

$$
m\ddot y = F\cos\vartheta - mg - F\delta\sin\vartheta + X_b\cos\vartheta - Y_b\sin\vartheta
$$

**Rotational:**

$$
J\ddot\vartheta = F\,l_{cp}\,\delta + C_{m\alpha}\,\alpha\,q_\infty S_m\,l \quad\Longrightarrow\quad \ddot\vartheta = g_2\,\delta + f_2
$$

**Nozzle actuator:**

$$
\tau_\delta\,\dot\delta = -\delta + \delta_{\mathrm{cmd}}
$$

For the aerodynamic model, coordinate frame, and rotation matrix see [`README-Aerodynamics (1).md`](README-Aerodynamics%20(1).md).

---

## 3. Control Law

For full derivations and formal proofs see [`README-derivation-backstepping-full (1).md`](README-derivation-backstepping-full%20(1).md).

### Step 1 — Desired pitch $\vartheta^*$ and throttle $\sigma$

$$
A_x = -k_{px}e_x - k_{dx}\dot x, \qquad A_y = -k_{py}e_y - k_{dy}\dot y
$$

$$
\sigma = \mathrm{clip}\!\left(\frac{m\sqrt{A_x^2+(A_y+g)^2}}{F_{\max}},\, \sigma_{\min},\, 1\right), \qquad \vartheta^* = \mathrm{atan2}(A_x,\, A_y+g)
$$

### Step 2 — Desired angular rate $\alpha_1$

$$
z_\vartheta = \vartheta - \vartheta^*, \qquad \alpha_1 = \dot\vartheta^* - k_\vartheta z_\vartheta, \qquad z_\omega = \dot\vartheta - \alpha_1
$$

### Step 3 — Desired nozzle angle $\alpha_2$

$$
\alpha_2 = \frac{1}{g_2}\!\left(-k_\omega z_\omega - z_\vartheta - f_2 + \dot\alpha_1\right), \qquad \alpha_{2,\mathrm{sat}} = \mathrm{clip}(\alpha_2,\,-\delta_{\max},\,\delta_{\max})
$$

Command filter: $\tau_f\,\dot\alpha_2^f = \alpha_{2,\mathrm{sat}} - \alpha_2^f$, then $z_\delta = \delta - \alpha_2^f$.

### Step 4 — Nozzle command $\delta_{\mathrm{cmd}}$

$$
\delta_{\mathrm{cmd}} = \delta + \tau_\delta\!\left(\dot\alpha_2^f - g_2 z_\omega - k_\delta z_\delta\right)
$$

### Stability summary

| Claim | Status | Method |
|-------|--------|--------|
| All signals uniformly ultimately bounded | **Proved** | Composite $V$, UUB theorem (Section 10.2) |
| $\dot x, \dot y, z_\vartheta, z_\omega, z_\delta \to 0$ | **Proved** (unsaturated) | Barbalat's lemma (Section 10.3) |
| $e_x, e_y \to 0$ | **Proved** (unsaturated) | LaSalle's principle (Section 10.4) |
| UUB under all three saturations | **Proved** | ISS analysis (Section 10.6) |
| No time-scale separation needed | **Correct** | Single unified $V$ |

---

## 4. Algorithm

```
Input: state q = (x, y, ẋ, ẏ, ϑ, ϑ̇, δ, α2f, ts_dot, a1_dot)

Block A — Step 1: desired pitch and throttle
  e_x = x - x_d,  e_y = y
  A_x = -k_px*e_x - k_dx*ẋ
  A_y = -k_py*e_y - k_dy*ẏ
  a_vert = max(A_y+g, |A_x|/tan(30°), 0.5)   [safety clamp]
  σ = clip(m*sqrt(A_x²+a_vert²) / F_max, σ_min, 1)
  ϑ* = atan2(A_x, a_vert)

Block B — Aerodynamics
  α_aoa = ϑ - atan2(ẋ, ẏ)    [= ϑ if |v| < 0.1 m/s]
  f2 = C_mα * α_aoa * q_inf * S_m * l / J
  g2 = σ*F_max * l_cp / J

Block C — Step 2: virtual angular rate
  z_ϑ = wrap(ϑ - ϑ*)
  α1  = ts_dot - k_ϑ * z_ϑ
  z_ω = ϑ̇ - α1

Block D — Step 3: virtual nozzle angle
  α2_raw = (1/g2) * (-k_ω*z_ω - z_ϑ - f2 + a1_dot)
  α2_sat = clip(α2_raw, -δ_max, δ_max)

Block E — Command filter
  α2f_dot = (α2_sat - α2f) / τ_f
  z_δ = δ - α2f

Block F — Step 4: nozzle command
  δ_cmd = δ + τ_δ * (α2f_dot - g2*z_ω - k_δ*z_δ)
  δ_cmd = clip(δ_cmd, -δ_max_cmd, δ_max_cmd)

Output: σ, δ_cmd
```

---

## 5. Experimental Setup

### Initial conditions

| Variable | Value | Notes |
|----------|-------|-------|
| $x(0)$ | 50.0 m | Horizontal offset from landing pad |
| $y(0)$ | 200.0 m | Altitude |
| $\vartheta(0)$ | 30.0 deg | Large initial tilt (attitude recovery test) |
| $\dot x(0)$ | 10.0 m/s | Moving away from pad |
| $\dot y(0)$ | −20.0 m/s | Descending |
| $\dot\vartheta(0)$ | 0.0 deg/s | |
| $\delta(0)$ | 0.0 deg | |

### Target state

$$
x_d = 0\ \text{m}, \qquad y_d = 0\ \text{m}, \qquad \vartheta^* \to 0\ \text{deg at equilibrium}
$$

### Physical parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| $g$ | 9.81 m/s² | Gravitational acceleration |
| $m$ | 13000 kg | Rocket mass |
| $F_{\max}$ | 191763 N | Maximum thrust ($= 1.5\,mg$) |
| $J$ | 318197 kg·m² | Moment of inertia about CoM |
| $l_{cp}$ | 10.5 m | CoM-to-nozzle distance |
| $l$ | 18.0 m | Reference length |
| $\delta_{\max}$ | 15 deg | Nozzle deflection limit |
| $\tau_\delta$ | 0.05 s | Nozzle actuator time constant |
| $\rho$ | 1.225 kg/m³ | Air density |
| $S_m$ | 1.1304 m² | Reference cross-sectional area |
| $C_{m\alpha}$ | 1.054 rad⁻¹ | Pitching moment coefficient (known) |
| $C_x$ | 0.358 | Axial drag coefficient |
| $C_{y\alpha}$ | 3.096 rad⁻¹ | Normal force slope |
| $\sigma_{\min}$ | 0.45 | Minimum throttle |

### Controller gains

| Gain | Value | Role |
|------|-------|------|
| $k_{px}$ | 0.3 | Horizontal position |
| $k_{dx}$ | 1.5 | Horizontal velocity damping |
| $k_{py}$ | 0.5 | Vertical position |
| $k_{dy}$ | 5.0 | Vertical velocity damping |
| $k_\vartheta$ | 6.0 | Pitch error |
| $k_\omega$ | 5.0 | Angular rate error |
| $k_\delta$ | 15.0 | Nozzle error |
| $\alpha_{1,\max}$ | 1.5 rad/s | Virtual rate saturation |
| $\tau_f$ | 0.01 s | Command filter time constant |

### Numerical setup

| Setting | Value |
|---------|-------|
| Simulation time | 30.0 s |
| Integrator | `scipy.integrate.solve_ivp`, RK45 |
| Max step | 0.01 s |
| Touchdown threshold | $y \leq 0.3$ m |

---

## 6. Module Descriptions

| File | Role |
|------|------|
| `src/system.py` | 10-state ODE RHS, rocket parameters, aerodynamic model, `wrap_angle` |
| `src/controller.py` | `BacksteppingController` — all four backstepping steps, command filter, derivative filter states |
| `src/simulation.py` | `simulate()` returning full state + control history; touchdown terminal event |
| `src/visualization.py` | 6 diagnostic plots + GIF animation |
| `src/main.py` | CLI entry point: `--config`, `--output-root`, `--animate` |

---

## 7. Results Summary

| Metric | Value |
|--------|-------|
| Landing time | 24.4 s |
| Final $x$ | −0.10 m |
| Final $y$ | 0.30 m |
| Final speed | 0.13 m/s |
| Final $\vartheta$ | −0.03 deg |
| Final $\dot\vartheta$ | 0.007 deg/s |
| Max $|\vartheta|$ during flight | 35.1 deg |
| Max $|\delta|$ | 15.0 deg (briefly saturated during attitude recovery) |

**Key result:** Starting from a 30° tilt and descending at 20 m/s, the controller recovers attitude within ~6 s, converges horizontally to within 10 cm of the landing pad, and touches down at less than 0.15 m/s. The nozzle saturates briefly during the initial attitude correction but the ISS proof guarantees bounded errors throughout.

### Limitations

- Slow vertical descent in the final phase (exponential approach to $y=0$) — the position loop is tuned for stability, not time-optimality.
- $P$ and $Q$ coupling terms (theoretical backstepping feedforwards) are omitted in implementation due to saturation at high entry velocities — covered by ISS bounds.
- No rotational aerodynamic damping modelled.
- Sensor noise, wind disturbances, and 3D effects not included.

---

## 8. Figures

### Animation

![Backstepping landing](outputs/backstepping/animations/backstepping_landing.gif)

### Position and velocity

![Position and velocity](outputs/backstepping/figures/position_velocity.png)

Horizontal position $x(t)$ converging to 0, altitude $y(t)$ descending, $\dot x$ and $\dot y$ decaying to zero.

### Attitude and nozzle

![Attitude and nozzle](outputs/backstepping/figures/attitude_and_gimbal.png)

Pitch $\vartheta(t)$ recovering from 30° to 0°, angular rate $\dot\vartheta$, nozzle angle $\delta$ vs virtual reference $\alpha_2^f$.

### Backstepping error coordinates

![Backstepping errors](outputs/backstepping/figures/backstepping_errors.png)

$z_\vartheta$, $z_\omega$, $z_\delta$ — all converge to zero after the initial transient, confirming the Lyapunov bound.

### Lyapunov function and throttle

![Lyapunov and throttle](outputs/backstepping/figures/lyapunov_and_throttle.png)

$V(t)$ is non-increasing after the initial transient. Throttle $\sigma(t)$ rises to 1.0 during braking, settles near hover throttle ($\approx 0.67$) during descent.

### Planar trajectory

![Planar trajectory](outputs/backstepping/figures/planar_trajectory.png)

Full 2D trajectory from start (50 m right, 200 m altitude) to landing pad at origin.

---

## 9. Possible Extensions

- **Fuel-optimal descent**: Replace the proportional position law with a minimum-fuel guidance (e.g., powered-explicit-guidance or lossless convexification).
- **Robustness to parameter uncertainty**: Combine backstepping with online adaptive estimation of $C_{m\alpha}$ — the matched-uncertainty structure means the estimate plugs directly into the $f_2$ cancellation term in (VC2) without restructuring the proof.
- **3D extension**: Extend the planar model to 6-DOF with yaw and roll channels.
- **Wind disturbances**: Add bounded wind as an ISS disturbance input and verify the quantitative bounds from Section 10.6.

---

## 10. Notes on AI Use

AI assistance was used to help scaffold code structure, organise documentation, and check mathematical notation consistency. All equations, proofs, and physical interpretations were reviewed and verified before submission.
