# Project 1 — Planar TVC Rocket with Lyapunov Attitude Control

![Visualisation preview](figures/rocket_visualization_preview.png)

## Overview
This repository implements a **planar thrust-vector-controlled rocket** with two attitude regulators:
- **Lyapunov attitude controller** (`AttitudeLyapunovController`);
- **cross-term Lyapunov controller** (`CrossTermLyapunovController`).

The simulation pipeline runs both controllers from the same initial condition and generates a direct comparison plot. The control objective is attitude stabilization only:
- **pitch:** `phi = 0 rad`
- **angular rate:** `omega = 0 rad/s`

Translational states evolve freely under a fixed hover throttle and are not controlled.

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
- `figures/comparison.png`
- `figures/summary.json`
- `animations/rocket_attitude_realtime.mp4`

## Repository Structure
```text
project_1_lyapunov_control_planar_tvc_rocket/
├── README.md
├── README-derivation-lyapunov.md
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

## 1. Problem Definition
The control task is to stabilize the pitch angle and angular rate of a planar rocket to zero under gravity, with a nozzle deflection angle limited to $|\delta| \leq \delta_{max}$, and fixed hover throttle.

### Method class
The method belongs to **Lyapunov-based nonlinear control**: a Lyapunov function candidate is constructed over the attitude error states, and the gimbal command is derived to guarantee $\dot{V} \leq 0$.

### Context and assumptions
1. **Constant mass**: Mass is treated as fixed, $\dot{m} = 0$. Consequently inertia moment $\dot{J} = 0$ and $J$ is constant.
2. **No aerodynamic drag**: Aerodynamic effects are not considered.
3. **Attitude-only control**: The control objective is restricted to stabilizing $\phi \to 0$, $\dot{\phi} \to 0$. Translational states $(x, y, \dot{x}, \dot{y})$ evolve freely and are not controlled. The throttle is fixed giving constant thrust $F = mg$.8
4. **Exact mass knowledge**: $m$ is assumed known precisely at each timestep. In simulation this is exact; in hardware it would require a propellant gauge.

## 2. System Description
### State, control, and notation
The state is

$$
q = [x, y, \phi, \dot{x}, \dot{y}, \dot{\phi}]^T
$$

where
- $x, y$ — inertial horizontal and vertical position (m)
- $\phi$ — pitch angle from the vertical axis, positive rightward (rad)
- $\dot{x}, \dot{y}$ — translational velocities (m/s)
- $\dot{\phi}$ — angular rate (rad/s)

The sole control input is the nozzle deflection angle:

$$
u = \delta
$$

where $\delta$ is the nozzle deflection angle measured from the rocket body axis, with $|\delta| \leq \delta_{max}$.

### Nonlinear dynamics used in the code

Parameters appearing in the equations:
- $F = mg$ — constant thrust equal to gravity compensation
- $l_{cp}$ — distance from the center of mass to the nozzle exit (m); determines the torque arm of the thrust vector
- $J_{const}$ — moment of inertia of the rocket about the center of mass (kg·m²); frozen at midpoint mass
- $g$ — gravitational acceleration, 9.81 m/s²

With constant thrust $F = mg$:

**Newton's second law** (translational):

$$
m\ddot{x} = F\sin(\phi + \delta), \qquad m\ddot{y} = F\cos(\phi + \delta) - mg
$$

Substituting $F = mg$:

$$
\ddot{x} = g\sin(\phi + \delta), \qquad \ddot{y} = g\cos(\phi + \delta) - g
$$

**Angular momentum equation** $\dot{L} = \tau$ (rotational):

$$
J_{const}\ddot{\phi} = -F \cdot l_{cp}\sin(\delta)
$$

Substituting $F = mg$:

$$
\ddot{\phi} = -\frac{mg \cdot l_{cp}}{J_{const}}\sin(\delta)
$$


## 3. Mathematical Specification

The controller stabilizes attitude only($\phi = 0$, $\dot{\phi} = 0$). The sole control input is the nozzle deflection angle $\delta$.

### Lyapunov attitude law

The attitude error and angular rate are:

$$
e_\phi = \phi, \qquad \dot{e}_\phi = \dot{\phi}
$$

The Lyapunov function candidate is:

$$
V = \frac{1}{2} k_\phi e_\phi^2 + \frac{1}{2} \dot{\phi}^2
$$

Taking the time derivative along the attitude dynamics and requiring $\dot{V} \leq 0$ yields the nozzle deflection command:

$$
\sin(\delta) = \frac{J_{const}}{F \cdot l_{cp}}\left(k_\phi e_\phi + k_\omega \dot{\phi}\right)
$$

Substituting $F = mg$:

$$
\delta = \arcsin\left(\mathrm{clamp}\left(\frac{J_{const}}{mg \cdot l_{cp}}\left(k_\phi e_\phi + k_\omega \dot{\phi}\right),\ -1,\ 1\right)\right)
$$

followed by a hard saturation to `[-delta_max, delta_max]`. This gives $\dot{V} = -k_\omega \dot{\phi}^2 \leq 0$, and by LaSalle's invariance principle all trajectories converge to $(\phi, \dot{\phi}) = (0, 0)$.

For the full derivation see [README-derivation-lyapunov.md](README-derivation-lyapunov.md).

### Cross-term Lyapunov attitude law

For the cross-term regulator, the Lyapunov candidate adds a cross term:

$$
V = \frac{1}{2}k_\phi e_\phi^2 + \frac{1}{2}\dot{\phi}^2 + c\, e_\phi \dot{\phi}
$$

Taking $\dot{V} \leq 0$ with this candidate yields:

$$
n = k_\phi e_\phi \dot{\phi} + (c + k_\omega)\dot{\phi}^2 + k_c e_\phi^2
$$

$$
d = \dot{\phi} + c\, e_\phi
$$

$$
\sin(\delta) = \frac{J_{const}}{F \cdot l_{cp}} \cdot \frac{n}{d} = \frac{J_{const}}{mg \cdot l_{cp}} \cdot \frac{n}{d}
$$

followed by the same `arcsin` and hard saturation to `[-delta_max, delta_max]`. If `|d| < eps`, the implementation falls back to the Lyapunov attitude law for numerical robustness.

### Stability interpretation
- The controller drives the rocket pitch toward $\phi_{target} = 0$ and damps angular motion via a Lyapunov argument guaranteeing $\dot{V} \leq 0$.
- Translational states $(x, y, \dot{x}, \dot{y})$ evolve freely and are not controlled.

A longer derivation note is included in [README-derivation-lyapunov.md](README-derivation-lyapunov.md).

## 4. Method Description
### Control pipeline
At every integration step the code performs the following pipeline:
1. read the current state `(x, y, phi, vx, vy, omega, mass)`;
2. compute the attitude error `e_phi = phi`;
3. compute the nozzle deflection angle `delta` from the Lyapunov attitude law;
4. propagate the nonlinear dynamics with `solve_ivp`;
5. post-process the state history into plots, an animation, and summary metrics.

### Why the controller works in practice
The Lyapunov attitude law drives `phi` and `omega` to zero by construction — $\dot{V} \leq 0$ is guaranteed for all $k_\phi > 0$, $k_\omega > 0$. Translational states evolve freely under the fixed hover throttle.

## 5. Experimental Setup
### Initial condition
- `x(0) = 0.0 m`
- `y(0) = 8.0 m`
- `phi(0) = 20.0 deg`
- `vx(0) = 0.0 m/s`
- `vy(0) = 0.0 m/s`
- `omega(0) = -8.0 deg/s`
- `m = 1.60 kg` (constant)

### Target state
- `phi_target = 0.0 rad`
- `omega_target = 0.0 rad/s`

Translational states are not controlled — position and velocity evolve freely.

### Physical parameters
- `g = 9.81 m/s^2`
- `mass = 1.60 kg`
- `F_max = 24.0 N`
- `J_const = 0.1183 kg m^2`
- `l_cp = 0.6091 m`
- `delta_max = 15 deg`

### Controller gains (Lyapunov attitude controller)
- `k_phi = 18.0`
- `k_omega = 7.0`
- `phi_target_deg = 0.0`

### Controller gains (cross-term Lyapunov)
- `k_phi = 25.0`
- `k_omega = 10.0`
- `k_c = 7.0`
- `c = 0.2`
- `phi_target_deg = 0.0`

### Numerical setup
- final simulation time: `5.0 s`
- sample count: `721`
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
- `src/controller.py` implements two attitude regulators: `AttitudeLyapunovController` and `CrossTermLyapunovController`;
- `src/simulation.py` integrates the system and provides `simulate` (Lyapunov attitude) plus `simulate_both` (Lyapunov attitude + cross-term);
- `src/visualization.py` generates all figures, including `comparison.png`, and the corrected real-time MP4 animation;
- `src/main.py` runs both regulators and writes a combined `figures/summary.json` (`pd`, `cross_term`, `comparison`).

## 7. Results Summary
Both regulators converge in attitude (`phi -> 0`, `omega -> 0`) in the default experiment. The cross-term regulator reduces peak gimbal demand but increases translational speed and drift compared with the Lyapunov attitude controller.

### Quantitative results
- **Lyapunov attitude controller**
  - final position: `(4.286100220018184, 7.317845231040285) m`
  - final pitch: `-9.289540202597493e-08 deg`
  - final angular rate: `2.6765738366331744e-06 deg/s`
  - final speed: `0.8828260180662892 m/s`
  - max absolute gimbal: `5.50247288376789 deg`
  - max speed: `0.9787758492524684 m/s`
- **Cross-term controller**
  - final position: `(7.218912011175233, 7.153153571277549) m`
  - final pitch: `4.8792597768221715e-06 deg`
  - final angular rate: `-1.911513020195674e-05 deg/s`
  - final speed: `1.5609724965023237 m/s`
  - max absolute gimbal: `2.6852208077477684 deg`
  - max speed: `1.5609724965023237 m/s`
- **Difference (cross-term − Lyapunov attitude)**
  - `final_phi_deg_diff = 4.972155178848146e-06`
  - `final_omega_deg_s_diff = -2.1791704038589914e-05`
  - `max_abs_delta_deg_diff = -2.8172520760201216 deg`
  - `max_speed_diff = 0.5821966472498553 m/s`

### What works
- Both regulators stabilize attitude and angular rate to near-zero.
- The cross-term regulator decreases peak gimbal demand compared with the Lyapunov attitude controller.
- The comparison pipeline is reproducible via one command and saves both plot and numeric comparison.
- The exported MP4 now matches the physical simulation time instead of playing too fast.

### Alternative regulator (cross-term)
- Lyapunov function:
  - `V = 0.5*k_phi*e_phi^2 + 0.5*omega^2 + c*e_phi*omega`
- Control law:
  - `numerator = k_phi*e_phi*omega + (c + k_omega)*omega^2 + k_c*e_phi^2`
  - `denominator = omega + c*e_phi`
  - `sin(delta) = (J / (alpha * F_max * l_cp)) * (numerator / denominator)`
- Singularity protection in code:
  - if `|denominator| < eps`, fallback to Lyapunov attitude law.
- Practical interpretation for current tuning:
  - smoother actuator demand (`|delta|` peak lower),
  - weaker translational performance (`max_speed` and drift higher).

### Cross-term sweep summary
The repository includes a gain sweep report generated from `figures/crossterm_sweep_phi01_compact.json` with threshold `|phi| < 0.1 rad`.

| Metric | Value |
|---|---|
| Input cases | 36 |
| Converged cases | 6 |
| Non-converged cases | 30 |
| Fixed gains during sweep | `k_phi=25`, `k_omega=10` |
| Swept gains | `k_c in [0,5]`, `c in [0.2,2.2]` |

Top converged settings by persistent settling time:

| Rank | `k_c` | `c` | First entry [s] | Settling [s] | Max abs delta [deg] | Max speed [m/s] |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0 | 0.2 | 0.4861 | 0.4861 | 10.4117 | 1.2931 |
| 2 | 1 | 0.2 | 0.4931 | 0.4931 | 9.1560 | 1.3194 |
| 3 | 2 | 0.2 | 0.5069 | 0.5069 | 7.9048 | 1.3480 |
| 4 | 3 | 0.2 | 0.5139 | 0.5139 | 6.6573 | 1.3796 |
| 5 | 4 | 0.2 | 0.5278 | 0.5278 | 5.4130 | 1.4148 |
| 6 | 5 | 0.2 | 0.5417 | 0.5417 | 4.1712 | 1.4549 |

Analysis:
- Convergence is highly concentrated in a narrow band of `c` (here only `c=0.2` converged in the tested grid).
- Small changes in `c` can move the system from fast convergence to complete non-convergence over the simulation horizon.
- Increasing `k_c` in the converged band reduces peak gimbal demand, but also tends to increase translational speed.

Conclusion:
- The cross-term regulator is significantly more sensitive to tuning than the Lyapunov attitude controller in this project setup.
- It can provide smoother actuator behavior, but practical tuning is difficult and requires careful grid search with explicit convergence checks.

### What remains limited
- The inertia and control moment arm are frozen at midpoint mass.
- Rotational drag, actuator dynamics, and sensor noise are omitted.
- The proof is quasi-static with respect to mass variation.
- Comparison currently uses one scenario (single initial condition and one gain set).

## 8. Figures and Interpretation
### State trajectories
![State trajectories](figures/state_trajectories.png)
Pitch angle and angular rate converge to zero from the initial condition without persistent oscillation.

### Attitude and gimbal command
![Attitude and gimbal](figures/attitude_and_gimbal.png)
Pitch and angular rate converge to zero, while the gimbal command remains far below the hard `15 deg` limit.

### Control effort
![Control and error](figures/control_and_error.png)
The nozzle deflection angle and angular rate decay smoothly — the closed-loop system settles without chattering.

### Planar trajectory
![Planar trajectory](figures/planar_trajectory.png)
The planar path shows the rocket's free translational motion during attitude stabilization.

### Lyapunov attitude vs cross-term comparison
![Controller comparison](figures/comparison_best.png)
Both controllers on the same axes. The cross-term controller exhibits lower peak gimbal demand than the Lyapunov attitude controller.

## 9. Animation
The project includes a corrected real-time animation:
- `animations/rocket_attitude_realtime.mp4`

The animation shows the rocket body, its current orientation, the nozzle deflection angle, the angular rate, and the current time.

## 10. Possible Extensions
- **Translational drag**: add drag forces $-\frac{\beta}{m}\dot{x}$ and $-\frac{\beta}{m}\dot{y}$ to the equations of motion and re-derive the stability analysis under dissipation.
- **Variable mass and inertia**: replace constant $m$, $J$, $l_{cp}$ with time-varying quantities updated from the fuel depletion model; extend the Lyapunov proof to the time-varying case.
- **Cascaded position control**: add an outer loop that computes a desired pitch angle $\phi_{des}$ from position errors $(e_x, e_y)$, feeding it as a reference to the inner attitude controller — enabling full $(x, y, \phi)$ stabilization.

## 11. Notes on AI Use
AI assistance was used to help scaffold code, restructure files, and polish the documentation and visualisation. The final equations, parameters, and interpretation of the plots should still be checked by the team before submission.
