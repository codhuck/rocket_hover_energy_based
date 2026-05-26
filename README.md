# Planar TVC Rocket — Model Predictive Control

![MPC mission](outputs/mpc/animations/mpc_mission.gif)

## Overview

This repository implements a **planar thrust-vector-controlled (TVC) rocket** controlled by a nonlinear **Model Predictive Controller (MPC)**.

The goal is not to track a pre-defined reference trajectory. Instead, the rocket receives only mission-level objectives:

1. start from the ground,
2. reach a target altitude,
3. return and land softly at a specified horizontal landing point.

The trajectory between these points is generated automatically by the optimizer.

The control objective is:

$$
[x, y, \dot x, \dot y, \vartheta, \dot\vartheta, \delta]^T
\rightarrow
[x_{\mathrm{land}}, 0, 0, 0, 0, 0, 0]^T
$$

where the rocket is controlled by:

$$
u = [\delta_{\mathrm{cmd}}, F]^T
$$

with $\delta_{\mathrm{cmd}}$ being the commanded nozzle deflection and $F$ the thrust magnitude.

---

## Quick Start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Main MPC simulation
python -m src.main --config configs/mpc.yaml --output-root outputs/mpc

# Main MPC simulation with GIF animation
python -m src.main --config configs/mpc.yaml --output-root outputs/mpc --animate

# Faster test configuration
python -m src.main --config configs/mpc_fast.yaml --output-root outputs/mpc_fast
```

On Linux/macOS, the helper script can also be used:

```bash
chmod +x run_project.sh
./run_project.sh
```

With animation:

```bash
./run_project.sh --animate
```

---

## Repository Structure

```text
rocket_hover_energy_based/
├── configs/
│   ├── mpc.yaml              # Main MPC configuration
│   └── mpc_fast.yaml         # Faster test configuration
├── docs/
│   ├── rocket_mpc_v2.pdf     # MPC problem statement
│   └── aerodynamics.pdf      # Aerodynamic coefficient derivation
├── src/
│   ├── system.py             # 7-state rocket dynamics and aerodynamics
│   ├── mpc_controller.py     # MPC controller and terminal landing assist
│   ├── simulation.py         # Simulation loop, mission phases, touchdown logic
│   ├── visualization.py      # Diagnostic plots and GIF animation
│   └── main.py               # CLI entry point
├── outputs/
│   ├── mpc/
│   │   ├── figures/
│   │   └── animations/
│   └── mpc_fast/
│       └── figures/
├── requirements.txt
├── run_project.sh
└── README.md
```

---

## 1. Problem Definition

The mission is a two-phase flight of a planar TVC rocket.

The rocket starts from the ground, rises to a target altitude $h_{\mathrm{target}}$, and then lands at a specified horizontal coordinate $x_{\mathrm{land}}$.

Unlike tracking control, no full reference trajectory is prescribed. The controller must find a dynamically feasible trajectory using only:

- current state,
- rocket dynamics,
- actuator limits,
- mission phase,
- terminal target.

### Mission Phases

| Phase | Objective |
|------|-----------|
| `ascent` | Reach target altitude $h_{\mathrm{target}}$ with small vertical velocity |
| `descent` | Land at $x_{\mathrm{land}}$ with near-zero velocity and attitude |

The phase switch occurs when the rocket is close to the target altitude and the vertical velocity is sufficiently small.

The touchdown condition requires the rocket to be close to the landing point and have small velocity, attitude, and angular rate.

---

## 2. System Description

### State Vector

The plant state is:

$$
s =
[x,\ y,\ \dot x,\ \dot y,\ \vartheta,\ \dot\vartheta,\ \delta]^T
\in \mathbb{R}^7
$$

where:

| State | Meaning | Units |
|------|---------|-------|
| $x$ | Horizontal position of the center of mass | m |
| $y$ | Altitude of the center of mass | m |
| $\dot x$ | Horizontal velocity | m/s |
| $\dot y$ | Vertical velocity | m/s |
| $\vartheta$ | Pitch angle from vertical, positive rightward | rad |
| $\dot\vartheta$ | Pitch angular velocity | rad/s |
| $\delta$ | Current nozzle deflection | rad |

### Control Vector

The MPC directly optimizes:

$$
u =
[\delta_{\mathrm{cmd}},\ F]^T
\in \mathbb{R}^2
$$

| Input | Meaning | Units |
|------|---------|-------|
| $\delta_{\mathrm{cmd}}$ | Commanded nozzle deflection | rad |
| $F$ | Thrust magnitude | N |

---

## 3. Equations of Motion

The rocket model uses a small-nozzle-deflection linearization.

### Translational Dynamics

$$
m\ddot x =
F\sin\vartheta
+
F\delta\cos\vartheta
+
X_b\sin\vartheta
+
Y_b\cos\vartheta
$$

$$
m\ddot y =
F\cos\vartheta
-
mg
-
F\delta\sin\vartheta
+
X_b\cos\vartheta
-
Y_b\sin\vartheta
$$

### Rotational Dynamics

$$
J\ddot\vartheta =
F l_{cp}\delta + M_b
$$

or equivalently:

$$
\ddot\vartheta =
\frac{F l_{cp}\delta + M_b}{J}
$$

### Nozzle Actuator

The nozzle actuator is modeled as a first-order system:

$$
\tau_\delta \dot\delta = -\delta + \delta_{\mathrm{cmd}}
$$

The simulator integrates the nonlinear dynamics with an RK4 step.

---

## 4. Aerodynamic Model

The rocket experiences three body-frame aerodynamic quantities:

$$
X_b = -C_x(M) q_\infty S_m
$$

$$
Y_b = C_y(\alpha, V) q_\infty S_m
$$

$$
M_b = m_z(\alpha, V) q_\infty S_m l
$$

where:

$$
q_\infty = \frac{1}{2}\rho V^2
$$

The normal force coefficient is modeled as:

$$
C_y(\alpha) =
\begin{cases}
0.05403\alpha, & V \in [0, 500]\ \mathrm{m/s} \\
0.02599\alpha + 0.008257\alpha^2, & V \in [500, 2200]\ \mathrm{m/s}
\end{cases}
$$

The pitching moment coefficient is modeled as:

$$
m_z(\alpha) =
\begin{cases}
0.01840\alpha, & V \in [0, 500]\ \mathrm{m/s} \\
0.02113\alpha - 0.0006463\alpha^2, & V \in [500, 2200]\ \mathrm{m/s}
\end{cases}
$$

---

## 5. MPC Formulation

At every simulation step, the controller solves a finite-horizon optimal control problem.

The discrete dynamics are:

$$
s_{k+1} = f_d(s_k, u_k)
$$

where $f_d$ is obtained by numerical integration of the nonlinear rocket dynamics.

### Cost Function

The cost function consists of a stage cost on control effort and a terminal penalty on the final state of the horizon:

$$
J =
\sum_{k=0}^{N-1}
\left[
R_\delta \delta_{\mathrm{cmd},k}^2
+
R_F(F_k - F_{\mathrm{hover}})^2
\right]
+
(s_N - s_{\mathrm{target}})^T
P
(s_N - s_{\mathrm{target}})
$$

where:

$$
F_{\mathrm{hover}} = mg
$$

The stage cost penalizes control usage, while the terminal cost pulls the end of the prediction horizon toward the current phase target.

### Why There Is No Tracking Trajectory

The MPC does not penalize deviation from a full reference trajectory at every step.

Instead, only the terminal state is penalized. This allows the rocket to find its own trajectory rather than being forced to follow a manually designed path.

This is the key idea of the project: the trajectory is a result of optimization, not an input to the controller.

---

## 6. Mission Targets and Terminal Weights

### Ascent Phase

During ascent, the target only fixes the altitude and selected stability-related components:

$$
s_{\mathrm{target,ascent}} =
\begin{bmatrix}
\ast \\
h_{\mathrm{target}} \\
\ast \\
0 \\
0 \\
0 \\
\ast
\end{bmatrix}
$$

The symbols $\ast$ indicate components that are intentionally left free by assigning zero terminal weights.

The horizontal position, horizontal velocity, and nozzle state are therefore not prescribed during ascent.

This allows the optimizer to choose a convenient horizontal position at the top of the trajectory.

### Descent Phase

During descent, the landing target is fully specified:

$$
s_{\mathrm{target,descent}} =
\begin{bmatrix}
x_{\mathrm{land}} \\
0 \\
0 \\
0 \\
0 \\
0 \\
0
\end{bmatrix}
$$

The terminal weights in descent strongly penalize:

- landing position error,
- horizontal velocity,
- vertical velocity,
- pitch angle,
- angular velocity.
---

## 7. Constraints

The controller respects physical and actuator limits.

### Control Constraints

$$
|\delta_{\mathrm{cmd}}| \leq \delta_{\mathrm{cmd,max}}
$$

$$
F_{\min} \leq F \leq F_{\max}
$$

### State Constraints

The simulation and controller penalize or limit:

$$
|\vartheta| \leq \vartheta_{\max}
$$

$$
|\delta| \leq \delta_{\max}
$$

$$
y \geq 0
$$

In the current implementation, control bounds are enforced directly by the optimizer, while some state limits are handled with soft penalties for numerical robustness.

---

## 8. Implementation Details

The current implementation uses a practical single-shooting nonlinear MPC formulation.

At each step:

1. the current state is measured,
2. the active mission phase is selected,
3. the MPC optimizes a sequence of future controls,
4. only the first control input is applied,
5. the rocket state is advanced using RK4,
6. the optimized sequence is shifted forward and used as a warm start.

### Solver

The controller uses `scipy.optimize` for the nonlinear finite-horizon optimization.

This keeps the project lightweight and easy to run without requiring CasADi, IPOPT, or ACADOS.

### Terminal Landing Assist

The default configuration includes a terminal landing assist during the final descent phase.

This improves touchdown quality and prevents the single-shooting solver from producing unstable or sideways touchdown behavior near the ground.

It can be disabled in the configuration:

```yaml
controller:
  assist_override_descent: false
```

With this option disabled, the project runs a more purely MPC-based descent, but touchdown quality may be worse unless the weights and horizon are further tuned.

---

## 9. Experimental Setup

### Mission Parameters

| Parameter | Value |
|----------|------:|
| $x_{\mathrm{start}}$ | 0.0 m |
| $x_{\mathrm{land}}$ | 80.0 m |
| $h_{\mathrm{target}}$ | 100.0 m |

### Physical Parameters

| Parameter | Value |
|----------|------:|
| $m$ | 13 000 kg |
| $J$ | 318 196 kg·m² |
| $g$ | 9.81 m/s² |
| $l_{cp}$ | 10.3 m |
| $l$ | 18.0 m |
| $\rho$ | 1.225 kg/m³ |
| $S_m$ | 4.5216 m² |
| $F_{\max}$ | 191 763 N |
| $\delta_{\max}$ | 15 deg |

### MPC Parameters

| Parameter | Meaning |
|----------|---------|
| $N$ | Prediction horizon length |
| $\Delta t$ | MPC time step |
| $R_\delta$ | Nozzle command penalty |
| $R_F$ | Thrust deviation penalty |
| $P$ | Terminal state penalty matrix |

The exact numerical values are defined in:

```text
configs/mpc.yaml
```

A faster lower-cost test setup is provided in:

```text
configs/mpc_fast.yaml
```

---

## 10. MPC Results

### Animation

![MPC mission](outputs/mpc/animations/mpc_mission.gif)

### Planar Trajectory

![Planar trajectory](outputs/mpc/figures/planar_trajectory.png)

The rocket ascends to the target altitude, transitions into descent, and lands near the prescribed landing point.

### Position and Velocity

![Position and velocity](outputs/mpc/figures/position_velocity.png)

The velocity is reduced near touchdown, producing a soft landing profile.

### Attitude and Nozzle

![Attitude and nozzle](outputs/mpc/figures/attitude_and_gimbal.png)

The rocket attitude returns close to vertical during the terminal landing phase. The nozzle deflection remains within its saturation limits.

### Control and Mission Phase

![Control and phase](outputs/mpc/figures/control_and_phase.png)

The plot shows thrust, nozzle command, and the active mission phase.

### Aerodynamics

![Aerodynamics](outputs/mpc/figures/aerodynamics.png)

Aerodynamic forces and moment are evaluated along the trajectory and included in the plant dynamics.

### Touchdown Summary

A typical result using the default MPC configuration is:

| Metric | Value |
|-------|------:|
| Touchdown success | true |
| Final $x$ | 80.01 m |
| Final $y$ | 0.80 m |
| Final speed | 0.32 m/s |
| Final pitch angle | −0.53 deg |
| Landing time | 42.0 s |

Detailed simulation metrics are saved to:

```text
outputs/mpc/figures/summary.json
```

---

## 11. Fast MPC Test Results

The `mpc_fast.yaml` configuration is intended for quick testing and debugging.

Run it with:

```bash
python -m src.main --config configs/mpc_fast.yaml --output-root outputs/mpc_fast
```

It produces the same figure set:

```text
outputs/mpc_fast/figures/
```

This configuration is faster, but less accurate than the main MPC setup.

---

## 12. Main Differences from the Backstepping Version

| Feature | Backstepping version | MPC version |
|--------|----------------------|-------------|
| Control philosophy | Analytic Lyapunov-based controller | Online finite-horizon optimization |
| Reference trajectory | Landing objective near origin | Mission-level ascent/descent targets |
| State dimension | Extended implementation state | 7-state plant model |
| Control inputs | Throttle and nozzle command | Thrust and nozzle command |
| Trajectory generation | Indirect from feedback law | Directly generated by optimization |
| Stability proof | Lyapunov/UUB analysis | Optimization-based receding horizon behavior |
| Solver | Closed-form control law | Numerical nonlinear optimization |

---

## 13. Possible Extensions

- Replace the `scipy.optimize` single-shooting MPC with a full CasADi/IPOPT multiple-shooting NLP.
- Add hard state constraints for attitude, altitude, and nozzle angle.
- Add fuel-optimal cost terms such as $\sum F_k$ or $\sum F_k^2$.
- Add obstacle avoidance and flight corridor constraints.
- Extend the model to 3D motion with yaw, roll, and lateral thrust-vectoring.
- Add wind disturbances and robustness tests.
- Compare MPC against the original backstepping and PID controllers under identical mission conditions.

---

## 14. Notes on AI Use

AI assistance was used to help scaffold the MPC implementation, organize the repository, generate documentation, and check mathematical notation consistency. All equations, simulation behavior, and physical interpretations were reviewed before submission.