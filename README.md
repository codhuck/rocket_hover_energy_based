# 2D Rocket MPC Project

This project simulates a planar TVC rocket with constant mass and constant pitch inertia.
The plant state is

```text
[x, y, vx, vy, theta, omega, delta]
```

and the control input is

```text
[delta_cmd, F]
```

The model includes gravity, thrust-vectoring, actuator dynamics, and the aerodynamic force/moment coefficients from the supplied derivation.

## What changed in the improved version

The first prototype could declare touchdown too early: it only checked height and total speed. This version adds a stricter landing gate:

```text
x error, vx, vy, total speed, theta, omega, and height
```

It also fixes the nozzle actuator integration. The actuator time constant is smaller than the controller sampling time, so the previous RK4 integration could become numerically fragile. The actuator is now updated analytically over each sample.

For the descent phase, the project now uses a hybrid mode by default:

```text
ascent: nonlinear MPC
 descent: terminal landing assist / PID-style safety layer
```

This is controlled by the config keys:

```yaml
controller:
  landing_assist: true
  assist_override_descent: true
```

To run the pure single-shooting MPC descent without the safety layer, set:

```yaml
controller:
  assist_override_descent: false
```

The hybrid mode is enabled by default because the pure SciPy single-shooting MPC is intentionally lightweight and can be too myopic for clean touchdown. A stricter research-grade version should use CasADi/IPOPT or ACADOS with hard state constraints.

## Install

```bash
python -m venv .venv
source .venv/bin/activate      # macOS / Linux
# .venv\Scripts\Activate.ps1   # Windows PowerShell
pip install -r requirements.txt
```

## Run the main mission

```bash
python -m src.main --config configs/mpc.yaml --output-root outputs/mpc
```

Expected touchdown summary for the included config is approximately:

```text
touchdown_success: true
final_x:           80.01 m
final_y:           0.80 m
final_speed:       0.32 m/s
final_theta:       -0.53 deg
landing_time:      42.0 s
```

## Quick smoke test

```bash
python -m src.main --config configs/mpc_fast.yaml --output-root outputs/mpc_fast
```

## Render animation

```bash
python -m src.main --config configs/mpc.yaml --output-root outputs/mpc --animate
```

The GIF will be saved to:

```text
outputs/mpc/animations/mpc_mission.gif
```

## Output folders

After a run, the project creates:

```text
outputs/mpc/figures/
outputs/mpc/animations/   # only with --animate
```

The most useful files are:

```text
outputs/mpc/figures/planar_trajectory.png
outputs/mpc/figures/position_velocity.png
outputs/mpc/figures/attitude_and_gimbal.png
outputs/mpc/figures/control_and_phase.png
outputs/mpc/figures/summary.json
```
