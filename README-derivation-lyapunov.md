# Mathematical Derivations and Analysis: Planar TVC Rocket Attitude Control

This document provides detailed mathematical derivations and analysis for the planar TVC rocket attitude stabilization system. It serves as a comprehensive reference for the theoretical foundations of the Lyapunov-based control strategy implemented in this repository.

## Table of Contents

- [Plant Description](#plant-description)
  - [System Diagram](#system-diagram)
  - [State Variables and Control Inputs](#state-variables-and-control-inputs)
  - [System Parameters](#system-parameters)
  - [Equations of Motion](#equations-of-motion)
- [Simplifying Assumptions](#simplifying-assumptions)
- [Lyapunov-Based Attitude Control](#lyapunov-based-attitude-control)
  - [Control Objective](#control-objective)
  - [Lyapunov Function Candidate](#lyapunov-function-candidate)
  - [Control Law Derivation](#control-law-derivation)
  - [Solvability Condition](#solvability-condition)
- [Stability Analysis](#stability-analysis)
- [Controller Implementation Details](#controller-implementation-details)
- [References](#references)

---

## Plant Description

The system is a planar (2D) rocket with thrust vector control (TVC). A single engine is mounted at the base of the rocket. The nozzle can be deflected by a gimbal angle $\delta$ relative to the body axis, redirecting the thrust vector and generating a corrective torque about the center of mass. This is the same actuation principle used in Falcon 9 during powered descent.

### System Diagram

$$
         ^ y (vertical)
         |
    _____|_____
   |     *     |   <-- nose
   |           |
   |     •     |   <-- center of mass (CoM)
   |           |
   |___________|
         |
      [nozzle]
         \         <-- gimbal deflection δ
          \
           ↙ F    <-- thrust vector
         x (horizontal) →
$$

The pitch angle $\varphi$ is measured from the vertical (+y axis), positive rightward. The gimbal angle $\delta$ is measured from the rocket body axis, positive rightward. The thrust $F$ acts along the nozzle axis.

### State Variables and Control Inputs

The system state vector is:

$$
\mathbf{q} = [x,\; y,\; \varphi,\; \dot{x},\; \dot{y},\; \dot{\varphi}]^T
$$

where:
- $x, y$ — inertial horizontal and vertical position (m)
- $\varphi$ — pitch angle from vertical, positive rightward (rad)
- $\dot{x}, \dot{y}$ — inertial velocities (m/s)
- $\dot{\varphi}$ — angular rate (rad/s)

The sole control input is the gimbal deflection angle:

$$
u = \delta, \qquad |\delta| \leq \delta_{\max}
$$

The throttle $\alpha$ is not a control input — it is fixed at the hover value throughout (see Parameters).

### System Parameters

| Symbol | Meaning | Units |
|--------|---------|-------|
| $m$ | Total mass (constant under P1 assumption) | kg |
| $J$ | Moment of inertia about CoM (constant) | kg·m² |
| $l_{cp}$ | Distance from CoM to nozzle exit | m |
| $g$ | Gravitational acceleration, 9.81 | m/s² |
| $F_{\max}$ | Maximum thrust $= \dot{m}_{\max} \cdot v_e$ | N |
| $\delta_{\max}$ | Gimbal angle limit | rad |
| $F$ | Constant thrust $= mg$ | N |

### Equations of Motion

The throttle is fixed at the hover value so that thrust exactly balances gravity:

$$
F = mg \tag{1}
$$

The translational states $x$, $y$, $\dot{x}$, $\dot{y}$ evolve freely under this thrust — they are **not controlled in P1**:

$$
\ddot{x} = \frac{mg}{m} \sin(\varphi + \delta) = g\sin(\varphi + \delta) \tag{2}
$$

$$
\ddot{y} = \frac{mg}{m} \cos(\varphi + \delta) - g = g\cos(\varphi + \delta) - g \tag{3}
$$

Note that equations (2)–(3) are exact for all $\varphi$ and $\delta$ — the decomposition $\sin(\varphi+\delta)$ and $\cos(\varphi+\delta)$ is a kinematic identity, not a small-angle approximation. At equilibrium ($\varphi = 0$, $\delta = 0$), equation (3) gives $\ddot{y} = 0$ — gravity is exactly balanced. Equation (2) gives $\ddot{x} = 0$ — no horizontal acceleration. However any horizontal velocity accumulated during the correction phase persists, since there is no mechanism to cancel it. This is a known limitation of the attitude-only controller, stated explicitly in the Simplifying Assumptions.

Substituting $F = mg$ into the angular momentum balance about the CoM:

$$
\ddot{\varphi} = -\frac{mg \cdot l_{cp}}{J} \sin(\delta) \tag{4}
$$

The quantity $mg \cdot l_{cp} / J$ is a constant with units of rad/s² and represents the maximum angular acceleration per unit $\sin(\delta)$. The negative sign reflects the restoring convention: positive $\delta$ (nozzle deflected rightward) generates a negative (leftward) angular acceleration, which corrects positive pitch.

---

## Simplifying Assumptions

The following assumptions are made for Project 1 and must be stated explicitly in any analysis using these equations:

1. **Constant mass**: Mass is treated as fixed, $\dot{m} = 0$. Consequently $\dot{J} = 0$ and $J$, $l_{cp}$ are constants. This eliminates the variable-inertia coupling term $\dot{J}\dot{\varphi}$ from equation (4) and simplifies the Lyapunov analysis from a time-varying to a time-invariant problem. For typical mission durations where fuel mass is a small fraction of total mass, this is a reasonable first approximation.

2. **No aerodynamic drag**: Translational drag $(\beta = 0)$ and rotational aerodynamic moment $(\beta_r = 0)$ are neglected. At low velocities this is acceptable; at higher speeds drag provides additional passive damping that only helps stability.

3. **Attitude-only control**: The control objective is restricted to stabilizing $\varphi \to 0$, $\dot{\varphi} \to 0$. Translational states $(x, y, \dot{x}, \dot{y})$ evolve freely and are not controlled. The throttle is fixed at $\alpha = mg / F_{\max}$, giving constant thrust $F = mg$. Horizontal velocity accumulated during attitude correction is not cancelled — this is a stated limitation of P1.

4. **Exact mass knowledge**: $m$ is assumed known precisely at each timestep. In simulation this is exact; in hardware it would require a propellant gauge.

These assumptions reduce the full 7-state variable-mass system to a tractable 2-state attitude subsystem suitable for a clean Lyapunov stability proof.

---

## Lyapunov-Based Attitude Control

### Control Objective

The equilibrium to be stabilized is the upright hover condition:

$$
\varphi^* = 0, \quad \dot{\varphi}^* = 0
$$

We define the attitude error coordinates:

$$
e_\varphi = \varphi - \varphi^* = \varphi, \qquad \dot{e}_\varphi = \dot{\varphi}
$$

The sole control input is $\delta$. The thrust is constant at $F = mg$ (from Assumption 3), so the attitude dynamics reduce to:

$$
\ddot{\varphi} = -\frac{mg \cdot l_{cp}}{J} \sin(\delta) \tag{5}
$$

### Lyapunov Function Candidate

We propose the following quadratic Lyapunov function candidate over the attitude error states:

$$
V = \frac{1}{2} k_\varphi \, e_\varphi^2 + \frac{1}{2} \dot{\varphi}^2 \tag{6}
$$

where $k_\varphi > 0$ is a positive gain. This function has two components:
1. **Position term** $\frac{1}{2} k_\varphi e_\varphi^2$: penalizes pitch angle deviation from zero.
2. **Velocity term** $\frac{1}{2} \dot{\varphi}^2$: penalizes angular rate.

**Verification that $V$ is a valid Lyapunov candidate:**

- $V(0, 0) = 0$ — zero at the equilibrium. ✓
- $V(e_\varphi, \dot{\varphi}) > 0$ for all $(e_\varphi, \dot{\varphi}) \neq (0, 0)$ since both terms are non-negative and $k_\varphi > 0$. ✓
- $V \to \infty$ as $\|(e_\varphi, \dot{\varphi})\| \to \infty$ — radially unbounded. ✓

### Control Law Derivation

Taking the time derivative of $V$ along the system trajectories:

$$
\dot{V} = k_\varphi \, e_\varphi \, \dot{e}_\varphi + \dot{\varphi} \, \ddot{\varphi}
$$

Since $e_\varphi = \varphi$ and $\dot{e}_\varphi = \dot{\varphi}$, this becomes:

$$
\dot{V} = k_\varphi \, \varphi \, \dot{\varphi} + \dot{\varphi} \, \ddot{\varphi}
$$

Factoring out $\dot{\varphi}$:

$$
\dot{V} = \dot{\varphi} \left( k_\varphi \, \varphi + \ddot{\varphi} \right) \tag{7}
$$

Substituting the attitude dynamics (5):

$$
\dot{V} = \dot{\varphi} \left( k_\varphi \, \varphi - \frac{mg \cdot l_{cp}}{J} \sin(\delta) \right) \tag{8}
$$

To guarantee $\dot{V} \leq 0$, we require the expression in parentheses to be proportional to $-\dot{\varphi}$ with a positive coefficient. We set:

$$
k_\varphi \, \varphi - \frac{mg \cdot l_{cp}}{J} \sin(\delta) = -k_\omega \, \dot{\varphi} \tag{9}
$$

where $k_\omega > 0$ is a damping gain. Substituting (9) into (8):

$$
\dot{V} = -k_\omega \, \dot{\varphi}^2 \leq 0 \tag{10}
$$

This is negative semi-definite for all $k_\omega > 0$.

Solving equation (9) for $\delta$:

$$
\sin(\delta) = \frac{J}{mg \cdot l_{cp}} \left( k_\varphi \, \varphi + k_\omega \, \dot{\varphi} \right)
$$

$$
\boxed{\delta = \arcsin\!\left(\text{clamp}\!\left(\frac{J}{mg \cdot l_{cp}} \left( k_\varphi \, \varphi + k_\omega \, \dot{\varphi} \right),\; -1,\; 1\right)\right)} \tag{11}
$$

The coefficient $J / (mg \cdot l_{cp})$ is a positive constant determined entirely by physical parameters. The clamp operation enforces $|\sin(\delta)| \leq 1$ — ensuring the argument remains in the domain of $\arcsin$. Physically this corresponds to gimbal saturation: when the demanded correction exceeds the gimbal authority, the gimbal is held at its maximum. Gimbal angle saturation is then applied as a second constraint:

$$
\delta \leftarrow \text{clamp}(\delta,\; -\delta_{\max},\; \delta_{\max}) \tag{12}
$$

### Solvability Condition

The control law (11) requires the denominator $mg \cdot l_{cp} \neq 0$. Since $F = mg > 0$ always (the engine is on by assumption) and $l_{cp} > 0$ for any physical rocket where the CoM is above the nozzle, this condition is always satisfied. No singularity guard is needed in implementation.

To avoid gimbal saturation during normal operation, the gains should be chosen such that:

$$
\frac{J}{mg \cdot l_{cp}} \left( k_\varphi \, |\varphi|_{\max} + k_\omega \, |\dot{\varphi}|_{\max} \right) \leq 1 \tag{13}
$$

If this is violated, $\dot{V} \leq 0$ is still maintained — a saturated gimbal cannot worsen stability — but the rate of convergence slows.

---

## Stability Analysis

The construction of the control law already contains the stability argument. We summarise it here explicitly.

From (10), $\dot{V} = -k_\omega \dot{\varphi}^2 \leq 0$ along all trajectories, so $V$ is non-increasing and all trajectories are bounded. $\dot{V} = 0$ only when $\dot{\varphi} = 0$.

Applying **LaSalle's invariance principle**: in the largest invariant set where $\dot{\varphi} = 0$, the dynamics require $\ddot{\varphi} = 0$ as well. Substituting into (5):

$$
0 = -\frac{mg \cdot l_{cp}}{J} \sin(\delta)
$$

Since $mg \cdot l_{cp} / J > 0$, this gives $\sin(\delta) = 0$. Substituting back into (9) with $\dot{\varphi} = 0$:

$$
k_\varphi \, \varphi = 0 \implies \varphi = 0
$$

The largest invariant set is therefore the single point $(\varphi, \dot{\varphi}) = (0, 0)$. By LaSalle's invariance principle, **all trajectories converge asymptotically to the upright equilibrium**.

**What this guarantees**: convergence $\varphi(t) \to 0$, $\dot{\varphi}(t) \to 0$ as $t \to \infty$, from any initial condition, provided the gimbal does not saturate persistently.

**Where the result is approximate**: the constant-mass assumption (Assumption 1) means the proof strictly applies to a time-invariant system. For slowly varying mass the result holds approximately. When gimbal saturation (12) is active, $\dot{V} \leq 0$ is preserved but convergence may slow.

---

## Controller Implementation Details

The controller is implemented as a stateless function evaluated at each ODE integration step.

```python
def lyapunov_attitude_controller(state, params):
    """
    Lyapunov-based attitude controller (equations 11-12).

    Parameters
    ----------
    state  : array [x, y, phi, vx, vy, omega]
    params : dict with keys:
             m, J, l_cp, g, delta_max, k_phi, k_omega

    Returns
    -------
    delta : float  gimbal angle in [-delta_max, delta_max]
    """
    phi   = state[2]
    omega = state[5]   # phi_dot

    m         = params['m']
    J         = params['J']
    l_cp      = params['l_cp']
    g         = params['g']
    k_phi     = params['k_phi']
    k_omega   = params['k_omega']
    delta_max = params['delta_max']

    # Constant thrust — hover condition (equation 1)
    F = m * g

    # Gimbal angle from Lyapunov condition (equations 11-12)
    sin_delta = (J / (F * l_cp)) * (k_phi * phi + k_omega * omega)
    sin_delta = np.clip(sin_delta, -1.0, 1.0)       # domain guard
    delta     = np.arcsin(sin_delta)
    delta     = np.clip(delta, -delta_max, delta_max)  # gimbal saturation

    return delta
$$

**Required gain conditions for stability:**

$$
k_\varphi > 0, \quad k_\omega > 0
$$

**Recommended starting values** for a rocket with $m = 5$ kg, $l_{cp} = 1.0$ m, $J = 0.1$ kg·m²:

| Parameter | Value | Role |
|-----------|-------|------|
| $k_\varphi$ | 5.0 | Proportional attitude restoring |
| $k_\omega$ | 3.0 | Angular rate damping |
| $\delta_{\max}$ | 0.3 rad | Physical gimbal limit |

Tuning guideline: increase $k_\omega$ first if the response oscillates; increase $k_\varphi$ if convergence is too slow. Check condition (13) against the expected initial condition range to avoid persistent gimbal saturation.

**Monitoring $V(t)$ in simulation:**

```python
def lyapunov_value(state, params):
    """Compute V along the trajectory — must be monotonically decreasing."""
    phi   = state[2]
    omega = state[5]
    k_phi = params['k_phi']
    return 0.5 * k_phi * phi**2 + 0.5 * omega**2
$$

Plot $V(t)$ alongside the state trajectories. A monotonically decreasing $V(t)$ is the numerical verification of the theoretical guarantee $\dot{V} \leq 0$.

---

## References

1. Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall. — Standard reference for Lyapunov stability theory and LaSalle's invariance principle.

2. Slotine, J.-J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. — Chapter 4 for Lyapunov-based controller design methodology.

3. Wie, B. (1998). *Space Vehicle Dynamics and Control*. AIAA Education Series. — Attitude control of launch vehicles with thrust vector control.

4. Isidori, A. (1995). *Nonlinear Control Systems* (3rd ed.). Springer. — Cascade systems and timescale separation arguments.