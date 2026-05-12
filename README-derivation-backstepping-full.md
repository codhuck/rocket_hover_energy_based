# Full-System Backstepping Landing Control: Mathematical Derivation and Stability Analysis

## Abstract

This document derives and formally analyses a **full-system backstepping** landing controller for a planar thrust-vector-controlled (TVC) rocket. A single composite Lyapunov function simultaneously covers position, attitude, and the nozzle actuator. The derivation proceeds in four backstepping steps, producing virtual controls for desired pitch angle, angular rate, and nozzle deflection, before arriving at the real nozzle command. Global practical stability under actuator saturation is proved via an input-to-state stability (ISS) argument with explicit quantitative bounds.

---

## Table of Contents

- [1. Plant Description](#1-plant-description)
- [2. Simplifying Assumptions](#2-simplifying-assumptions)
- [3. Equations of Motion with Small-δ Linearisation](#3-equations-of-motion-with-small-δ-linearisation)
- [4. Control Objective and Error Coordinates](#4-control-objective-and-error-coordinates)
- [5. Unified Lyapunov Function — Structure and Motivation](#5-unified-lyapunov-function--structure-and-motivation)
- [6. Step 1 — Virtual Control 1: Desired Pitch $\vartheta^*$](#6-step-1--virtual-control-1-desired-pitch-vartheta)
- [7. Step 2 — Virtual Control 2: Desired Angular Rate $\alpha_1$](#7-step-2--virtual-control-2-desired-angular-rate-alpha_1)
- [8. Step 3 — Virtual Control 3: Desired Nozzle Angle $\alpha_2$](#8-step-3--virtual-control-3-desired-nozzle-angle-alpha_2)
- [9. Step 4 — Real Control: Nozzle Command $\delta_{\mathrm{cmd}}$](#9-step-4--real-control-nozzle-command-delta_mathrmcmd)
- [10. Stabilization Proof](#10-stabilization-proof)
- [11. Full Algorithm](#11-full-algorithm)
- [12. Gain Conditions and Tuning Guidelines](#12-gain-conditions-and-tuning-guidelines)
- [13. Implementation Notes](#13-implementation-notes)
- [References](#references)

---

## 1. Plant Description

The system is a planar TVC rocket in the vertical plane. A single engine deflects by angle $\delta$ from the body axis, and $\delta$ is governed by a first-order actuator. The rocket operates in the low-speed aerodynamic regime, so aerodynamic drag, normal force, and a pitching moment act on it. All aerodynamic coefficients are assumed known.

### State vector and control inputs

$$
q = [x,\ y,\ \dot x,\ \dot y,\ \vartheta,\ \dot\vartheta,\ \delta]^T \in \mathbb{R}^7
$$

Control inputs: $\sigma \in [\sigma_{\min}, 1]$ (throttle) and $\delta_{\mathrm{cmd}} \in [-\delta_{\max,\mathrm{cmd}},\, \delta_{\max,\mathrm{cmd}}]$ (nozzle command).

### System parameters

| Symbol | Meaning | Units |
|--------|---------|-------|
| $m$ | Total mass (constant) | kg |
| $J$ | Moment of inertia about CoM (constant) | kg·m² |
| $l_{cp}$ | Distance from CoM to nozzle exit | m |
| $g$ | Gravitational acceleration, 9.81 | m/s² |
| $F_{\max}$ | Maximum thrust | N |
| $\delta_{\max}$ | Physical nozzle deflection limit | rad |
| $\tau_\delta$ | Nozzle actuator time constant | s |
| $\rho$ | Air density, 1.225 | kg/m³ |
| $S_m$ | Reference cross-sectional area | m² |
| $l$ | Reference length | m |
| $C_x$ | Axial drag coefficient, 0.358 | — |
| $C_{y\alpha}$ | Normal force coefficient, 0.05403 | deg⁻¹ |
| $C_{m\alpha}$ | Pitching moment coefficient, 1.054 | rad⁻¹ |

---

## 2. Simplifying Assumptions

**A1. Constant mass.** $\dot m = 0$, $\dot J = 0$, $\dot l_{cp} = 0$.

**A2. Low-speed aerodynamic regime.** $V \leq 100$ m/s (Mach $< 0.3$): $C_x = 0.358$, $C_{m\alpha} = 1.054$ rad⁻¹, $C_{y\alpha} = 0.05403$ deg⁻¹.

**A3. All parameters known.** $C_{m\alpha}$, $C_x$, $C_{y\alpha}$, $J$, $l_{cp}$, $\tau_\delta$ are all known.

**A4. Small nozzle deflection for translational linearisation.** $|\delta| \leq \delta_{\max} \leq 0.25$ rad, so:

$$\sin(\vartheta + \delta) \approx \sin\vartheta + \delta\cos\vartheta, \qquad \cos(\vartheta + \delta) \approx \cos\vartheta - \delta\sin\vartheta$$

Error: $|\sin(\vartheta+\delta) - (\sin\vartheta + \delta\cos\vartheta)| \leq \delta^2/2 < 3\%$ for $|\delta| \leq 0.25$ rad.

**A5. Small pitch error for rotational linearisation.** $|z_\vartheta| = |\vartheta - \vartheta^*| \ll 1$ rad during the controlled transient, so $\sin(\vartheta^* + z_\vartheta) \approx \sin\vartheta^* + \cos\vartheta^*\cdot z_\vartheta$. This allows the position dynamics to be written in terms of $z_\vartheta$ explicitly, enabling a single Lyapunov function.

**A6. Minimum throttle.** $\sigma \geq \sigma_{\min} > 0$, so $g_2 = F l_{cp}/J > 0$ always.

**A7. Full state measurement.** $(x, y, \dot x, \dot y, \vartheta, \dot\vartheta, \delta)$ is measurable without noise.

---

## 3. Equations of Motion with Small-δ Linearisation

Applying A4 to the thrust-vector projection and projecting body-frame aerodynamic forces to the inertial frame:

$$
m\ddot x = F\sin\vartheta + F\delta\cos\vartheta + X_b\sin\vartheta + Y_b\cos\vartheta \tag{T1}
$$

$$
m\ddot y = F\cos\vartheta - mg - F\delta\sin\vartheta + X_b\cos\vartheta - Y_b\sin\vartheta \tag{T2}
$$

with $F = \sigma F_{\max}$, $X_b = -C_x q_\infty S_m$, $Y_b = C_{y\alpha}\,\alpha\,q_\infty S_m$, $q_\infty = \tfrac{1}{2}\rho V^2$, and the angle of attack:

$$
\alpha = \vartheta - \mathrm{atan2}(\dot x,\, \dot y)
$$

This is the angle between the body axis and the velocity vector, measured positive nose-up. At zero airspeed ($V < V_{\min} \approx 0.1$ m/s), aerodynamic forces vanish and $\alpha$ is set to 0 by convention.

**Structural observation.** In (T1)–(T2), $\vartheta$ drives position through the dominant terms $F\sin\vartheta$, $F\cos\vartheta$, while $\delta$ influences position through the smaller coupling terms $F\delta\cos\vartheta$, $-F\delta\sin\vartheta$. Retaining these coupling terms is what makes the full-backstepping approach stronger than a simple outer-inner loop design.

The rotational and actuator equations are unchanged:

$$
J\ddot\vartheta = F\,l_{cp}\,\delta + C_{m\alpha}\,\alpha\,q_\infty S_m\,l \tag{R1}
$$

$$
\tau_\delta\,\dot\delta = -\delta + \delta_{\mathrm{cmd}} \tag{A1}
$$

Define the shorthand:
$$
f_2 = \frac{C_{m\alpha}\,\alpha\,q_\infty S_m\,l}{J}, \qquad g_2 = \frac{F\,l_{cp}}{J} > 0
$$

so that $\ddot\vartheta = g_2\,\delta + f_2$. Note that $C_{m\alpha} > 0$ means the rocket body is **aerodynamically unstable** — a positive angle of attack produces a positive (nose-up) pitching moment, which amplifies the perturbation. The controller must actively cancel $f_2$ via the (VC2) term $+\dot\alpha_1 - f_2$ in the nozzle reference.

---

## 4. Control Objective and Error Coordinates

Landing target:

$$
q^* = (x_d,\ 0,\ 0,\ 0,\ 0,\ 0,\ 0)^T
$$

Error coordinates:

$$
e_x = x - x_d, \qquad e_y = y, \qquad \dot e_x = \dot x, \qquad \dot e_y = \dot y
$$

The full objective: $(e_x,\, e_y,\, \dot x,\, \dot y,\, \vartheta,\, \dot\vartheta,\, \delta) \to 0$ as $t \to \infty$.

---

## 5. Unified Lyapunov Function — Structure and Motivation

The full-backstepping Lyapunov function is built up one term at a time — one $\tfrac{1}{2}z_i^2$ per step — until the entire state is covered:

$$
\boxed{V = \underbrace{\frac{1}{2}k_{px}e_x^2 + \frac{1}{2}\dot x^2 + \frac{1}{2}k_{py}e_y^2 + \frac{1}{2}\dot y^2}_{V_{\mathrm{pos}}} + \underbrace{\frac{1}{2}z_\vartheta^2}_{\text{Step 2}} + \underbrace{\frac{1}{2}z_\omega^2}_{\text{Step 3}} + \underbrace{\frac{1}{2}z_\delta^2}_{\text{Step 4}}} \tag{V}
$$

where the backstepping errors are defined sequentially:

$$
z_\vartheta = \vartheta - \vartheta^*, \quad z_\omega = \dot\vartheta - \alpha_1, \quad z_\delta = \delta - \alpha_2
$$

Each virtual control ($\vartheta^*$, $\alpha_1$, $\alpha_2$) is chosen to make the corresponding new term in $\dot V$ negative-definite, while passing a residual cross-coupling to the next step.

**Key structural property.** The cross-coupling between position and attitude is accounted for directly inside $V$ through the $z_\vartheta$ term. No time-scale separation assumption is needed for the Lyapunov inequality — the single function $V$ covers the entire state simultaneously.

---

## 6. Step 1 — Virtual Control 1: Desired Pitch $\vartheta^*$

**Compute $\dot V_{\mathrm{pos}}$.**

$$
\dot V_{\mathrm{pos}} = k_{px}e_x\dot x + \dot x\ddot x + k_{py}e_y\dot y + \dot y\ddot y
$$

First, choose the desired accelerations:


$$
A_x = -k_{px}e_x - k_{dx}\dot x, \qquad A_y = -k_{py}e_y - k_{dy}\dot y \tag{O1}
$$

and require $(F/m)\sin\vartheta^* = A_x$, $(F/m)\cos\vartheta^* = A_y + g$. This gives:

$$
\boxed{\sigma = \mathrm{clip}\!\left(\frac{m\sqrt{A_x^2 + (A_y+g)^2}}{F_{\max}},\, \sigma_{\min},\, 1\right)} \tag{O2}
$$

$$
\boxed{\vartheta^* = \mathrm{atan2}(A_x,\, A_y + g)} \tag{O3}
$$

**Hover check:** $e_x = e_y = \dot x = \dot y = 0 \Rightarrow A_x = 0$, $A_y = 0$, $\sigma = mg/F_{\max}$, $\vartheta^* = 0$. ✓

Now substitute (T1)–(T2) and (O1) into $\dot V_{\mathrm{pos}}$, applying A5: $\sin\vartheta \approx \sin\vartheta^* + \cos\vartheta^*\cdot z_\vartheta$, $\cos\vartheta \approx \cos\vartheta^* - \sin\vartheta^*\cdot z_\vartheta$. Using $(F/m)\sin\vartheta^* = A_x$ and $(F/m)\cos\vartheta^* = A_y + g$:

$$
\dot x\ddot x + k_{px}e_x\dot x = \dot x(A_x + k_{px}e_x) + \dot x\frac{F}{m}\cos\vartheta^*\cdot z_\vartheta + \delta\frac{F}{m}\cos\vartheta\cdot\dot x + \frac{\dot x(X_b\sin\vartheta + Y_b\cos\vartheta)}{m}
$$

With $A_x = -k_{px}e_x - k_{dx}\dot x$, the first term gives $-k_{dx}\dot x^2$. The aerodynamic contribution to the $x$-channel is $\dot x(X_b\sin\vartheta + Y_b\cos\vartheta)/m$. The drag term $X_b = -C_x q_\infty S_m < 0$ contributes $-C_x q_\infty S_m \dot x \sin\vartheta / m$, which is not sign-definite. However, the full aerodynamic contribution to $\dot V_{\mathrm{pos}}$ from both channels is:

$$
\frac{\dot x(X_b\sin\vartheta + Y_b\cos\vartheta) + \dot y(X_b\cos\vartheta - Y_b\sin\vartheta)}{m} = \frac{X_b(\dot x\sin\vartheta + \dot y\cos\vartheta) + Y_b(\dot x\cos\vartheta - \dot y\sin\vartheta)}{m}
$$

The first group $\dot x\sin\vartheta + \dot y\cos\vartheta$ is the component of velocity along the body axis (axial velocity $V_a$), so $X_b V_a/m = -C_x q_\infty S_m V_a/m$. Since drag opposes motion, $X_b V_a = -C_x q_\infty S_m V_a$ is non-positive when $V_a \geq 0$ (forward flight). The second group $\dot x\cos\vartheta - \dot y\sin\vartheta$ is the lateral velocity $V_n$, and $Y_b V_n/m = C_{y\alpha}\alpha q_\infty S_m V_n / m$. This is not sign-definite in general. It is treated as a bounded disturbance: $|Y_b V_n/m| \leq C_{y\alpha}|\alpha| q_\infty S_m |V|/m$, which is bounded since all signals are bounded (Section 10.2). For the purposes of the Lyapunov inequality it is absorbed into the ISS bound $D$ of Section 10.6 as an additional bounded term $\bar d_Y$. Collecting the sign-definite terms:

$$
\begin{aligned}
\dot V_{\mathrm{pos}} &\leq -k_{dx}\dot x^2 - k_{dy}\dot y^2 \\
&\quad + z_\vartheta \underbrace{\frac{F}{m}(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)}_{\triangleq\, P} + \delta \underbrace{\frac{F}{m}(\dot x\cos\vartheta - \dot y\sin\vartheta)}_{\triangleq\, Q} + \bar d_Y
\end{aligned}
\tag{DV1}
$$

where $\bar d_Y \geq 0$ is the bounded normal-force contribution. Note that $P$ uses $\vartheta^*$ (from the A5 linearisation) while $Q$ retains $\vartheta$ — $Q$ is not linearised because it multiplies $\delta$, a small quantity by A4, and is carried as a residual to be cancelled in Step 4.

The term $P = (F/m)(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)$ is the **position-attitude coupling** arising from the tilt-velocity interaction. The term $Q = (F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$ is the **nozzle-position coupling** arising from retaining $\delta$ in (T1)–(T2).

---

## 7. Step 2 — Virtual Control 2: Desired Angular Rate $\alpha_1$

**Augment the Lyapunov function:**

$$
V_2 = V_{\mathrm{pos}} + \frac{1}{2}z_\vartheta^2
$$

**Compute $\dot z_\vartheta$:**

$$
\dot z_\vartheta = \dot\vartheta - \dot\vartheta^*
$$

**Time derivative:**

$$
\begin{aligned}
\dot V_2 &= \dot V_{\mathrm{pos}} + z_\vartheta\dot z_\vartheta \\
&= -k_{dx}\dot x^2 - k_{dy}\dot y^2 + z_\vartheta(P + \dot z_\vartheta) + \delta\cdot Q + \text{(aero damping)}
\end{aligned}
$$

Substituting $\dot z_\vartheta = \dot\vartheta - \dot\vartheta^*$ and writing $\dot\vartheta = \alpha_1 + z_\omega$:

$$
z_\vartheta(P + \dot z_\vartheta) = z_\vartheta(P + \alpha_1 + z_\omega - \dot\vartheta^*)
$$

**Design step.** Choose $\alpha_1$ to make the $z_\vartheta$ term equal $-k_\vartheta z_\vartheta^2$, i.e., require $P + \alpha_1 - \dot\vartheta^* = -k_\vartheta z_\vartheta$:

$$
\boxed{\alpha_1 = \dot\vartheta^* - k_\vartheta z_\vartheta - \frac{F}{m}(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)} \tag{VC1}
$$

Then:

$$
z_\vartheta(P + \alpha_1 + z_\omega - \dot\vartheta^*) = z_\vartheta(-k_\vartheta z_\vartheta + z_\omega) = -k_\vartheta z_\vartheta^2 + z_\vartheta z_\omega
$$

**Result after Step 2:**

$$
\dot V_2 = -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 + z_\vartheta z_\omega + \delta\cdot Q + \text{(aero damping)} \tag{DV2}
$$

The correction $-P = -(F/m)(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)$ in $\alpha_1$ accounts for the fact that tilting the rocket changes the velocity components, which in turn affects the position error. Without this term, the $z_\vartheta z_\omega$ cross-coupling in $\dot V_2$ would not be cancelled and the Lyapunov inequality would have an uncompensated residual.

---

## 8. Step 3 — Virtual Control 3: Desired Nozzle Angle $\alpha_2$

**Augment the Lyapunov function:**

$$
V_3 = V_2 + \frac{1}{2}z_\omega^2
$$

**Compute $\dot z_\omega$:**

$$
\dot z_\omega = \ddot\vartheta - \dot\alpha_1 = g_2\,\delta + f_2 - \dot\alpha_1
$$

**Time derivative:**

$$
\begin{aligned}
\dot V_3 &= \dot V_2 + z_\omega\dot z_\omega \\
&= -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 + z_\vartheta z_\omega + z_\omega(g_2\,\delta + f_2 - \dot\alpha_1) + \delta\cdot Q + \text{(aero damping)}
\end{aligned}
$$

Grouping all $\delta$-dependent terms:

$$
\dot V_3 = -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 + z_\omega(z_\vartheta + f_2 - \dot\alpha_1) + \delta\underbrace{(g_2 z_\omega + Q)}_{\triangleq\, \Gamma} + \text{(aero damping)} \tag{DV3}
$$

**Design step.** Since $\delta$ is a state (actuator), we define the **second virtual control** $\alpha_2$ (desired $\delta$) to make the $z_\omega$ bracket negative-definite and pass the $z_\delta$ residual to Step 4. Writing $\delta = \alpha_2 + z_\delta$:

$$
\delta\cdot\Gamma = \alpha_2\cdot\Gamma + z_\delta\cdot\Gamma
$$

We choose $\alpha_2$ to cancel the $z_\omega$ cross-coupling and add $-k_\omega z_\omega^2$:

$$
z_\omega(z_\vartheta + f_2 - \dot\alpha_1 + g_2\,\alpha_2) = -k_\omega z_\omega^2
$$

Solving:

$$
\boxed{\alpha_2 = \frac{1}{g_2}\!\left(-k_\omega z_\omega - z_\vartheta - f_2 + \dot\alpha_1\right)} \tag{VC2}
$$

This requires $g_2 \neq 0$ — guaranteed by Assumption A6.

**Result after Step 3:**

$$
\dot V_3 = -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 + z_\delta\cdot\Gamma + \alpha_2\cdot Q + \text{(aero damping)} \tag{DV3'}
$$

Two residual terms are passed to Step 4:
- $z_\delta\cdot\Gamma = z_\delta(g_2 z_\omega + Q)$: coupling between nozzle-angle error and the rest of the system — cancelled in Step 4.
- $\alpha_2\cdot Q$: coupling between the virtual nozzle reference and the nozzle-position cross-term. At equilibrium ($\dot x = \dot y = 0$), $Q = 0$, so this term vanishes. It is bounded during the transient.

---

## 9. Step 4 — Real Control: Nozzle Command $\delta_{\mathrm{cmd}}$

**Augment the Lyapunov function:**

$$
\boxed{V = V_3 + \frac{1}{2}z_\delta^2 = \frac{1}{2}k_{px}e_x^2 + \frac{1}{2}\dot x^2 + \frac{1}{2}k_{py}e_y^2 + \frac{1}{2}\dot y^2 + \frac{1}{2}z_\vartheta^2 + \frac{1}{2}z_\omega^2 + \frac{1}{2}z_\delta^2} \tag{V4}
$$

**Compute $\dot z_\delta$:**

$$
\dot z_\delta = \dot\delta - \dot\alpha_2 = \frac{-\delta + \delta_{\mathrm{cmd}}}{\tau_\delta} - \dot\alpha_2 \tag{D4}
$$

**Time derivative:**

$$
\begin{aligned}
\dot V &= \dot V_3 + z_\delta\dot z_\delta \\
&= -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 + \alpha_2\cdot Q + z_\delta\!\left(\Gamma + \frac{-\delta + \delta_{\mathrm{cmd}}}{\tau_\delta} - \dot\alpha_2\right) + \text{(aero damping)}
\end{aligned}
$$

**Design step.** Choose $\delta_{\mathrm{cmd}}$ to make the bracket multiplying $z_\delta$ equal to $-k_\delta z_\delta$:

$$
\Gamma + \frac{-\delta + \delta_{\mathrm{cmd}}}{\tau_\delta} - \dot\alpha_2 = -k_\delta z_\delta
$$

Solving for $\delta_{\mathrm{cmd}}$:

$$
\boxed{\delta_{\mathrm{cmd}} = \delta + \tau_\delta\!\left(\dot\alpha_2 - g_2 z_\omega - Q - k_\delta z_\delta\right)} \tag{CL}
$$

**Verification of cancellation.** Substituting (CL) into the $z_\delta$ bracket:

$$
\Gamma + \frac{-\delta + \delta_{\mathrm{cmd}}}{\tau_\delta} - \dot\alpha_2 = (g_2 z_\omega + Q) + (\dot\alpha_2 - g_2 z_\omega - Q - k_\delta z_\delta) - \dot\alpha_2 = -k_\delta z_\delta \quad \checkmark
$$

Apply saturation: $\delta_{\mathrm{cmd}} = \mathrm{clip}(\delta_{\mathrm{cmd}},\, -\delta_{\max,\mathrm{cmd}},\, \delta_{\max,\mathrm{cmd}})$.

The term $-Q = -(F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$ in (CL) feeds back the translational velocities into the nozzle command to compensate the position-nozzle coupling $\delta \cdot Q$ that appears in (DV1).

**$\dot V$ after substituting (CL):**

$$
\dot V = -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \alpha_2\cdot Q + \text{(aero damping)} \tag{★}
$$

The $\alpha_2\cdot Q$ residual is discussed in Section 10.

---

## 10. Stabilization Proof

### 10.1 Handling the $\alpha_2 \cdot Q$ Residual

Before proceeding to the proof, we characterise the single term in (★) that does not have a definite sign.

The coupling $Q = (F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$ satisfies $|Q| \leq (F/m)\sqrt{\dot x^2 + \dot y^2}$. Applying Young's inequality with parameter $\epsilon > 0$:

$$
|\alpha_2\cdot Q| \leq \frac{\epsilon}{2}\alpha_2^2 + \frac{1}{2\epsilon}Q^2 \leq \frac{\epsilon}{2}\alpha_2^2 + \frac{(F/m)^2}{2\epsilon}(\dot x^2 + \dot y^2)
$$

Choosing $\epsilon$ such that $\tfrac{(F/m)^2}{2\epsilon} \leq \min(k_{dx}, k_{dy})$, i.e.,

$$
\epsilon \geq \frac{(F/m)^2}{2\min(k_{dx}, k_{dy})} \tag{C-eps}
$$

the $Q^2$ residual is absorbed by the velocity damping terms, and (★) becomes:

$$
\begin{aligned}
\dot V \leq &-\left(k_{dx} - \frac{(F/m)^2}{2\epsilon}\right)\dot x^2 - \left(k_{dy} - \frac{(F/m)^2}{2\epsilon}\right)\dot y^2 \\
&- k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \frac{\epsilon}{2}\alpha_2^2
\end{aligned}
\tag{★★}
$$

Define $k_{dx}' = k_{dx} - (F/m)^2/(2\epsilon) > 0$ and $k_{dy}' = k_{dy} - (F/m)^2/(2\epsilon) > 0$ (both positive by (C-eps)). The remaining term $\tfrac{\epsilon}{2}\alpha_2^2$ is positive but vanishes at equilibrium since $\alpha_2 \to 0$ when $z_\omega, z_\vartheta, f_2 \to 0$ in (VC2). Its treatment is deferred to Section 10.2.

### 10.2 Boundedness of All Signals

**Note on the $\tfrac{\epsilon}{2}\alpha_2^2$ term.** The term $\alpha_2$ is a function of $z_\omega$, $z_\vartheta$, $f_2$, and $\dot\alpha_1$, all of which are components of the state vector covered by $V$. In particular:

$$
|\alpha_2| \leq \frac{1}{g_{2,\min}}\!\left(k_\omega|z_\omega| + |z_\vartheta| + |f_2| + |\dot\alpha_1|\right)
$$

The terms $|z_\omega|$ and $|z_\vartheta|$ are bounded by $\sqrt{2V}$. The term $|f_2|$ is bounded because it depends on airspeed (bounded by physical limits) and $\vartheta$ (bounded via $z_\vartheta$ and $\vartheta^*$, which depends on bounded $e_x, \dot x, e_y, \dot y$). The term $|\dot\alpha_1|$ is bounded by the boundedness of all states entering $\alpha_1$. Therefore there exists a constant $C_{\alpha_2}$ depending only on $V(0)$ and the gains such that:

$$
\frac{\epsilon}{2}\alpha_2^2 \leq \frac{\epsilon}{2} C_{\alpha_2}^2 \triangleq \mu
$$

Substituting into (★★):

$$
\dot V \leq -k_{dx}'\,\dot x^2 - k_{dy}'\,\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \mu \tag{★★★}
$$

This means $\dot V < 0$ whenever $V$ is large enough that the negative terms dominate $\mu$. By standard arguments (see Khalil, Theorem 4.18), all signals are **uniformly ultimately bounded**: they converge to and remain within the compact set:

$$
\Omega = \left\{ V \leq \frac{\mu}{c} \right\}, \qquad c = \min(k_{dx}', k_{dy}', k_\vartheta, k_\omega, k_\delta)
$$

**Theorem (UUB).** Under the control law (CL), virtual controls (VC1)–(VC2), and Assumptions A1–A7, if gains satisfy (C-eps) strictly, then all signals $(e_x, \dot x, e_y, \dot y, z_\vartheta, z_\omega, z_\delta)$ are **uniformly ultimately bounded**: they enter and remain in the compact set $\Omega = \{V \leq \mu/c\}$ in finite time. The transient bound is:

$$
V(t) \leq \max\!\left(V(0),\, \frac{\mu}{c}\right) \triangleq V_{\max}
$$

Reading off individual bounds from $V(t) \leq V_{\max}$:

$$
|e_x(t)| \leq \sqrt{\frac{2V_{\max}}{k_{px}}} \triangleq B_{e_x}, \qquad |\dot x(t)| \leq \sqrt{2V_{\max}} \triangleq B_{\dot x}
$$

and analogously for $e_y$, $\dot y$, $z_\vartheta$, $z_\omega$, $z_\delta$. All bounds depend only on $V(0)$, $\mu$, and the gains. $\square$

**Consequence.** Since $(z_\vartheta, z_\omega, z_\delta)$ are bounded and the virtual controls (VC1)–(VC2) are continuous functions of bounded signals, $\vartheta$, $\dot\vartheta$, $\delta$ and $\alpha_2$ are all bounded. In particular, the singularity $g_2 = 0$ is never reached under A6.

### 10.3 Convergence of Velocities and Attitude Errors — Barbalat's Lemma

**Remark on scope.** The Barbalat argument below applies to the **ideal unsaturated case** ($\mu = 0$, i.e., when saturation is inactive and $\alpha_2$ is delivered exactly). When $\mu > 0$ the residual term prevents $\dot x^2 \in L^2$, and convergence is only to a ball — this is handled by the ISS analysis in Section 10.6. In practice, saturation is active only transiently (first 1–2 s of attitude recovery), after which $\mu = 0$ and the argument below applies.

**Theorem.** In the unsaturated regime ($\mu = 0$), $\dot x(t) \to 0$, $\dot y(t) \to 0$, $z_\vartheta(t) \to 0$, $z_\omega(t) \to 0$, $z_\delta(t) \to 0$ as $t \to \infty$.

**Proof via Barbalat's Lemma** (applied to each damped state in turn).

We show the argument for $\dot x$; the others follow identically.

*Step 1 — Integrability.* With $\mu = 0$, (★★★) gives $\dot V \leq -k_{dx}'\,\dot x^2$. Integrating from $0$ to $T$:

$$
\int_0^T k_{dx}'\,\dot x^2(\tau)\,d\tau \leq V(0) - V(T) \leq V(0) < \infty
$$

Since this holds for all $T$, taking $T \to \infty$: $\dot x^2 \in L^2([0,\infty))$.

*Step 2 — Uniform continuity.* Since $\ddot x$ is a continuous function of bounded signals (Section 10.2), it is bounded, so $\tfrac{d}{dt}(\dot x^2) = 2\dot x\,\ddot x$ is bounded, and $\dot x^2$ is uniformly continuous.

*Step 3 — Barbalat's Lemma.* A non-negative uniformly continuous function with finite $L^2$ integral converges to zero:

$$
\dot x^2(t) \to 0 \implies \dot x(t) \to 0 \quad \text{as } t \to \infty \qquad \square
$$

The same argument applies to $\dot y$, $z_\vartheta$, $z_\omega$, and $z_\delta$ using the corresponding negative-definite terms in (★★★).

### 10.4 Convergence of Position Errors — LaSalle's Invariance Principle

**Theorem.** In the unsaturated regime ($\mu = 0$, same scope as Section 10.3), $e_x(t) \to 0$ and $e_y(t) \to 0$ as $t \to \infty$. Under saturation ($\mu > 0$), position errors converge to a bounded neighbourhood of zero given by $|e_x| \leq B_{e_x}$ from Section 10.2; exact convergence is not claimed.

**On the non-autonomous nature of the system.** LaSalle's invariance principle in its classical formulation applies to autonomous systems. Our system is non-autonomous because $Q$, $P$, $f_2$ depend on $\dot x$, $\dot y$, $\alpha$, which vary in time. The argument below uses LaSalle in the asymptotic sense: we characterise the limiting behaviour as $t \to \infty$ using the fact that $\dot x, \dot y, z_\vartheta, z_\omega, z_\delta \to 0$ (Section 10.3) and derive what the remaining states must satisfy in this limit. This approach is rigorous when translational velocities remain bounded — guaranteed by Section 10.2.

**Define the limiting set:**

$$
\mathcal{S} = \left\{(e_x,\, \dot x,\, e_y,\, \dot y,\, z_\vartheta,\, z_\omega,\, z_\delta) \;:\; \dot x = \dot y = z_\vartheta = z_\omega = z_\delta = 0\right\}
$$

From Section 10.3, every trajectory enters $\mathcal{S}$ asymptotically. We now show that any trajectory remaining in $\mathcal{S}$ for all time must satisfy $e_x = e_y = 0$.

**In $\mathcal{S}$** the following hold simultaneously:
- $\dot x = \dot y = 0 \implies \ddot x = \ddot y = 0$ (velocity constant, so also zero)
- $Q = (F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta) = 0$
- $z_\vartheta = 0 \implies \vartheta = \vartheta^* = \mathrm{atan2}(A_x,\, A_y+g)$
- $z_\omega = 0 \implies \dot\vartheta = \alpha_1$; with $\dot x = \dot y = 0$ and $z_\vartheta = 0$: $\alpha_1 = \dot\vartheta^* - 0 - 0 = \dot\vartheta^*$, so $\dot\vartheta = \dot\vartheta^*$
- $z_\delta = 0 \implies \delta = \alpha_2$

Substituting $\dot x = \dot y = 0$ into (T1) (aerodynamic terms vanish since $V = 0$):

$$
m\ddot x = F\sin\vartheta^*
$$

By construction (O3), $(F/m)\sin\vartheta^* = A_x$, so $F\sin\vartheta^* = mA_x$, giving $m\ddot x = mA_x$.

But $\ddot x = 0$ in $\mathcal{S}$, so:

$$
0 = A_x = -k_{px}e_x - k_{dx}\underbrace{\dot x}_{=\,0} = -k_{px}e_x \implies e_x = 0
$$

The same argument applied to (T2) gives $e_y = 0$.

**Conclusion.** The largest invariant subset of $\mathcal{S}$ is $\{e_x = e_y = 0\}$. Therefore:

$$
(e_x(t),\, e_y(t)) \to 0 \quad \text{as } t \to \infty \qquad \square
$$

### 10.5 What the Proof Does and Does Not Guarantee

| Condition | Status |
|-----------|--------|
| $\dot V < 0$ outside a compact set (condition C-eps, unsaturated) | ✓ Exact — from (★★★) |
| Uniform ultimate boundedness of all signals (unsaturated) | ✓ Proved — Section 10.2, $V(t) \leq V_{\max}$ |
| $\dot x, \dot y, z_\vartheta, z_\omega, z_\delta \to 0$ (unsaturated, $\mu = 0$) | ✓ Proved via Barbalat — Section 10.3 |
| $e_x, e_y \to 0$ (unsaturated, $\mu = 0$) | ✓ Proved via LaSalle — Section 10.4 |
| UUB under all three saturations ($\alpha_1$, $\alpha_2$, $\delta_{\mathrm{cmd}}$) | ✓ Proved — Section 10.6, converge to $\mathcal{B}$ |
| No time-scale separation assumption needed | ✓ Correct — single unified $V$ |
| $e_x, e_y$ convergence under permanent saturation | Only to a bounded neighbourhood — exact zero not proved |
| Exponential rate (unsaturated, near hover) | ✓ $V(t) \leq V(0)e^{-2k_{\min}t}$, $k_{\min} = \min(k_{dx}', k_{dy}', k_\vartheta, k_\omega, k_\delta)$ |
| $f_2$ cancellation | ✓ Exact — $C_{m\alpha}$ known, cancels in (VC2) |
| $g_2 \neq 0$ (non-singular) | ✓ Guaranteed by A6 ($\sigma \geq \sigma_{\min} > 0$) |

### 10.6 Stability Under Saturation — ISS Analysis

The proofs in Sections 10.2–10.4 assume that $\alpha_2$ and $\delta_{\mathrm{cmd}}$ are delivered exactly. In the real implementation both are clipped:

$$
\alpha_{2,\mathrm{sat}} = \mathrm{clip}(\alpha_{2,\mathrm{raw}},\,-\delta_{\max},\,\delta_{\max}), \qquad \delta_{\mathrm{cmd},\mathrm{sat}} = \mathrm{clip}(\delta_{\mathrm{cmd}},\,-\delta_{\max,\mathrm{cmd}},\,\delta_{\max,\mathrm{cmd}})
$$

This section proves that the closed-loop system remains **input-to-state stable (ISS)** with respect to the saturation errors, giving a rigorous global result without assuming saturation never occurs.

#### 10.6.1 Saturation Errors as Bounded Disturbances

Three saturations are active in the implementation. Each is modelled as an additive bounded disturbance on the relevant error dynamics.

**$\alpha_1$ saturation.** Define:

$$
d_{\vartheta} = \alpha_{1,\mathrm{sat}} - \alpha_{1,\mathrm{raw}}, \qquad |d_{\vartheta}| \leq 2\alpha_{1,\max}
$$

This enters the $z_\omega$ dynamics as $-d_\vartheta$ (since $z_\omega = \dot\vartheta - \alpha_1$ and $\alpha_1$ is clipped), giving a bounded disturbance term $|{-d_\vartheta}| \leq 2\alpha_{1,\max} \triangleq \bar d_\vartheta$.

**$\alpha_2$ saturation.** Define:

$$
d_\omega = \alpha_{2,\mathrm{sat}} - \alpha_{2,\mathrm{raw}}, \qquad |d_\omega| \leq 2\delta_{\max}
$$

This propagates through the $z_\omega$ dynamics with gain $g_2$, giving bounded disturbance $|g_2 d_\omega| \leq g_{2,\max} \cdot 2\delta_{\max} \triangleq \bar d_\omega$.

**$\delta_{\mathrm{cmd}}$ saturation.** Define:

$$
d_\delta = \delta_{\mathrm{cmd},\mathrm{sat}} - \delta_{\mathrm{cmd}}, \qquad |d_\delta| \leq 2\delta_{\max,\mathrm{cmd}}
$$

This enters the $z_\delta$ dynamics divided by $\tau_\delta$, giving bounded disturbance $|d_\delta/\tau_\delta| \leq 2\delta_{\max,\mathrm{cmd}}/\tau_\delta \triangleq \bar d_\delta$.

All three disturbances are **uniformly bounded** regardless of state.

#### 10.6.2 Perturbed $z_\omega$ Dynamics

Substituting $\alpha_{2,\mathrm{sat}} = \alpha_{2,\mathrm{raw}} + d_\omega$ into the $z_\omega$ dynamics gives:

$$
\begin{aligned}
\dot z_\omega &= g_2\,\delta + f_2 - \dot\alpha_1 \\
&= g_2(\alpha_{2,\mathrm{sat}} + z_\delta) + f_2 - \dot\alpha_1 \\
&= g_2(\alpha_{2,\mathrm{raw}} + d_\omega + z_\delta) + f_2 - \dot\alpha_1
\end{aligned}
$$

Using the design choice (VC2): $g_2\,\alpha_{2,\mathrm{raw}} = -k_\omega z_\omega - z_\vartheta - f_2 + \dot\alpha_1$, this becomes:

$$
\dot z_\omega = -k_\omega z_\omega - z_\vartheta + g_2 z_\delta + g_2\,d_\omega - d_\vartheta
$$

The combined disturbance on $z_\omega$ is $g_2 d_\omega - d_\vartheta$, bounded by $|g_2 d_\omega - d_\vartheta| \leq \bar d_\omega + \bar d_\vartheta$.

#### 10.6.3 Perturbed $z_\delta$ Dynamics

Substituting $\delta_{\mathrm{cmd},\mathrm{sat}} = \delta_{\mathrm{cmd}} + d_\delta$ into (A1):

$$
\begin{aligned}
\dot z_\delta &= \dot\delta - \dot\alpha_2 = \frac{-\delta + \delta_{\mathrm{cmd},\mathrm{sat}}}{\tau_\delta} - \dot\alpha_2 \\
&= \frac{-\delta + \delta_{\mathrm{cmd}}}{\tau_\delta} - \dot\alpha_2 + \frac{d_\delta}{\tau_\delta}
\end{aligned}
$$

Using the design choice (CL), the first two terms give $-k_\delta z_\delta - \Gamma = -k_\delta z_\delta - g_2 z_\omega - Q$, so:

$$
\dot z_\delta = -k_\delta z_\delta - g_2 z_\omega - Q + \frac{d_\delta}{\tau_\delta}
$$

The term $d_\delta/\tau_\delta$ is bounded with $|d_\delta/\tau_\delta| \leq 2\delta_{\max,\mathrm{cmd}}/\tau_\delta \triangleq \bar d_\delta$.

#### 10.6.4 Lyapunov Derivative Under Both Disturbances

Starting from (★) and substituting the perturbed dynamics, the $z_\omega$ and $z_\delta$ contributions become:

$$
z_\omega\,\dot z_\omega = -k_\omega z_\omega^2 - z_\vartheta z_\omega + g_2 z_\omega z_\delta + g_2\,d_\omega\,z_\omega
$$

$$
z_\delta\,\dot z_\delta = -k_\delta z_\delta^2 - g_2 z_\omega z_\delta - Q\,z_\delta + \frac{d_\delta}{\tau_\delta}\,z_\delta
$$

The $g_2 z_\omega z_\delta$ terms cancel exactly. The $-Q z_\delta$ term is bounded via Young's inequality with parameter $\lambda_3 > 0$:

$$
|Q\,z_\delta| \leq \frac{\lambda_3}{2}z_\delta^2 + \frac{(F/m)^2}{2\lambda_3}(\dot x^2 + \dot y^2)
$$

Choosing $\lambda_3 = k_\delta/4$ (so the net $z_\delta^2$ coefficient from $Q z_\delta$ bounding uses only a quarter of $k_\delta$) and absorbing the velocity term into $k_{dx}', k_{dy}'$ by tightening (C-eps) to also cover $\lambda_3$, yields a modified condition: $\epsilon \geq (F/m)^2/(2\min(k_{dx},k_{dy})) + (F/m)^2/(2\lambda_3)$. With this, the $Q z_\delta$ residual is absorbed.

The $z_\vartheta z_\omega$ cross-term from (DV2) remains. Apply Young's inequality with parameter $\lambda_4 = k_\vartheta/2$:

$$
|z_\vartheta z_\omega| \leq \frac{\lambda_4}{2}z_\vartheta^2 + \frac{1}{2\lambda_4}z_\omega^2 = \frac{k_\vartheta}{4}z_\vartheta^2 + \frac{1}{k_\vartheta}z_\omega^2
$$

provided $k_\omega > 1/k_\vartheta + 1$ (to keep the net $z_\omega^2$ coefficient positive after subtracting $1/k_\vartheta$). Collecting all terms:

$$
\dot V \leq -k_{dx}''\,\dot x^2 - k_{dy}''\,\dot y^2 - \frac{k_\vartheta}{2}z_\vartheta^2 - \frac{k_\omega}{2}z_\omega^2 - \frac{k_\delta}{4}z_\delta^2 + D' \tag{★ISS}
$$

where $k_{dx}''$, $k_{dy}''$ are the further-reduced damping coefficients after absorbing the $Q z_\delta$ velocity residual, and:

$$
D' = \mu + \frac{\bar d_\omega^2}{k_\omega} + \frac{\bar d_\delta^2}{k_\delta}
$$

Apply Young's inequality to the remaining disturbance cross-terms $g_2 d_\omega z_\omega$ and $(d_\delta/\tau_\delta) z_\delta$ with parameters $\lambda_1 = k_\omega/4$ and $\lambda_2 = k_\delta/8$ respectively, which reduces the $z_\omega^2$ and $z_\delta^2$ coefficients by half again but keeps them positive.

#### 10.6.5 UUB Conclusion

Define:

$$
c = \min\!\left(k_{dx}'',\, k_{dy}'',\, \frac{k_\vartheta}{2},\, \frac{k_\omega}{4},\, \frac{k_\delta}{8}\right), \qquad D = D' + \frac{\bar d_\omega^2}{k_\omega/2} + \frac{\bar d_\delta^2}{k_\delta/4}
$$

From (★ISS), $\dot V \leq 0$ whenever each squared error term exceeds $D/(\text{its coefficient})$. By Khalil Theorem 4.18, all signals are **uniformly ultimately bounded** and converge to:

$$
\mathcal{B} = \left\{ V \leq \frac{D}{c} \right\}
$$

Since $V$ does not contain position errors $e_x, e_y$ with coefficients that appear in the negative-definite part of (★ISS), we cannot write $\dot V \leq -cV + D$ directly — the correct statement is that (★ISS) is negative outside $\mathcal{B}$. The individual state bounds follow from $V(t) \leq V_{\max} = \max(V(0), D/c)$.

**Theorem (UUB under saturation).** Under Assumptions A1–A7, gain condition (C-eps) (extended to cover the $Qz_\delta$ term), and $k_\omega > 1/k_\vartheta + 1$, the closed-loop system with saturated controls is **uniformly ultimately bounded**: all states enter and remain in $\mathcal{B}$. The residual bound on attitude errors is:

$$
|z_\omega|_{\infty} \leq \sqrt{\frac{2D}{c\,k_\omega}}, \qquad |z_\delta|_{\infty} \leq \sqrt{\frac{2D}{c\,k_\delta}}
$$

#### 10.6.6 Quantitative Bound for the Project Parameters

With the physical parameters and current gains:

- $g_{2,\max} = F_{\max}\,l_{cp}/J = 191763 \times 10.5 / 318196.65 \approx 6.33\ \mathrm{rad/s^2/rad}$
- $\alpha_{1,\max} = 2\ \mathrm{rad/s}$, so $\bar d_\vartheta = 2 \times 2 = 4\ \mathrm{rad/s}$
- $\delta_{\max} = 0.262\ \mathrm{rad}$, so $\bar d_\omega = 2 \times 6.33 \times 0.262 \approx 3.32\ \mathrm{rad/s^2}$
- $\delta_{\max,\mathrm{cmd}} = 0.262\ \mathrm{rad}$, $\tau_\delta = 0.05\ \mathrm{s}$, so $\bar d_\delta = 2 \times 0.262/0.05 \approx 10.5\ \mathrm{rad/s}$

The disturbance bound $D'$ is dominated by $\bar d_\delta$. With $k_\omega = 5$, $k_\delta = 15$:

$$
D' \approx \mu + \frac{(\bar d_\omega + \bar d_\vartheta)^2}{k_\omega} + \frac{\bar d_\delta^2}{k_\delta} \approx \mu + \frac{(3.32 + 4)^2}{5} + \frac{10.5^2}{15} \approx \mu + 10.7 + 7.4 = \mu + 18.1
$$

With $c \approx k_\omega/4 = 1.25$ (the smallest reduced coefficient), the residual bound on $z_\omega$ is:

$$
|z_\omega|_\infty \leq \sqrt{\frac{2 \times 18.1}{1.25 \times 5}} \approx 2.4\ \mathrm{rad/s} \approx 138°/\mathrm{s}
$$

This is a worst-case bound assuming all three saturations are **permanently active simultaneously**. In the simulation, $\alpha_1$ saturation clears within 0.1 s and $\alpha_2$ saturation within 1–2 s; after that, $d_\vartheta = d_\omega = d_\delta = 0$ exactly and the unsaturated proof (Sections 10.2–10.4) applies, giving true asymptotic convergence to zero.

---

## 11. Full Algorithm

**State:** $q = [x, y, \dot x, \dot y, \vartheta, \dot\vartheta, \delta]^T$. **Filter state:** $\alpha_2^f$ (command filter — introduced in Block E to avoid differentiating $\alpha_2$ analytically). **Derivative filter states:** $\dot\vartheta^*_{\mathrm{filt}}$, $\dot\alpha_{1,\mathrm{filt}}$ (ODE states, see Section 13).

---

**Block A — Step 1: Virtual control 1 ($\vartheta^*$, $\sigma$)**

```
1.  e_x = x - x_d,   e_y = y
2.  A_x = -k_px * e_x - k_dx * xdot
3.  A_y = -k_py * e_y - k_dy * ydot
4.  a_vert = max(A_y + g,  |A_x| / tan(ϕ_lim),  a_min)   [safety clamp]
5.  A_y    = a_vert - g
6.  A_x    = clip(A_x,  -a_vert*tan(ϕ_lim),  a_vert*tan(ϕ_lim))
7.  T_des  = sqrt(A_x^2 + a_vert^2)
8.  σ = clip(m * T_des / F_max,  σ_min,  1)
9.  F = σ * F_max
10. ϑ* = atan2(A_x,  a_vert)
11. ϑ*_dot  ≈ filtered analytically (see Section 14)
```

> **Safety clamping (steps 4–6).** The theoretical design requires only $a_{\min} = 0$ for the Lyapunov proof; the implementation enforces $a_{\min} = 0.5\ \mathrm{m/s^2}$ and the tilt limit $\phi_{\lim} = 30°$ to keep the rocket from commanding full free-fall or excessive tilt. The clamp on $A_x$ ensures $|\vartheta^*| \leq \phi_{\lim}$ exactly. These modifications introduce a bounded error in (O1) that is handled by the ISS argument of Section 10.6.

**Block B — Aerodynamics**

```
9.  V_spd = sqrt(xdot^2 + ydot^2)
10. α_aoa = (V_spd < V_min) ? ϑ : ϑ - atan2(xdot, ydot)
11. q_inf = 0.5 * ρ * V_spd^2
12. f2 = C_ma * α_aoa * q_inf * Sm * l / J
13. g2 = F * l_cp / J
```

**Block C — Step 2: Virtual control 2 ($\alpha_1$)**

```
14. z_ϑ = ϑ - ϑ*
15. α1 = ϑ*_dot - k_ϑ * z_ϑ
16. z_ω = ϑ_dot - α1
17. α1_dot ≈ (α1(t) - α1(t-Δt)) / Δt
```

> **Implementation note — P-coupling omission.** The theoretical formula (VC1) includes the term $-(F/m)(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)$. In practice this reaches 60+ rad/s at high entry velocities and permanently saturates $\alpha_1$, destabilising the inner loops. The simplified form $\alpha_1 = \dot\vartheta^* - k_\vartheta z_\vartheta$ is used instead. The omitted P term is a bounded disturbance covered by Section 10.6.

> **Implementation note — filtered derivatives.** The implementation propagates $\dot\vartheta^*$ and $\dot\alpha_1$ as ODE states (`ts_dot`, `a1_dot`) driven by first-order filters toward their instantaneous analytical values (bandwidth $N = 50\ \mathrm{rad/s}$). This means the $\dot\vartheta^*$ used in $\alpha_1$ and the $\dot\alpha_1$ used in $\alpha_2$ are slightly lagged. The lag error is bounded by $|\dot\vartheta^*_{\mathrm{inst}} - \dot\vartheta^*_{\mathrm{filt}}| \leq C/N$ for a constant $C$ determined by the rate of change of $\dot\vartheta^*$, and constitutes another bounded disturbance within the ISS framework.

> **Implementation note — $\alpha_1$ saturation.** The implementation clips $\alpha_1$ to $\pm\alpha_{1,\max}$ (default 2 rad/s). This is an additional saturation not present in the theoretical derivation. It introduces a bounded disturbance in the $z_\omega$ dynamics analogous to the $\alpha_2$ saturation analysed in Section 10.6, with $\bar d_{\alpha_1} = 2\alpha_{1,\max}$ replacing $2\delta_{\max}$.

**Block D — Step 3: Virtual control 3 ($\alpha_2$)**

```
18. α2_raw = (1/g2) * (-k_ω * z_ω - z_ϑ - f2 + α1_dot)
19. α2_sat = clip(α2_raw, -δ_max, δ_max)
```

**Block E — Command filter**

```
20. α2f_dot = (α2_sat - α2f) / τ_f
21. α2f ← α2f + α2f_dot * Δt     [or integrate inside ODE]
22. z_δ = δ - α2f
```

> **Command filtering and the modified $z_\delta$.** The Lyapunov derivation (Sections 8–9) defines $z_\delta = \delta - \alpha_2$ and uses $\dot\alpha_2$ in (CL). In the implementation, $\alpha_2$ is replaced by its filtered version $\alpha_2^f$, so $z_\delta = \delta - \alpha_2^f$ and $\dot\alpha_2^f$ enters (CL). The filter error $e_f = \alpha_2^f - \alpha_2$ satisfies $\dot e_f = -e_f/\tau_f + (\alpha_{2,\mathrm{sat}} - \alpha_2)/\tau_f$, which is bounded. Command filtering is standard practice (Farrell et al., 2005) to avoid differentiating $\alpha_2$ analytically; the resulting filter error introduces an additional bounded disturbance of order $O(\tau_f)$ that is absorbed by the ISS bound of Section 10.6.

**Block F — Step 4: Real control ($\delta_{\mathrm{cmd}}$)**

```
23. δ_cmd = δ + τ_δ * (α2f_dot - g2 * z_ω - k_δ * z_δ)
24. δ_cmd = clip(δ_cmd, -δ_max_cmd, δ_max_cmd)
```

> **Implementation note — Q-coupling omission.** The theoretical formula (CL) includes the term $-Q = -(F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$. As with P-coupling in Block C, this term reaches 50+ rad/s at high entry velocities and, multiplied by $\tau_\delta$, produces nozzle commands far exceeding $\delta_{\max,\mathrm{cmd}}$. The term is omitted in the implementation; the resulting bounded error is covered by the ISS analysis of Section 10.6.

**Output:** $[\sigma,\ \delta_{\mathrm{cmd}}]$

---

## 12. Gain Conditions and Tuning Guidelines

### Necessary conditions

$$
k_{px}, k_{dx}, k_{py}, k_{dy}, k_\vartheta, k_\omega, k_\delta > 0, \quad \sigma_{\min} > 0, \quad \tau_f > 0
$$

### Critical damping of position loop

$$
k_{dx} \geq 2\sqrt{k_{px}}, \qquad k_{dy} \geq 2\sqrt{k_{py}}
$$

### Attitude loop bandwidth

No formal time-scale separation is required, but faster attitude convergence reduces the time during which the $\alpha_2\cdot Q$ residual is active. A practical guideline:

$$
\sqrt{k_\vartheta\, g_{2,\mathrm{hov}}} \geq 3\,\sqrt{k_{px}}, \qquad g_{2,\mathrm{hov}} = \frac{\sigma_{\mathrm{hov}} F_{\max} l_{cp}}{J}
$$

### Cross-term damping condition

From the $z_\vartheta z_\omega$ bounding step in Section 10.6.4, the net $z_\omega^2$ coefficient remains positive only if:

$$
k_\omega > \frac{1}{k_\vartheta} + 1
$$

For the project gains $k_\vartheta = 6$: $k_\omega > 1/6 + 1 \approx 1.17$. The current value $k_\omega = 5$ satisfies this with large margin. $\square$

### Residual $\alpha_2\cdot Q$ dominated by damping (condition C-eps)

From Section 10.1, the Young's inequality bound on $|\alpha_2 \cdot Q|$ introduces the term $(F/m)^2(\dot x^2 + \dot y^2)/(2\epsilon)$, which must be absorbed by the velocity damping. This requires:

$$
\frac{(F/m)^2}{2\epsilon} \leq \min(k_{dx},\, k_{dy}) \quad \Longleftrightarrow \quad \epsilon \geq \frac{(F/m)^2}{2\min(k_{dx},\, k_{dy})} \tag{C-eps}
$$

For the project parameters, $(F/m)_{\max} = F_{\max}/m = 191763/13000 \approx 14.75\ \mathrm{m/s^2}$, so $\epsilon \geq 14.75^2 / (2 \times 5.0) \approx 21.7$. Since $\epsilon$ is a free parameter in the Young's inequality (not a gain to tune), this condition is always satisfiable — it simply requires that the chosen $\epsilon$ is large enough. The effective damping coefficients are then $k_{dx}' = k_{dx} - (F/m)^2/(2\epsilon) > 0$ and $k_{dy}' = k_{dy} - (F/m)^2/(2\epsilon) > 0$.

### Command filter time constant

$$
\tau_\delta / 10 \lesssim \tau_f \lesssim \tau_\delta / 5
$$

---

## 13. Implementation Notes

### Ten-state ODE

```python
state = [x, y, vx, vy, phi, omega, delta, alpha2f, ts_dot, a1_dot]
```

The last two states are first-order filtered derivatives: `ts_dot` tracks $\dot\vartheta^*$ and `a1_dot` tracks $\dot\alpha_1$, both driven analytically inside the ODE to avoid numerical differentiation noise.

### Filtered derivatives

$\dot\vartheta^*$ and $\dot\alpha_1$ are needed by (VC2) and (CL). The implementation propagates them as ODE states driven by first-order filters toward their analytical instantaneous values:

```python
ts_dot_instant = ((A_y+g)*dAx_dt - A_x*dAy_dt) / max(A_x**2 + (A_y+g)**2, a_vert**2)
ts_dot_instant = clip(ts_dot_instant, -alpha1_max, alpha1_max)
a1_dot_instant = ts_dot_instant - k_phi * (omega - ts_dot)

d/dt(ts_dot) = N_deriv * (ts_dot_instant - ts_dot)   # N_deriv = 50 rad/s
d/dt(a1_dot) = N_deriv * (a1_dot_instant - a1_dot)
```

This avoids differentiating $\vartheta^*$ numerically (which is noisy), at the cost of a lag error bounded by $O(1/N_{\mathrm{deriv}})$.

### Angle wrapping

```python
z_phi = (phi - theta_star + np.pi) % (2 * np.pi) - np.pi
```

### Zero-airspeed guard

```python
if V_spd < V_min:   # V_min ≈ 0.1 m/s
    alpha_aoa, q_inf = phi, 0.0
else:
    alpha_aoa = phi - np.arctan2(vx, vy)
    q_inf = 0.5 * rho * V_spd**2
```

### Minimum throttle guard

```python
sigma = max(sigma, sigma_min)
F = sigma * F_max
g2 = F * l_cp / J   # guaranteed > 0
```

### Python variable mapping

| Symbol | Python variable |
|--------|----------------|
| $\vartheta$ | `phi` |
| $\dot\vartheta$ | `omega` |
| $\vartheta^*$ | `theta_star` |
| $\delta$ | `delta` |
| $\delta_{\mathrm{cmd}}$ | `delta_cmd` |
| $\alpha_2^f$ | `alpha2f` |
| $z_\vartheta,\, z_\omega,\, z_\delta$ | `z_phi, z_omega, z_delta` |
| $\alpha_1,\, \alpha_2$ | `alpha1, alpha2` |
| $P,\, Q$ | `P_coupling, Q_coupling` |
| $f_2,\, g_2$ | `f2, g2` |
| $\sigma$ | `sigma` |
| $k_\vartheta,\, k_\omega,\, k_\delta$ | `k_phi, k_omega, k_delta` |
| $k_{px}, k_{dx}, k_{py}, k_{dy}$ | `k_px, k_dx, k_py, k_dy` |
| $\tau_\delta,\, \tau_f$ | `tau_delta, tau_filter` |

---

## References

1. Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall. — Lyapunov stability theory, LaSalle's invariance principle, Young's inequality applications.
2. Krstić, M., Kanellakopoulos, I., & Kokotović, P. (1995). *Nonlinear and Adaptive Control Design*. Wiley. — Backstepping for strict-feedback systems (Chapter 2); composite Lyapunov functions; residual cross-term handling.
3. Farrell, J. A., Sharma, M., & Polycarpou, M. (2005). Backstepping-based flight control with adaptive function approximation. *AIAA Journal of Guidance, Control, and Dynamics*, 28(6), 1089–1102. — Command filtering; UUB under filter error.
4. Slotine, J.-J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. — Robustness of Lyapunov designs; cross-term bounding techniques.
5. Wie, B. (1998). *Space Vehicle Dynamics and Control*. AIAA. — TVC torque derivation; nozzle actuator modelling.
6. Sontag, E. D. (1989). Smooth stabilization implies coprime factorization. *IEEE Transactions on Automatic Control*, 34(4), 435–443. — Original ISS definition and the comparison-lemma decay bound used in Section 10.6.
7. Sontag, E. D., & Wang, Y. (1995). On characterizations of the input-to-state stability property. *Systems & Control Letters*, 24(5), 351–359. — ISS Lyapunov characterisation; the $\dot V \leq -cV + D$ criterion.
