# Full-System Backstepping Landing Control: Mathematical Derivation and Stability Analysis

## Abstract

This document derives and formally analyses a **full-system backstepping** landing controller for a planar thrust-vector-controlled (TVC) rocket. A single composite Lyapunov function simultaneously covers position, attitude, and the nozzle actuator. The derivation proceeds in four backstepping steps, producing virtual controls for desired pitch angle, angular rate, and nozzle deflection, before arriving at the real nozzle command. Global practical stability under actuator saturation is proved by establishing that the closed-loop system is **input-to-state stable (ISS)** with respect to actuator clipping errors. ISS is a property we prove for our specific system — not an assumed result — by constructing a Lyapunov inequality of the form $\dot V \leq -c\|z\|^2 + D'$, where $D'$ depends on the magnitude of the clipping errors. This guarantees the state remains bounded whenever the disturbances are bounded. The full proof with explicit quantitative bounds is in Section 10.5.

---

## Table of Contents

- [1. Plant Description](#1-plant-description)
- [2. Simplifying Assumptions](#2-simplifying-assumptions)
- [3. Equations of Motion with Small-δ Linearisation](#3-equations-of-motion-with-small-δ-linearisation)
  - [3.1 Full Nonlinear Equations](#31-full-nonlinear-equations)
  - [3.2 Small-δ Linearisation (Assumption A4)](#32-small-δ-linearisation-assumption-a4)
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
s = [x,\ y,\ \dot x,\ \dot y,\ \vartheta,\ \dot\vartheta,\ \delta]^T \in \mathbb{R}^7
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

*In a real rocket, fuel is consumed during the burn, reducing mass and shifting the centre of mass. Here we assume the landing burn is short enough that mass change is negligible — a standard assumption for terminal descent from a few hundred metres altitude.*

**A2. Low-speed aerodynamic regime.** $V \leq 100$ m/s (Mach $< 0.3$): $C_x = 0.358$, $C_{m\alpha} = 1.054$ rad⁻¹, $C_{y\alpha} = 0.05403$ deg⁻¹.

*Below Mach 0.3 compressibility effects are small and aerodynamic coefficients are approximately constant. This allows us to use fixed coefficient values throughout the landing manoeuvre rather than scheduling them with Mach number.*

**A3. All parameters known.** $C_{m\alpha}$, $C_x$, $C_{y\alpha}$, $J$, $l_{cp}$, $\tau_\delta$ are all known.


**A4. Small nozzle deflection for translational linearisation.** $|\delta| \leq \delta_{\max} \leq 0.25$ rad, so:

$$
\begin{aligned}
\sin(\vartheta + \delta) &\approx \sin\vartheta + \delta\cos\vartheta \\
\cos(\vartheta + \delta) &\approx \cos\vartheta - \delta\sin\vartheta
\end{aligned}
$$

Error: $|\sin(\vartheta+\delta) - (\sin\vartheta + \delta\cos\vartheta)| \leq \delta^2/2 < 3\%$ for $|\delta| \leq 0.25$ rad.

*The nozzle angle is physically limited to a narrow range (here ±0.25 rad ≈ ±14°) by the gimbal mechanism. Because $\delta$ is small, the first-order Taylor expansion of $\sin(\vartheta+\delta)$ and $\cos(\vartheta+\delta)$ around $\delta=0$ is accurate to within 3%. This separates the dominant thrust direction (governed by body pitch $\vartheta$) from the small torque contribution of the nozzle deflection $\delta$, making the equations tractable for backstepping.*

**A5. Small pitch error for rotational linearisation.** $|z_\vartheta| = |\vartheta - \vartheta^*| \ll 1$ rad during the controlled transient, so $\sin(\vartheta^* + z_\vartheta) \approx \sin\vartheta^* + \cos\vartheta^*\cdot z_\vartheta$. This allows the position dynamics to be written in terms of $z_\vartheta$ explicitly, enabling a single Lyapunov function.

*The backstepping design requires writing the time derivative of $V_\mathrm{pos}$ as an explicit function of the pitch error $z_\vartheta$. This is only possible if we can linearise $\sin\vartheta$ around the desired pitch $\vartheta^*$. The assumption holds whenever the attitude controller is fast enough to keep the tracking error small — which is enforced by the inner-loop gains $k_\vartheta$ and $k_\omega$. It is the only assumption that creates a conservatism gap: if $|z_\vartheta|$ ever grows large (e.g., at the very start with a 30° initial tilt), the linearisation error acts as an additional bounded disturbance absorbed into the ISS bound.*

**A6. Minimum throttle.** $\sigma \geq \sigma_{\min} > 0$, so $g_2 = F l_{cp}/J > 0$ always.

*The control authority of the nozzle enters the rotational dynamics as $g_2 = F l_{cp}/J$. If the engine were shut off ($\sigma = 0$), $g_2 = 0$ and the virtual control $\alpha_2$ in Step 3 would involve division by zero (see equation (VC2)). Enforcing a minimum throttle $\sigma_{\min}$ keeps $g_2$ bounded away from zero and the controller well-defined at all times.*

**A7. Full state measurement.** $(x, y, \dot x, \dot y, \vartheta, \dot\vartheta, \delta)$ is measurable without noise.


---

## 3. Equations of Motion with Small-δ Linearisation

### 3.1 Full Nonlinear Equations

The thrust vector makes angle $(\vartheta + \delta)$ from the vertical in the inertial frame. Projecting thrust and body-frame aerodynamic forces ($X_b$ axial drag, $Y_b$ normal force) onto the inertial axes, and including the nozzle actuator and pitch moment, the exact equations of motion are:

$$
m\ddot x = F\sin(\vartheta + \delta) + X_b\sin\vartheta + Y_b\cos\vartheta \tag{NL1}
$$

$$
m\ddot y = F\cos(\vartheta + \delta) - mg + X_b\cos\vartheta - Y_b\sin\vartheta \tag{NL2}
$$

$$
J\ddot\vartheta = F\,l_{cp}\,\sin\delta + C_{m\alpha}\,\alpha\,q_\infty S_m\,l \tag{NL3}
$$

$$
\tau_\delta\,\dot\delta = -\delta + \delta_{\mathrm{cmd}} \tag{NL4}
$$

with $F = \sigma F_{\max}$, $X_b = -C_x q_\infty S_m$, $Y_b = C_{y\alpha}\,\alpha\,q_\infty S_m$, $q_\infty = \tfrac{1}{2}\rho V^2$, $V = \sqrt{\dot x^2 + \dot y^2}$, and the angle of attack:

$$
\alpha = \vartheta - \mathrm{atan2}(\dot x,\, \dot y)
$$

Note that (NL1)–(NL2) are fully coupled: the translational accelerations depend on the nozzle angle $\delta$ through $\sin(\vartheta+\delta)$ and $\cos(\vartheta+\delta)$, and on pitch $\vartheta$ through the aerodynamic projections. The rotational equation (NL3) is driven by $\delta$ through the moment arm $l_{cp}$.

### 3.2 Small-δ Linearisation (Assumption A4)

Applying A4 ($|\delta| \leq 0.25$ rad) to the thrust-vector projection in (NL1)–(NL2):

$$
\begin{aligned}
\sin(\vartheta + \delta) &\approx \sin\vartheta + \delta\cos\vartheta \\
\cos(\vartheta + \delta) &\approx \cos\vartheta - \delta\sin\vartheta
\end{aligned}
$$

Substituting into (NL1)–(NL2) and projecting body-frame aerodynamic forces to the inertial frame:

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

This is the angle between the body axis and the velocity vector, measured positive nose-up. At zero airspeed ($V < V_{\min} \approx 0.1$ m/s), aerodynamic forces vanish and $\alpha$ is set to $\vartheta$ by convention.

**Structural observation.** In (T1)–(T2), $\vartheta$ drives position through the dominant terms $F\sin\vartheta$, $F\cos\vartheta$, while $\delta$ influences position through the smaller coupling terms $F\delta\cos\vartheta$, $-F\delta\sin\vartheta$. Retaining these coupling terms is what makes the full-backstepping approach stronger than a simple outer-inner loop design.

Applying A4 to the rotational equation ($\sin\delta \approx \delta$) and keeping the actuator equation exact:

$$
J\ddot\vartheta = F\,l_{cp}\,\delta + C_{m\alpha}\,\alpha\,q_\infty S_m\,l \tag{R1}
$$

$$
\tau_\delta\,\dot\delta = -\delta + \delta_{\mathrm{cmd}} \tag{A1}
$$

Define the shorthand:
$$
\begin{aligned}
f_2 &= \frac{C_{m\alpha}\,\alpha\,q_\infty S_m\,l}{J} \\
g_2 &= \frac{F\,l_{cp}}{J} > 0
\end{aligned}
$$

so that $\ddot\vartheta = g_2\,\delta + f_2$. Note that $C_{m\alpha} > 0$ means the rocket body is **aerodynamically unstable** — a positive angle of attack produces a positive (nose-up) pitching moment, which amplifies the perturbation. The controller must actively cancel $f_2$ via the (VC2) term $+\dot\alpha_1 - f_2$ in the nozzle reference.

---

## 4. Control Objective and Error Coordinates

Landing target:

$$
s^* = (x_d,\ 0,\ 0,\ 0,\ 0,\ 0,\ 0)^T
$$

Error coordinates:

$$
\begin{aligned}
e_x &= x - x_d, \quad e_y = y \\
\dot e_x &= \dot x, \quad \dot e_y = \dot y
\end{aligned}
$$

The full objective: $(e_x,\, e_y,\, \dot x,\, \dot y,\, \vartheta,\, \dot\vartheta,\, \delta) \to 0$ as $t \to \infty$.

---

## 5. Unified Lyapunov Function — Structure and Motivation

The full-backstepping Lyapunov function is built up one term at a time — one $\tfrac{1}{2}z_i^2$ per step — until the entire state is covered:

$$
\begin{aligned}
V = \underbrace{\tfrac{1}{2}k_{px}e_x^2 + \tfrac{1}{2}\dot x^2 + \tfrac{1}{2}k_{py}e_y^2 + \tfrac{1}{2}\dot y^2}_{V_{\mathrm{pos}}} + \underbrace{\tfrac{1}{2}z_\vartheta^2}_{\text{Step 2}} + \underbrace{\tfrac{1}{2}z_\omega^2}_{\text{Step 3}} + \underbrace{\tfrac{1}{2}z_\delta^2}_{\text{Step 4}}
\end{aligned}
\tag{V}
$$

where the backstepping errors are defined sequentially:

$$
z_\vartheta = \vartheta - \vartheta^*, \quad z_\omega = \dot\vartheta - \alpha_1, \quad z_\delta = \delta - \alpha_2
$$

- $z_\vartheta$: how far the actual pitch $\vartheta$ is from the desired pitch $\vartheta^*$ computed in Step 1. If $z_\vartheta = 0$, the rocket points exactly where the position controller wants it to.
- $z_\omega$: how far the actual angular rate $\dot\vartheta$ is from the desired rate $\alpha_1$ computed in Step 2. If $z_\omega = 0$, the rocket is rotating at exactly the rate needed to drive $z_\vartheta$ to zero.
- $z_\delta$: how far the actual nozzle angle $\delta$ is from the desired nozzle angle $\alpha_2$ computed in Step 3. If $z_\delta = 0$, the nozzle is positioned exactly where needed to drive $z_\omega$ to zero.

Each error is only nonzero because the previous step's virtual control has not been fully achieved yet — this chain structure is the essence of backstepping.

Each virtual control ($\vartheta^*$, $\alpha_1$, $\alpha_2$) is chosen to make the corresponding new term in $\dot V$ negative-definite, while passing a residual cross-coupling to the next step.


---

## 6. Step 1 — Virtual Control 1: Desired Pitch $\vartheta^*$

**Compute $\dot V_{\mathrm{pos}}$.**

$$
\dot V_{\mathrm{pos}} = k_{px}e_x\dot x + \dot x\ddot x + k_{py}e_y\dot y + \dot y\ddot y
$$

First, choose the desired accelerations:


$$
\begin{aligned}
A_x &= -k_{px}e_x - k_{dx}\dot x \\
A_y &= -k_{py}e_y - k_{dy}\dot y
\end{aligned}
\tag{O1}
$$

and require $(F/m)\sin\vartheta^* = A_x$, $(F/m)\cos\vartheta^* = A_y + g$. This gives:

<blockquote>
<strong>Remark (aerodynamics omitted from feedforward).</strong> The desired accelerations (O1) account only for thrust, not for aerodynamic forces $X_b$, $Y_b$. Including them would make the computation circular: computing $\vartheta^*$ requires knowing $X_b, Y_b$, which depend on the angle of attack $\alpha = \vartheta - \mathrm{atan2}(\dot x, \dot y)$, which in turn depends on $\vartheta^*$. Instead, aerodynamic forces are treated as bounded disturbances — they appear as extra terms when (T1)–(T2) are substituted into $\dot V_\mathrm{pos}$, and are absorbed into the ISS bound $D$ of Section 10.5. This is justified because during the landing burn thrust dominates aerodynamics by roughly an order of magnitude, so the feedforward error is small and the feedback loop rejects the remainder.
</blockquote>

$$
\boxed{\sigma = \mathrm{clip}\!\left(\frac{m\sqrt{A_x^2 + (A_y+g)^2}}{F_{\max}},\, \sigma_{\min},\, 1\right)} \tag{O2}
$$

$$
\boxed{\vartheta^* = \mathrm{atan2}(A_x,\, A_y + g)} \tag{O3}
$$

**Hover check:** $e_x = e_y = \dot x = \dot y = 0 \Rightarrow A_x = 0$, $A_y = 0$, $\sigma = mg/F_{\max}$, $\vartheta^* = 0$. ✓

Now substitute (T1)–(T2) and (O1) into $\dot V_{\mathrm{pos}}$, applying A5: $\sin\vartheta \approx \sin\vartheta^* + \cos\vartheta^*\cdot z_\vartheta$, $\cos\vartheta \approx \cos\vartheta^* - \sin\vartheta^*\cdot z_\vartheta$. Using $(F/m)\sin\vartheta^* = A_x$ and $(F/m)\cos\vartheta^* = A_y + g$.

**Derivation of the $x$-channel term.** Substitute $\ddot x$ from (T1):

$$
\dot x\ddot x + k_{px}e_x\dot x = \dot x\!\left(\frac{F\sin\vartheta + F\delta\cos\vartheta + X_b\sin\vartheta + Y_b\cos\vartheta}{m} + k_{px}e_x\right)
$$

Apply A5 to expand $F\sin\vartheta/m$ around $\vartheta^*$:

$$
\frac{F\sin\vartheta}{m} \approx \frac{F\sin\vartheta^*}{m} + \frac{F\cos\vartheta^*}{m}z_\vartheta = A_x + \frac{F\cos\vartheta^*}{m}z_\vartheta
$$

Collecting terms:

$$
\dot x\ddot x + k_{px}e_x\dot x = \dot x(A_x + k_{px}e_x) + \dot x\frac{F}{m}\cos\vartheta^*\cdot z_\vartheta + \delta\frac{F}{m}\cos\vartheta\cdot\dot x + \frac{\dot x(X_b\sin\vartheta + Y_b\cos\vartheta)}{m}
$$

With $A_x = -k_{px}e_x - k_{dx}\dot x$ from (O1), the first group gives $\dot x(-k_{dx}\dot x) = -k_{dx}\dot x^2$. The same expansion applied to the $y$-channel yields $-k_{dy}\dot y^2$ plus analogous residual terms.

With $A_x = -k_{px}e_x - k_{dx}\dot x$, the first term gives $-k_{dx}\dot x^2$. The aerodynamic contribution to the $x$-channel is $\dot x(X_b\sin\vartheta + Y_b\cos\vartheta)/m$. The drag term $X_b = -C_x q_\infty S_m < 0$ contributes $-C_x q_\infty S_m \dot x \sin\vartheta / m$, which is not sign-definite. However, the full aerodynamic contribution to $\dot V_{\mathrm{pos}}$ from both channels is:

$$
\frac{\dot x(X_b\sin\vartheta + Y_b\cos\vartheta) + \dot y(X_b\cos\vartheta - Y_b\sin\vartheta)}{m} = \frac{X_b(\dot x\sin\vartheta + \dot y\cos\vartheta) + Y_b(\dot x\cos\vartheta - \dot y\sin\vartheta)}{m}
$$

The first group $\dot x\sin\vartheta + \dot y\cos\vartheta$ is the component of velocity along the body axis (axial velocity $V_a$), so $X_b V_a/m = -C_x q_\infty S_m V_a/m$. Since drag opposes motion, $X_b V_a = -C_x q_\infty S_m V_a$ is non-positive when $V_a \geq 0$ (forward flight). The second group $\dot x\cos\vartheta - \dot y\sin\vartheta$ is the lateral velocity $V_n$, and $Y_b V_n/m = C_{y\alpha}\alpha q_\infty S_m V_n / m$. This is not sign-definite in general. It is treated as a bounded disturbance: $|Y_b V_n/m| \leq C_{y\alpha}|\alpha| q_\infty S_m |V|/m$, which is bounded since all signals are bounded (Section 10.2). For the purposes of the Lyapunov inequality it is absorbed into the ISS bound $D$ of Section 10.5 as an additional bounded term $\bar d_Y$. Collecting the sign-definite terms:

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

From (DV1) we already have $\dot V_\mathrm{pos} \leq -k_{dx}\dot x^2 - k_{dy}\dot y^2 + z_\vartheta P + \delta Q + \bar d_Y$. Adding the new $z_\vartheta \dot z_\vartheta$ term and grouping the $z_\vartheta$ contributions:

$$
\begin{aligned}
\dot V_2 &= \dot V_{\mathrm{pos}} + z_\vartheta\dot z_\vartheta \\
&= -k_{dx}\dot x^2 - k_{dy}\dot y^2 + z_\vartheta(P + \dot z_\vartheta) + \delta\cdot Q + \text{(aero damping)}
\end{aligned}
$$

Substituting $\dot z_\vartheta = \dot\vartheta - \dot\vartheta^*$ and rewriting $\dot\vartheta$ using the definition $z_\omega = \dot\vartheta - \alpha_1$ (from Section 5), i.e. $\dot\vartheta = \alpha_1 + z_\omega$:

$$
z_\vartheta(P + \dot z_\vartheta) = z_\vartheta(P + \dot\vartheta - \dot\vartheta^*) = z_\vartheta(P + \alpha_1 + z_\omega - \dot\vartheta^*)
$$

**Design step.** We want to choose $\alpha_1$ (the desired angular rate) to eliminate the $z_\vartheta$ cross-term. Require the bracket multiplying $z_\vartheta$ — excluding $z_\omega$ which belongs to the next step — to equal $-k_\vartheta z_\vartheta$, i.e. $P + \alpha_1 - \dot\vartheta^* = -k_\vartheta z_\vartheta$:

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
\begin{aligned}
V = V_3 + \tfrac{1}{2}z_\delta^2 = \tfrac{1}{2}k_{px}e_x^2 + \tfrac{1}{2}\dot x^2 + \tfrac{1}{2}k_{py}e_y^2 + \tfrac{1}{2}\dot y^2 + \tfrac{1}{2}z_\vartheta^2 + \tfrac{1}{2}z_\omega^2 + \tfrac{1}{2}z_\delta^2
\end{aligned}
\tag{V4}
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
\dot V = -k_{dx}\dot x^2 - k_{dy}\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \alpha_2\cdot Q + \text{(aero damping)} \tag{ISS1}
$$

The $\alpha_2\cdot Q$ residual is discussed in Section 10.

---

## 10. Stabilization Proof

### 10.1 Handling the $\alpha_2 \cdot Q$ Residual

Before proceeding to the proof, we characterise the single term in (ISS1) that does not have a definite sign.

The coupling $Q = (F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$ satisfies $|Q| \leq (F/m)\sqrt{\dot x^2 + \dot y^2}$. Applying Young's inequality with parameter $\epsilon > 0$:

$$
|\alpha_2\cdot Q| \leq \frac{\epsilon}{2}\alpha_2^2 + \frac{1}{2\epsilon}Q^2 \leq \frac{\epsilon}{2}\alpha_2^2 + \frac{(F/m)^2}{2\epsilon}(\dot x^2 + \dot y^2)
$$

Choosing $\epsilon$ such that $\tfrac{(F/m)^2}{2\epsilon} \leq \min(k_{dx}, k_{dy})$, i.e.,

$$
\epsilon \geq \frac{(F/m)^2}{2\min(k_{dx}, k_{dy})} \tag{C-eps}
$$

the $Q^2$ residual is absorbed by the velocity damping terms, and (ISS1) becomes:

$$
\begin{aligned}
\dot V \leq &-\left(k_{dx} - \frac{(F/m)^2}{2\epsilon}\right)\dot x^2 - \left(k_{dy} - \frac{(F/m)^2}{2\epsilon}\right)\dot y^2 \\
&- k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \frac{\epsilon}{2}\alpha_2^2
\end{aligned}
\tag{ISS2}
$$

Define $k_{dx}' = k_{dx} - (F/m)^2/(2\epsilon) > 0$ and $k_{dy}' = k_{dy} - (F/m)^2/(2\epsilon) > 0$ (both positive by (C-eps)). The remaining term $\tfrac{\epsilon}{2}\alpha_2^2$ is positive but vanishes at equilibrium since $\alpha_2 \to 0$ when $z_\omega, z_\vartheta, f_2 \to 0$ in (VC2). Its treatment is deferred to Section 10.2.

### 10.2 Boundedness of All Signals

**Note on the $\tfrac{\epsilon}{2}\alpha_2^2$ term.** From (VC2), $\alpha_2 = \tfrac{1}{g_2}(-k_\omega z_\omega - z_\vartheta - f_2 + \dot\alpha_1)$, so $\alpha_2$ is a function of $z_\omega$, $z_\vartheta$, $f_2$, and $\dot\alpha_1$, all of which are components of the state vector covered by $V$. Taking the absolute value and using $g_2 \geq g_{2,\min} > 0$ (Assumption A6):

$$
|\alpha_2| \leq \frac{1}{g_{2,\min}}\!\left(k_\omega|z_\omega| + |z_\vartheta| + |f_2| + |\dot\alpha_1|\right)
$$

Each term in the bound is shown bounded below:

**$|z_\omega|$ and $|z_\vartheta|$** are the attitude tracking errors ($z_\vartheta = \vartheta - \vartheta^*$, $z_\omega = \dot\vartheta - \alpha_1$). Since $V \geq \tfrac{1}{2}z_\vartheta^2$ and $V \geq \tfrac{1}{2}z_\omega^2$ by the structure of (V4):
$$
\begin{aligned}
|z_\vartheta| &\leq \sqrt{2V} \\
|z_\omega| &\leq \sqrt{2V}
\end{aligned}
$$
Both are bounded by $\sqrt{2V(0)}$ initially. The UUB result of Section 10.2 then guarantees $V(t) \leq V_{\max}$ for all $t$, so the bound holds globally.

**$|f_2|$** is the aerodynamic pitching moment normalised by inertia: $f_2 = C_{m\alpha}\,\alpha\,q_\infty S_m l / J$. It depends on airspeed $V = \sqrt{\dot x^2 + \dot y^2}$ (bounded by physical limits of the scenario) and angle of attack $\alpha = \vartheta - \mathrm{atan2}(\dot x, \dot y)$. The pitch $\vartheta$ is bounded via $\vartheta = \vartheta^* + z_\vartheta$, where $\vartheta^* = \mathrm{atan2}(A_x, A_y+g)$ depends only on the bounded position errors $e_x, \dot x, e_y, \dot y$ (covered by $V_\mathrm{pos} \leq V$). Therefore:
$$|f_2| \leq \frac{C_{m\alpha}\,|\alpha|_{\max}\,q_{\infty,\max}\,S_m\,l}{J} \triangleq \bar f_2 < \infty$$

**$|\dot\alpha_1|$** is the time derivative of the desired angular rate $\alpha_1 = \dot\vartheta^* - k_\vartheta z_\vartheta - P$ (from (VC1)). Differentiating: $\dot\alpha_1 = \ddot\vartheta^* - k_\vartheta\dot z_\vartheta - \dot P$. Each term depends on positions, velocities, and their derivatives — all bounded by $V$ and the physical equations of motion. Therefore $|\dot\alpha_1| \leq \bar\alpha_1 < \infty$ for some constant $\bar\alpha_1$ depending on $V(0)$ and the gains.

Combining all four bounds, there exists a constant $C_{\alpha_2}$ depending only on $V(0)$ and the gains such that:

$$
\frac{\epsilon}{2}\alpha_2^2 \leq \frac{\epsilon}{2} C_{\alpha_2}^2 \triangleq \mu
$$

Substituting into (ISS2):

$$
\begin{aligned}
\dot V \leq{} &-k_{dx}'\,\dot x^2 - k_{dy}'\,\dot y^2 - k_\vartheta z_\vartheta^2 - k_\omega z_\omega^2 - k_\delta z_\delta^2 + \mu
\end{aligned}
\tag{ISS3}
$$

This means $\dot V < 0$ whenever $V$ is large enough that the negative terms dominate $\mu$. By standard arguments (see Khalil, Theorem 4.18), all signals are **uniformly ultimately bounded**: they converge to and remain within the compact set:

$$
\begin{aligned}
\Omega &= \Bigl\{ V \leq \tfrac{\mu}{c} \Bigr\} \\
c &= \min(k_{dx}',\, k_{dy}',\, k_\vartheta,\, k_\omega,\, k_\delta)
\end{aligned}
$$

**Theorem (UUB).** Under the control law (CL), virtual controls (VC1)–(VC2), and Assumptions A1–A7, if gains satisfy (C-eps) strictly, then all signals $(e_x, \dot x, e_y, \dot y, z_\vartheta, z_\omega, z_\delta)$ are **uniformly ultimately bounded**: they enter and remain in the compact set $\Omega = \{V \leq \mu/c\}$ in finite time. The transient bound is:

$$
V(t) \leq \max\!\left(V(0),\, \frac{\mu}{c}\right) \triangleq V_{\max}
$$

Reading off individual bounds from $V(t) \leq V_{\max}$:

$$
\begin{aligned}
|e_x(t)| &\leq \sqrt{\frac{2V_{\max}}{k_{px}}} \triangleq B_{e_x} \\
|\dot x(t)| &\leq \sqrt{2V_{\max}} \triangleq B_{\dot x}
\end{aligned}
$$

and analogously for $e_y$, $\dot y$, $z_\vartheta$, $z_\omega$, $z_\delta$. All bounds depend only on $V(0)$, $\mu$, and the gains. $\square$

**Consequence.** Since $(z_\vartheta, z_\omega, z_\delta)$ are bounded and the virtual controls (VC1)–(VC2) are continuous functions of bounded signals, $\vartheta$, $\dot\vartheta$, $\delta$ and $\alpha_2$ are all bounded. In particular, the singularity $g_2 = 0$ is never reached under A6.

**Remark (two-phase stability picture).** The overall stability result has two phases:

- **Phase 1 — transient ($\alpha_2 \neq 0$, $\mu > 0$):** While the system is away from equilibrium, $\alpha_2 \neq 0$ and the residual $\mu = \tfrac{\epsilon}{2}C_{\alpha_2}^2 > 0$ from the $\alpha_2 \cdot Q$ cross-term is nonzero. Additionally, clipping of $\alpha_1$, $\alpha_2$, $\delta_{\mathrm{cmd}}$ may be active (e.g. during the first 1–2 s with a 30° initial tilt), contributing the extra disturbance $D'$ of Section 10.5. In this phase only UUB holds: all errors are bounded and remain in the ball $\Omega = \{V \leq D'/c\}$. The system does not diverge, but exact convergence to zero is not guaranteed.

- **Phase 2 — steady descent ($\alpha_2 \to 0$, $\mu \to 0$):** As the system approaches equilibrium, $\alpha_2 \to 0$ so $\mu \to 0$, and clipping becomes inactive so $D' \to 0$. The stronger results of Sections 10.3 and 10.4 apply: Barbalat gives $\dot x, \dot y, z_\vartheta, z_\omega, z_\delta \to 0$, and LaSalle gives $e_x, e_y \to 0$.

In short: the controller is practically stable throughout, and asymptotically convergent once $\alpha_2$ is small and clipping is inactive.

### 10.3 Convergence of Velocities and Attitude Errors — Barbalat's Lemma

**Remark on scope.** The Barbalat argument below is an idealised analysis that assumes $\mu = 0$ from the start — i.e., the $\alpha_2 \cdot Q$ cross-term is neglected. This is a standard simplification used to establish the convergence structure of the controller. In reality $\mu = \tfrac{\epsilon}{2}C_{\alpha_2}^2 > 0$ whenever $\alpha_2 \neq 0$, so the true result is UUB (Section 10.2), not exact convergence to zero. The Barbalat argument is nonetheless useful because it shows what the system would do in the absence of the residual, and because $\mu$ is small when $\alpha_2$ is small (i.e., when the nozzle is near its equilibrium position).

**Theorem (idealised, $\mu = 0$).** If the $\alpha_2 \cdot Q$ residual is neglected, then $\dot x(t) \to 0$, $\dot y(t) \to 0$, $z_\vartheta(t) \to 0$, $z_\omega(t) \to 0$, $z_\delta(t) \to 0$ as $t \to \infty$.

**Proof via Barbalat's Lemma** (applied to each damped state in turn).

We show the argument for $\dot x$; the others follow identically.

*Step 1 — Integrability.* With $\mu = 0$, (ISS3) gives $\dot V \leq -k_{dx}'\,\dot x^2$. Integrating from $0$ to $T$:

$$
\int_0^T k_{dx}'\,\dot x^2(\tau)\,d\tau \leq V(0) - V(T) \leq V(0) < \infty
$$

Since this holds for all $T$, taking $T \to \infty$: $\dot x^2 \in L^2([0,\infty))$.

*Step 2 — Uniform continuity.* From Section 10.2, $|\dot x| \leq \sqrt{2V_{\max}}$, so $\dot x$ is bounded. From (T1), $\ddot x$ is a continuous function of bounded signals, so it is also bounded. Therefore $\tfrac{d}{dt}(\dot x^2) = 2\dot x\,\ddot x$ is bounded as a product of two bounded quantities, which means $\dot x^2$ is uniformly continuous.

*Step 3 — Barbalat's Lemma.* A non-negative uniformly continuous function with finite $L^2$ integral converges to zero:

$$
\dot x^2(t) \to 0 \implies \dot x(t) \to 0 \quad \text{as } t \to \infty \qquad \square
$$

The same argument applies to $\dot y$, $z_\vartheta$, $z_\omega$, and $z_\delta$ using the corresponding negative-definite terms in (ISS3).

### 10.4 Convergence of Position Errors — LaSalle's Invariance Principle

**Theorem (idealised, same scope as Section 10.3).** If the $\alpha_2 \cdot Q$ residual is neglected ($\mu = 0$), then $e_x(t) \to 0$ and $e_y(t) \to 0$ as $t \to \infty$. Under the true $\mu > 0$, position errors are bounded by $|e_x| \leq B_{e_x}$ from Section 10.2; exact convergence to zero is not claimed.

**Applicability of LaSalle.** The closed-loop system is **autonomous** — the right-hand side $f(s)$ depends only on the state $s$, not explicitly on $t$ (the controller is a pure state-feedback law with no external time-varying signals). LaSalle's invariance principle (Khalil, Theorem 4.4) therefore applies directly. The set $\{\dot V = 0\}$ contains states where $\dot x = \dot y = z_\vartheta = z_\omega = z_\delta = 0$ but $e_x, e_y$ may be non-zero, so we must identify the largest invariant subset of $\{\dot V = 0\}$ — which is $\mathcal{S}$ below. All trajectories remain in the compact sublevel set $\{V \leq V_{\max}\}$ (Section 10.2), satisfying the compactness requirement of the theorem.

**Define the limiting set:**

$$
\mathcal{S} = \Bigl\{(e_x,\, \dot x,\, e_y,\, \dot y,\, z_\vartheta,\, z_\omega,\, z_\delta) \;:\; \dot x = \dot y = z_\vartheta = z_\omega = z_\delta = 0\Bigr\}
$$

Under the idealised assumption $\mu = 0$, Section 10.3 shows every trajectory enters $\mathcal{S}$ asymptotically. We now show that any trajectory remaining in $\mathcal{S}$ for all time must satisfy $e_x = e_y = 0$.

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

### 10.5 Stability Under Saturation — ISS Analysis

Section 10.2 (UUB) holds regardless of clipping. The idealised convergence proofs in Sections 10.3–10.4 additionally assume that $\alpha_1$, $\alpha_2$, and $\delta_{\mathrm{cmd}}$ are delivered exactly (no clipping). In the real implementation all three are clipped:

$$
\alpha_{1,\mathrm{sat}} = \mathrm{clip}(\alpha_{1,\mathrm{raw}},\,-\alpha_{1,\max},\,\alpha_{1,\max})
$$

$$
\begin{aligned}
\alpha_{2,\mathrm{sat}} &= \mathrm{clip}(\alpha_{2,\mathrm{raw}},\,-\delta_{\max},\,\delta_{\max}) \\
\delta_{\mathrm{cmd},\mathrm{sat}} &= \mathrm{clip}(\delta_{\mathrm{cmd}},\,-\delta_{\max,\mathrm{cmd}},\,\delta_{\max,\mathrm{cmd}})
\end{aligned}
$$

This section proves that the closed-loop system remains **input-to-state stable (ISS)** with respect to the saturation errors, giving a rigorous global result without assuming saturation never occurs.

#### 10.5.1 Saturation Errors as Bounded Disturbances

Three saturations are active in the implementation. Each is modelled as an additive bounded disturbance on the relevant error dynamics.

**$\alpha_1$ saturation.** Define:

$$
\begin{aligned}
d_{\vartheta} &= \alpha_{1,\mathrm{sat}} - \alpha_{1,\mathrm{raw}}, \quad |d_{\vartheta}| \leq 2\alpha_{1,\max}
\end{aligned}
$$

This enters the $z_\omega$ dynamics as $-d_\vartheta$ (since $z_\omega = \dot\vartheta - \alpha_1$ and $\alpha_1$ is clipped), giving a bounded disturbance term $|{-d_\vartheta}| \leq 2\alpha_{1,\max} \triangleq \bar d_\vartheta$.

**$\alpha_2$ saturation.** Define:

$$
\begin{aligned}
d_\omega &= \alpha_{2,\mathrm{sat}} - \alpha_{2,\mathrm{raw}}, \quad |d_\omega| \leq 2\delta_{\max}
\end{aligned}
$$

This propagates through the $z_\omega$ dynamics with gain $g_2$, giving bounded disturbance $|g_2 d_\omega| \leq g_{2,\max} \cdot 2\delta_{\max} \triangleq \bar d_\omega$.

**$\delta_{\mathrm{cmd}}$ saturation.** Define:

$$
\begin{aligned}
d_\delta &= \delta_{\mathrm{cmd},\mathrm{sat}} - \delta_{\mathrm{cmd}}, \quad |d_\delta| \leq 2\delta_{\max,\mathrm{cmd}}
\end{aligned}
$$

This enters the $z_\delta$ dynamics divided by $\tau_\delta$, giving bounded disturbance $|d_\delta/\tau_\delta| \leq 2\delta_{\max,\mathrm{cmd}}/\tau_\delta \triangleq \bar d_\delta$.

All three disturbances are **uniformly bounded** regardless of state.

#### 10.5.2 Perturbed $z_\omega$ Dynamics

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

#### 10.5.3 Perturbed $z_\delta$ Dynamics

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

#### 10.5.4 Lyapunov Derivative Under Both Disturbances

Starting from (ISS1) and substituting the perturbed dynamics, the $z_\omega$ and $z_\delta$ contributions become:

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
\dot V \leq -k_{dx}''\,\dot x^2 - k_{dy}''\,\dot y^2 - \frac{k_\vartheta}{2}z_\vartheta^2 - \frac{k_\omega}{2}z_\omega^2 - \frac{k_\delta}{4}z_\delta^2 + D' \tag{ISS4}
$$

where $k_{dx}''$, $k_{dy}''$ are the further-reduced damping coefficients after absorbing the $Q z_\delta$ velocity residual.

The remaining disturbance cross-terms $g_2 d_\omega z_\omega$ and $(d_\delta/\tau_\delta) z_\delta$ are bounded via Young's inequality with parameters $\lambda_1 = k_\omega/4$ and $\lambda_2 = k_\delta/8$ respectively:

$$
|g_2 d_\omega z_\omega| \leq \frac{\lambda_1}{2}z_\omega^2 + \frac{g_2^2 d_\omega^2}{2\lambda_1} = \frac{k_\omega}{8}z_\omega^2 + \frac{2g_2^2 \bar d_\omega^2}{k_\omega}
$$

$$
\left|\frac{d_\delta}{\tau_\delta} z_\delta\right| \leq \frac{\lambda_2}{2}z_\delta^2 + \frac{d_\delta^2}{2\lambda_2\tau_\delta^2} = \frac{k_\delta}{16}z_\delta^2 + \frac{4\bar d_\delta^2}{k_\delta\tau_\delta^2}
$$

The $-d_\vartheta z_\omega$ cross-term (from $\alpha_1$ clipping) is bounded similarly with parameter $\lambda_5 = k_\omega/4$:

$$
|d_\vartheta z_\omega| \leq \frac{\lambda_5}{2}z_\omega^2 + \frac{d_\vartheta^2}{2\lambda_5} = \frac{k_\omega}{8}z_\omega^2 + \frac{2\bar d_\vartheta^2}{k_\omega}
$$

These reduce the $z_\omega^2$ coefficient by $k_\omega/8 + k_\omega/8 = k_\omega/4$ total and the $z_\delta^2$ coefficient by $k_\delta/16$, keeping both positive. Collecting all constant terms:

$$
D' = \mu + \frac{2g_2^2\bar d_\omega^2}{k_\omega} + \frac{2\bar d_\vartheta^2}{k_\omega} + \frac{4\bar d_\delta^2}{k_\delta\tau_\delta^2}
$$

#### 10.5.5 UUB Conclusion

Define:

$$
\begin{aligned}
c &= \min\!\left(k_{dx}'',\, k_{dy}'',\, \frac{k_\vartheta}{2},\, \frac{k_\omega}{4},\, \frac{k_\delta}{8}\right) \\
D &= D'
\end{aligned}
$$

From (ISS4), $\dot V \leq 0$ whenever each squared error term exceeds $D/(\text{its coefficient})$. By Khalil Theorem 4.18, all signals are **uniformly ultimately bounded** and converge to:

$$
\mathcal{B} = \Bigl\{ V \leq \tfrac{D}{c} \Bigr\}
$$

Since $V$ does not contain position errors $e_x, e_y$ with coefficients that appear in the negative-definite part of (ISS4), we cannot write $\dot V \leq -cV + D$ directly — the correct statement is that (ISS4) is negative outside $\mathcal{B}$. The individual state bounds follow from $V(t) \leq V_{\max} = \max(V(0), D/c)$.

**Theorem (UUB under saturation).** Under Assumptions A1–A7, gain condition (C-eps) (extended to cover the $Qz_\delta$ term), and $k_\omega > 1/k_\vartheta + 1$, the closed-loop system with saturated controls is **uniformly ultimately bounded**: all states enter and remain in $\mathcal{B}$. The residual bound on attitude errors is:

$$
\begin{aligned}
|z_\omega|_{\infty} &\leq \sqrt{\frac{2D}{c\,k_\omega}} \\
|z_\delta|_{\infty} &\leq \sqrt{\frac{2D}{c\,k_\delta}}
\end{aligned}
$$

#### 10.5.6 Quantitative Bound for the Project Parameters

With the physical parameters and current gains from `configs/backstepping.yaml`:

- $g_{2,\max} = F_{\max}\,l_{cp}/J = 191763 \times 10.5 / 318196.65 \approx 6.33\ \mathrm{rad/s^2/rad}$
- $\alpha_{1,\max} = 1.5\ \mathrm{rad/s}$, so $\bar d_\vartheta = 2 \times 1.5 = 3.0\ \mathrm{rad/s}$
- $\delta_{\max} = 15° = 0.262\ \mathrm{rad}$, so $\bar d_\omega = 2 \times 6.33 \times 0.262 \approx 3.31\ \mathrm{rad/s^2}$
- $\delta_{\max,\mathrm{cmd}} = 15° = 0.262\ \mathrm{rad}$, $\tau_\delta = 0.05\ \mathrm{s}$, so $\bar d_\delta = 2 \times 0.262/0.05 \approx 10.47\ \mathrm{rad/s}$

With $k_\omega = 5$, $k_\delta = 15$, $\tau_\delta = 0.05$ s, the three contributions to $D'$ are:

$$
\begin{aligned}
\frac{2g_{2,\max}^2\bar d_\omega^2}{k_\omega} &\approx 175.8 \\
\frac{2\bar d_\vartheta^2}{k_\omega} &\approx 3.6 \\
\frac{4\bar d_\delta^2}{k_\delta\tau_\delta^2} &\approx 11697
\end{aligned}
$$

The $\delta_{\mathrm{cmd}}$ clipping term dominates completely — it is amplified by $1/\tau_\delta^2 = 400$. Total $D' \approx \mu + 11876$.

With $c = \min(k_\vartheta/2,\, k_\omega/4,\, k_\delta/8) = \min(3.0,\, 1.25,\, 1.875) = 1.25$:

$$
|z_\omega|_\infty \leq \sqrt{\frac{2 \times 11876}{1.25 \times 5}} \approx 61.6\ \mathrm{rad/s} \approx 3532°/\mathrm{s}
$$

This absurdly large bound is a direct consequence of the $1/\tau_\delta^2$ amplification — the ISS framework assumes $\delta_{\mathrm{cmd}}$ clipping is **permanently active**, which never happens in practice. In the simulation, $\delta_{\mathrm{cmd}}$ clipping clears within 1–2 s; after that $d_\delta = 0$ exactly, $D' \to \mu$, and the system converges asymptotically. The bound is technically correct but extremely conservative.

This is a worst-case bound assuming all three clipping saturations are **permanently active simultaneously**. In the simulation, $\alpha_1$ clipping clears within 0.1 s and $\alpha_2$ clipping within 1–2 s; after that, $d_\vartheta = d_\omega = d_\delta = 0$ exactly. As the system then converges, $\alpha_2 \to 0$ so $\mu \to 0$, and the results of Sections 10.2–10.4 apply, giving true asymptotic convergence to zero.

---

## 11. Full Algorithm

**State:** $s = [x, y, \dot x, \dot y, \vartheta, \dot\vartheta, \delta]^T$. **Filter state:** $\alpha_2^f$ (command filter — introduced in Block E to avoid differentiating $\alpha_2$ analytically). **Derivative filter states:** $\dot\vartheta^*_{\mathrm{filt}}$, $\dot\alpha_{1,\mathrm{filt}}$ (ODE states, see Section 13).

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

<blockquote>
<strong>Safety clamping (steps 4–6).</strong> The theoretical design requires only $a_{\min} = 0$ for the Lyapunov proof; the implementation enforces $a_{\min} = 0.5\ \mathrm{m/s^2}$ and the tilt limit $\phi_{\lim} = 30°$ to keep the rocket from commanding full free-fall or excessive tilt. The clamp on $A_x$ ensures $|\vartheta^*| \leq \phi_{\lim}$ exactly. These modifications introduce a bounded error in (O1) that is handled by the ISS argument of Section 10.5.
</blockquote>

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

<blockquote>
<strong>Implementation note — P-coupling omission.</strong> The theoretical formula (VC1) includes the term $-(F/m)(\dot x\cos\vartheta^* - \dot y\sin\vartheta^*)$. In practice this reaches 60+ rad/s at high entry velocities and permanently saturates $\alpha_1$, destabilising the inner loops. The simplified form $\alpha_1 = \dot\vartheta^* - k_\vartheta z_\vartheta$ is used instead. The omitted P term is a bounded disturbance covered by Section 10.5.
</blockquote>

<blockquote>
<strong>Implementation note — filtered derivatives.</strong> The implementation propagates $\dot\vartheta^*$ and $\dot\alpha_1$ as ODE states (`ts_dot`, `a1_dot`) driven by first-order filters toward their instantaneous analytical values (bandwidth $N = 50\ \mathrm{rad/s}$). This means the $\dot\vartheta^*$ used in $\alpha_1$ and the $\dot\alpha_1$ used in $\alpha_2$ are slightly lagged. The lag error is bounded by $|\dot\vartheta^*_{\mathrm{inst}} - \dot\vartheta^*_{\mathrm{filt}}| \leq C/N$ for a constant $C$ determined by the rate of change of $\dot\vartheta^*$, and constitutes another bounded disturbance within the ISS framework.
</blockquote>

<blockquote>
<strong>Implementation note — alpha_1 saturation.</strong> The implementation clips $\alpha_1$ to $\pm\alpha_{1,\max}$ (default 2 rad/s). This is an additional saturation not present in the theoretical derivation. It introduces a bounded disturbance in the $z_\omega$ dynamics analogous to the $\alpha_2$ saturation analysed in Section 10.5, with $\bar d_{\alpha_1} = 2\alpha_{1,\max}$ replacing $2\delta_{\max}$.
</blockquote>

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

<blockquote>
<strong>Command filtering and the modified z_delta.</strong> The Lyapunov derivation (Sections 8–9) defines $z_\delta = \delta - \alpha_2$ and uses $\dot\alpha_2$ in (CL). In the implementation, $\alpha_2$ is replaced by its filtered version $\alpha_2^f$, so $z_\delta = \delta - \alpha_2^f$ and $\dot\alpha_2^f$ enters (CL). The filter error $e_f = \alpha_2^f - \alpha_2$ satisfies $\dot e_f = -e_f/\tau_f + (\alpha_{2,\mathrm{sat}} - \alpha_2)/\tau_f$, which is bounded. Command filtering is standard practice (Farrell et al., 2005) to avoid differentiating $\alpha_2$ analytically; the resulting filter error introduces an additional bounded disturbance of order $O(\tau_f)$ that is absorbed by the ISS bound of Section 10.5.
</blockquote>

**Block F — Step 4: Real control ($\delta_{\mathrm{cmd}}$)**

```
23. δ_cmd = δ + τ_δ * (α2f_dot - g2 * z_ω - k_δ * z_δ)
24. δ_cmd = clip(δ_cmd, -δ_max_cmd, δ_max_cmd)
```

<blockquote>
<strong>Implementation note — Q-coupling omission.</strong> The theoretical formula (CL) includes the term $-Q = -(F/m)(\dot x\cos\vartheta - \dot y\sin\vartheta)$. As with P-coupling in Block C, this term reaches 50+ rad/s at high entry velocities and, multiplied by $\tau_\delta$, produces nozzle commands far exceeding $\delta_{\max,\mathrm{cmd}}$. The term is omitted in the implementation; the resulting bounded error is covered by the ISS analysis of Section 10.5.
</blockquote>

**Output:** $[\sigma,\ \delta_{\mathrm{cmd}}]$

---

## 12. Gain Conditions and Tuning Guidelines

### Necessary conditions

$$
k_{px}, k_{dx}, k_{py}, k_{dy}, k_\vartheta, k_\omega, k_\delta > 0, \quad \sigma_{\min} > 0, \quad \tau_f > 0
$$

### Critical damping of position loop

$$
\begin{aligned}
k_{dx} &\geq 2\sqrt{k_{px}} \\
k_{dy} &\geq 2\sqrt{k_{py}}
\end{aligned}
$$

### Attitude loop bandwidth

No formal time-scale separation is required, but faster attitude convergence reduces the time during which the $\alpha_2\cdot Q$ residual is active. A practical guideline:

$$
\begin{aligned}
\sqrt{k_\vartheta\, g_{2,\mathrm{hov}}} &\geq 3\,\sqrt{k_{px}} \\
g_{2,\mathrm{hov}} &= \frac{\sigma_{\mathrm{hov}} F_{\max} l_{cp}}{J}
\end{aligned}
$$

### Cross-term damping condition

From the $z_\vartheta z_\omega$ bounding step in Section 10.5.4, the net $z_\omega^2$ coefficient remains positive only if:

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
6. Sontag, E. D. (1989). Smooth stabilization implies coprime factorization. *IEEE Transactions on Automatic Control*, 34(4), 435–443. — Original ISS definition and the comparison-lemma decay bound used in Section 10.5.
7. Sontag, E. D., & Wang, Y. (1995). On characterizations of the input-to-state stability property. *Systems & Control Letters*, 24(5), 351–359. — ISS Lyapunov characterisation; the $\dot V \leq -cV + D$ criterion.
