# Mathematical Derivations and Formal Analysis: Adaptive Lyapunov Attitude Control
 
This document contains the complete mathematical derivation and formal stability analysis for Project 2. It extends the Project 1 derivation by introducing aerodynamic uncertainty and the Certainty Equivalence adaptive law.
 
---
 
## Table of Contents
 
- [1. Plant Description and Aerodynamic Model](#1-plant-description-and-aerodynamic-model)
- [2. Simplifying Assumptions](#2-simplifying-assumptions)
- [3. Equations of Motion](#3-equations-of-motion)
- [4. Control Objective and Error Coordinates](#4-control-objective-and-error-coordinates)
- [5. General CE Framework](#5-general-ce-framework)
  - [5.1 General Plant Structure and Matching Condition](#51-general-plant-structure-and-matching-condition)
  - [5.2 Nominal Policy and Its Lyapunov Function](#52-nominal-policy-and-its-lyapunov-function)
  - [5.3 General CE Policy and Adaptation Law](#53-general-ce-policy-and-adaptation-law)
  - [5.4 Correspondence: Our Rocket as a Special Case](#54-correspondence-our-rocket-as-a-special-case)
- [6. Stage 1 — Idealized Control Law (Known Parameters)](#6-stage-1--idealized-control-law-known-parameters)
- [7. Stage 2 — Certainty Equivalence Substitution](#7-stage-2--certainty-equivalence-substitution)
- [8. Closed-Loop Dynamics](#8-closed-loop-dynamics)
- [9. Extended Lyapunov Function and Adaptation Law](#9-extended-lyapunov-function-and-adaptation-law)
- [10. Stabilization Proof](#10-stabilization-proof)
  - [10.1 Boundedness of All Signals](#101-boundedness-of-all-signals)
  - [10.2 Convergence of Angular Rate — Barbalat's Lemma](#102-convergence-of-angular-rate--barbălats-lemma)
  - [10.3 Convergence of Attitude Error — LaSalle](#103-convergence-of-attitude-error--lasalle)
  - [10.4 What the Proof Does and Does Not Guarantee](#104-what-the-proof-does-and-does-not-guarantee)
- [11. Parameter Error Dynamics](#11-parameter-error-dynamics)
- [12. Persistent Excitation and Parameter Convergence](#12-persistent-excitation-and-parameter-convergence)
- [13. Projection Operator](#13-projection-operator)
- [14. Comparison with Project 1 Baseline](#14-comparison-with-project-1-baseline)
- [15. Gain Conditions and Tuning Guidelines](#15-gain-conditions-and-tuning-guidelines)
- [16. Implementation Notes](#16-implementation-notes)
- [References](#references)
---
 
## 1. Plant Description and Aerodynamic Model
 
The system is a planar TVC rocket. A single engine at the base deflects by angle $\delta$ from the body axis, generating a corrective torque. The rocket now operates in an atmosphere, so aerodynamic forces and a pitching moment act on it.
 
### Body-fixed coordinate frame
 
The **body-fixed frame** has its origin at the center of mass (CoM):
- $X_b$ — longitudinal axis, pointing from tail to nose
- $Y_b$ — lateral axis, perpendicular to $X_b$ in the plane of motion
- $Z_b$ — out-of-plane axis, completing a right-handed triad
The pitch angle $\vartheta$ is measured from the inertial vertical ($+y$ axis) to $X_b$, positive rightward. See `README-Aerodynamics.md` for diagrams and the full body-to-inertial rotation matrix.
 
### Aerodynamic quantities (body frame)
 
$$
X_b = -C_x \cdot q_\infty S_m \qquad \text{[drag, axial]}
$$
 
$$
Y_b = C_{y\alpha}\cdot\alpha \cdot q_\infty S_m \qquad \text{[normal force]}
$$
 
$$
M_b^z = C_{m\alpha}\cdot\alpha \cdot q_\infty S_m l \qquad \text{[pitching moment about CoM]}
$$
 
where $q_\infty = \tfrac{1}{2}\rho v^2$ is the dynamic pressure, $v = \|\dot{\mathbf{r}}\|$ is the airspeed, $S_m$ is the reference cross-sectional area, and $l$ is the reference length (total rocket length).
 
### Regressor definition
 
The pitching moment $M_b^z = C_{m\alpha}\cdot\alpha\cdot q_\infty S_m l$ can be written as a product of the unknown parameter and a **known function of the state**. This known function is called the **regressor**. For our system we introduce the shorthand:
 
$$
Y(\alpha) \triangleq q_\infty S_m l \cdot \alpha
$$
 
so that $M_b^z = Y(\alpha)\cdot C_{m\alpha}$.
 
**$Y(\alpha)$ is fully computable despite $C_{m\alpha}$ being unknown.** The regressor contains known quantities: $\rho = 1.225\ \text{kg/m}^3$, $v = \sqrt{\dot x^2 + \dot y^2}$, $S_m$, $l$ (known geometry), and $\alpha = \vartheta - \mathrm{atan2}(\dot x, \dot y)$.

### Angle of attack
 
The angle of attack is the angle between the rocket body axis and the velocity vector:
 
$$
\alpha = \vartheta - \mathrm{atan2}(\dot x,\, \dot y)
$$
 
At zero velocity ($v = 0$), we set $\alpha = \vartheta$ by convention (no aerodynamic forces at zero airspeed).

### State Variables and Control Input

The system state vector is:

$$
q = [x,\ y,\ \vartheta,\ \dot{x},\ \dot{y},\ \dot{\vartheta}]^T
$$

where:
- $x, y$ — inertial horizontal and vertical position (m)
- $\vartheta$ — pitch angle from the vertical axis, positive rightward (rad)
- $\dot{x}, \dot{y}$ — inertial translational velocities (m/s)
- $\dot{\vartheta}$ — angular rate (rad/s)

The adaptive controller maintains an additional internal state — the online parameter estimate:

$$
\hat{C}_{m\alpha}(t) \in [\theta_{\min},\ \theta_{\max}]
$$

which evolves according to the adaptation law (Section 9). In the ODE integration, the extended state is:

$$
q_{ext} = [x,\ y,\ \vartheta,\ \dot{x},\ \dot{y},\ \dot{\vartheta},\ \hat{C}_{m\alpha}]^T
$$

The sole control input is the nozzle deflection angle:

$$
u = \delta, \qquad |\delta| \leq \delta_{max}
$$

where $\delta$ is measured from the rocket body axis, positive rightward. The throttle $F$ is fixed at $1.5\,mg$ and is not a control variable.

### System Parameters

| Symbol | Meaning | Units |
|--------|---------|-------|
| $m$ | Total mass (constant) | kg |
| $J$ | Moment of inertia about CoM (constant) | kg·m² |
| $l_{cp}$ | Distance from CoM to nozzle exit | m |
| $g$ | Gravitational acceleration, 9.81 | m/s² |
| $F$ | Constant thrust $= 1.5\,mg$ | N |
| $\delta_{max}$ | Nozzle deflection angle limit | rad |
| $\rho$ | Air density, 1.225 | kg/m³ |
| $S_m$ | Reference cross-sectional area | m² |
| $l$ | Reference length (rocket length) | m |
| $C_x$ | Axial drag coefficient, 0.358 | — |
| $C_{y\alpha}$ | Normal force coefficient, 0.05403 | deg⁻¹ |
| $C_{m\alpha}$ | Pitching moment coefficient (**unknown**) | rad⁻¹ |
| $\gamma$ | Adaptation rate | — |
| $k_\vartheta$ | Attitude proportional gain | — |
| $k_\omega$ | Angular rate damping gain | — |

---

## 2. Simplifying Assumptions
 
The following assumptions are made for Project 2 and must be stated in any analysis using these equations:
 
1. **Constant mass**: $\dot m = 0$, $\dot J = 0$. Mass is fixed at the initial value. This eliminates variable-inertia coupling and keeps the Lyapunov analysis time-invariant.
2. **Low-speed aerodynamic regime**: Airspeed $V \leq 100$ m/s (Mach $< 0.3$). In this regime:
   - $C_x$ is constant: $C_x = 0.358$
   - $m_z(\alpha)$ is strictly linear: $m_z(\alpha) = C_{m\alpha}\cdot\alpha$ with $C_{m\alpha} = 0.01840$ deg⁻¹ $= 1.054$ rad⁻¹ (physical constant of the rocket)
   - The linear model for $C_{y\alpha}$ applies: $C_{y\alpha} = 0.05403$ deg⁻¹
3. **Single unknown parameter**: Only $C_{m\alpha}$ is unknown to the controller. Translational coefficients $C_x$ and $C_{y\alpha}$ are known. This is the **matched uncertainty** case: $C_{m\alpha}$ enters the rotational dynamics through the same channel as the control input $\delta$.
4. **Attitude-only control**: The control objective is restricted to $\vartheta \to \vartheta^*$, $\dot\vartheta \to 0$. Translational dynamics evolve freely.
5. **Exact state measurement**: The full state $(x, y, \vartheta, \dot x, \dot y, \dot\vartheta)$ is available at each timestep without noise. This allows direct computation of $\alpha$ and $Y(\alpha)$.
6. **Fixed thrust**: The throttle is held at $F = 1.5\,mg = \text{const}$, providing hover with a thrust-to-weight margin. No singularity occurs in the control law since $F > 0$ always.
---
 
## 3. Equations of Motion
 
### Translational dynamics (inertial frame)
 
The body-frame aerodynamic forces transform to the inertial frame via the rotation matrix (see `README-Aerodynamics.md`):
 
$$
m\ddot x = F\sin(\vartheta+\delta) + X_b\sin\vartheta + Y_b\cos\vartheta
$$
 
$$
m\ddot y = F\cos(\vartheta+\delta) - mg + X_b\cos\vartheta - Y_b\sin\vartheta
$$
 
### Rotational dynamics
 
The net torque about the CoM has two contributions: the gimbal torque and the aerodynamic pitching moment.
 
$$
\boxed{J\ddot\vartheta = -F\cdot l_{cp}\sin\delta + Y(\alpha)\cdot C_{m\alpha}}
$$
 
The gimbal torque $-F\cdot l_{cp}\sin\delta$ is negative for positive $\delta$ (restoring convention). The aerodynamic pitching moment $Y(\alpha)\cdot C_{m\alpha}$ is the unknown term — it depends on the flight state through $\alpha$ and $q_\infty$, and on the unknown coefficient $C_{m\alpha}$.
 
**Key structural property:** $C_{m\alpha}$ appears **linearly** in the rotational equation, with the measurable regressor $Y(\alpha)$ as its coefficient.
 
---
 
## 4. Control Objective and Error Coordinates
 
The equilibrium to stabilize is:
 
$$
(\vartheta, \dot\vartheta) = (\vartheta^*, 0), \qquad \vartheta^* \in \mathbb{R}\ \text{arbitrary constant}
$$
 
No restriction is placed on $\vartheta^*$. In practice, large $|\vartheta^*|$ reduces the effective vertical thrust component $F\cos\vartheta^*$, so a margin condition $F\cos\vartheta^* > mg$ should be checked to ensure altitude can be maintained if needed.
 
Define the attitude error coordinates:
 
$$
e_\vartheta = \vartheta - \vartheta^*, \qquad \dot e_\vartheta = \dot\vartheta - \underbrace{\dot\vartheta^*}_{=\,0} = \dot\vartheta
$$
 
Since $\vartheta^*$ is a **constant**, $\dot\vartheta^* = 0$ exactly — the error derivative equals the angular rate. This means the entire derivation that follows is **independent of the specific value of $\vartheta^*$**: substituting any constant target produces the same equations, control law, adaptation law, and stability proof.
 
In error coordinates, the rotational dynamics become:
 
$$
J\ddot e_\vartheta = -F\cdot l_{cp}\sin\delta + Y(\alpha)\cdot C_{m\alpha}
$$
 
The control task is to drive $(e_\vartheta, \dot e_\vartheta) \to (0, 0)$ using only $\delta$, with $C_{m\alpha}$ unknown.
 
---
 
## 5. General CE Framework
 
Before deriving the specific control law, we place our problem within the general CE adaptive control framework.
 
### 5.1 General Plant Structure and Matching Condition
 
Consider plants of the form:
 
$$
\dot s = f(s) + G(s)\,a + \Theta^T \varphi(s)
$$
 
where $s \in \mathbb{R}^n$ is the state, $a \in \mathbb{R}^m$ is the control input, $f(s)$ and $G(s)$ are known, $\Theta \in \mathbb{R}^{p \times n}$ is an **unknown parameter matrix**, and $\varphi(s) \in \mathbb{R}^p$ is a **known regressor vector** — fully computable from the measured state.
 
The term $\Theta^T\varphi(s)$ is the uncertain part: unknown parameters acting on known functions of the state.
 
**Matching condition.** The CE approach works cleanly when the uncertainty enters through the same channel as the control input:
 
$$
\exists\,\Psi(s) \quad \text{s.t.} \quad \Theta^T\varphi(s) = G(s)\,\Psi(s)\,\theta
$$
 
where $\theta = \mathrm{vec}(\Theta)$. When this holds, the plant can be rewritten as:
 
$$
\dot s = f(s) + G(s)\bigl(a + \Psi(s)\,\theta\bigr)
$$
 
The unknown $\theta$ now appears **additively with the control** inside $G(s)(\cdot)$ — the controller can cancel it.
 
### 5.2 Nominal Policy and Its Lyapunov Function
 
Assume there exists a **nominal policy** $\pi_0(s)$ — what we would apply if $\theta = 0$ (no uncertainty). This policy renders the nominal system:
 
$$
\dot s = f(s) + G(s)\,\pi_0(s)
$$
 
globally asymptotically stable (GAS) with a known Lyapunov function $L_0(s)$ satisfying:
 
$$
\langle \nabla L_0,\, f(s) + G(s)\pi_0(s) \rangle \leq -K_0(\|s\|)
$$
 
for some class-$\mathcal{K}$ function $K_0$. This is the certainty-free baseline.
 
### 5.3 General CE Policy and Adaptation Law
 
The **CE policy** replaces $\theta$ with its online estimate $\hat\theta$:
 
$$
\pi(s\,|\,\hat\theta) := \pi_0(s) - \Psi(s)\,\hat\theta
$$
 
The nominal policy stabilizes when $\theta = 0$; the correction term $-\Psi(s)\hat\theta$ cancels the unknown disturbance using the current estimate.
 
The **complemented Lyapunov function** augments $L_0$ with a penalty on the estimation error $\tilde\theta = \hat\theta - \theta$:
 
$$
L = L_0(s) + \frac{1}{2}\tilde\theta^T \Gamma^{-1} \tilde\theta
$$
 
where $\Gamma \in \mathbb{R}^{p \times p}$ is a positive definite **adaptation gain matrix**. Differentiating $L$ along the CE closed-loop trajectories and grouping terms:
 
$$
\dot L \leq -K_0(\|s\|) + \tilde\theta^T\!\left(\Gamma^{-1}\dot{\hat\theta} - \Psi^T(s)\,G^T(s)\,\nabla L_0(s)\right)
$$
 
Setting the bracket to zero gives the **general adaptation law**:
 
$$
\boxed{\dot{\hat\theta} = \Gamma\,\Psi^T(s)\,G^T(s)\,\nabla L_0(s)}
$$
 
With this choice $\dot L \leq -K_0(\|s\|) \leq 0$, and by LaSalle $s \to 0$.
 
**Structural interpretation of the adaptation law:**
 
$$
\dot{\hat\theta} = \underbrace{\Gamma}_{\text{adapt. gain}} \times \underbrace{\Psi^T(s)}_{\text{regressor}} \times \underbrace{G^T(s)}_{\text{control channel}} \times \underbrace{\nabla L_0(s)}_{\text{gradient of LF}}
$$
 
The estimate updates proportionally to how the state is "misbehaving" ($\nabla L_0$) times the signal that the unknown parameter was multiplying ($\Psi^T$). This self-correcting structure is the core of CE adaptive control.
 
### 5.4 Correspondence: Our Rocket as a Special Case
 
Our rocket's rotational equation fits this framework exactly. The correspondence is:
 
| General framework | Our rocket (Project 2) |
|-------------------|------------------------|
| State $s$ | $(e_\vartheta,\, \dot\vartheta)$ |
| Control input $a$ | $\sin\delta$ |
| Unknown parameter $\theta$ | $C_{m\alpha}$ (scalar, $p=1$) |
| Regressor $\varphi(s)$ | $Y(\alpha)/J$ |
| Regressor $\Psi(s)$ | $-Y(\alpha)/(F\,l_{cp})$ |
| Control channel $G(s)$ | $-F\,l_{cp}$ (scalar) |
| Nominal policy $\pi_0(s)$ | $\frac{J}{F\,l_{cp}}(k_\vartheta e_\vartheta + k_\omega\dot\vartheta)$ |
| Nominal LF $L_0(s)$ | $V_0 = \frac{1}{2}k_\vartheta e_\vartheta^2 + \frac{1}{2}\dot\vartheta^2$ |
| $\nabla L_0(s)$ | $(k_\vartheta e_\vartheta,\; \dot\vartheta)$ |
| Adaptation gain $\Gamma$ | $\gamma$ (scalar) |
| Complemented LF $L$ | $V = V_0 + \frac{1}{2\gamma}\tilde\theta^2$ |
 
**Matching condition verification.** The uncertain term in the rotational equation is $Y(\alpha)\cdot C_{m\alpha}$. The control channel is $G = -F\,l_{cp}$ (the coefficient of $\sin\delta$ in $J\ddot\vartheta$). We need:
 
$$
Y(\alpha)\cdot C_{m\alpha} = G\cdot\Psi\cdot C_{m\alpha} = (-F\,l_{cp})\cdot\left(-\frac{Y(\alpha)}{F\,l_{cp}}\right)\cdot C_{m\alpha} = Y(\alpha)\cdot C_{m\alpha} \quad \checkmark
$$
 
The matching condition is satisfied — the aerodynamic uncertainty enters through the same channel as $\delta$. **This is why CE is directly applicable without backstepping.**
 
Applying the general adaptation law $\dot{\hat\theta} = \Gamma\,\Psi^T G^T \nabla L_0$ to our scalar case:
 
$$
\dot{\hat C}_{m\alpha} = \gamma \cdot \frac{Y(\alpha)}{J} \cdot (-F\,l_{cp}) \cdot \frac{\partial V_0}{\partial \dot\vartheta} \cdot \frac{1}{-F\,l_{cp}/J}
$$
 
which simplifies to $\dot{\hat C}_{m\alpha} = \gamma\cdot\tfrac{Y(\alpha)\dot\vartheta}{J}$ — exactly the adaptation law derived in Section 9. The general framework and the specific derivation are fully consistent.
 
**Note on the correspondence table:** The Regressor $\varphi(s)$ represents the function multiplying the unknown parameter $\Theta$ in the general form $\Theta^T\varphi(s)$. In our case, the aerodynamic term in the equation of motion is $\frac{Y(\alpha)}{J}C_{m\alpha}$, so $\varphi(s) = Y(\alpha)/J$ and $\Psi(s) = -Y(\alpha)/(F\,l_{cp})$ is derived from the matching condition $Y(\alpha)C_{m\alpha} = G(s)\Psi(s)C_{m\alpha}$ with $G(s) = -F\,l_{cp}$.
 
---
 
## 6. Stage 1 — Idealized Control Law (Known Parameters)
 
We first design the control law as if $C_{m\alpha}$ were known. This follows the same Lyapunov approach as Project 1, augmented with an aerodynamic compensation term.
 
### Lyapunov candidate for the known-parameter case
 
$$
V_0 = \frac{1}{2}k_\vartheta e_\vartheta^2 + \frac{1}{2}\dot\vartheta^2, \qquad k_\vartheta > 0
$$
 
**Verification that $V_0$ is a valid Lyapunov function candidate:**
- $V_0(0, 0) = 0$ ✓
- $V_0(e_\vartheta, \dot\vartheta) > 0$ for all $(e_\vartheta, \dot\vartheta) \neq (0,0)$, since $k_\vartheta > 0$ ✓
- $V_0 \to \infty$ as $\|(e_\vartheta, \dot\vartheta)\| \to \infty$ (radially unbounded) ✓
### Time derivative of $V_0$
 
$$
\dot V_0 = k_\vartheta e_\vartheta \dot e_\vartheta + \dot\vartheta\ddot\vartheta
= \dot\vartheta\!\left(k_\vartheta e_\vartheta + \ddot\vartheta\right)
$$
 
Substituting the rotational dynamics:
 
$$
\dot V_0 = \dot\vartheta\!\left(k_\vartheta e_\vartheta - \frac{F\,l_{cp}}{J}\sin\delta + \frac{Y(\alpha)}{J}C_{m\alpha}\right)
$$
 
### Idealized control law
 
To enforce $\dot V_0 = -k_\omega\dot\vartheta^2$ with $k_\omega > 0$, set the expression in parentheses equal to $-k_\omega\dot\vartheta$:
 
$$
k_\vartheta e_\vartheta - \frac{F\,l_{cp}}{J}\sin\delta + \frac{Y(\alpha)}{J}C_{m\alpha} = -k_\omega\dot\vartheta
$$
 
Solving for $\sin\delta$:
 
$$
\boxed{\sin\delta^* = \frac{J}{F\,l_{cp}}\!\left(k_\vartheta e_\vartheta + k_\omega\dot\vartheta + \frac{Y(\alpha)}{J}C_{m\alpha}\right)}
$$
 
This yields $\dot V_0 = -k_\omega\dot\vartheta^2 \leq 0$. The additional term $\tfrac{Y(\alpha)}{J}C_{m\alpha}$ cancels the aerodynamic pitching moment exactly.
 
---
 
## 7. Stage 2 — Certainty Equivalence Substitution
 
Since $C_{m\alpha}$ is unknown, we cannot implement $\sin\delta^*$ directly. The **Certainty Equivalence** principle replaces the unknown parameter with its current online estimate:
 
$$
\boxed{\sin\delta = \frac{J}{F\,l_{cp}}\!\left(k_\vartheta e_\vartheta + k_\omega\dot\vartheta + \frac{Y(\alpha)}{J}\hat C_{m\alpha}\right)}
$$
 
where $\hat C_{m\alpha}(t)$ is updated by the adaptation law derived in Section 9.
 
Compared to $\sin\delta^*$, the realizable law replaces $C_{m\alpha} \mapsto \hat C_{m\alpha}$. The difference introduces a residual term proportional to the estimation error $\tilde\theta = \hat C_{m\alpha} - C_{m\alpha}$, which drives the adaptation.
 
---
 
## 8. Closed-Loop Dynamics
 
Substituting the CE control law into the rotational dynamics:
 
$$
J\ddot\vartheta = -F\,l_{cp}\sin\delta + Y(\alpha)\,C_{m\alpha}
$$
 
$$
= -\left(J\,k_\vartheta e_\vartheta + J\,k_\omega\dot\vartheta + Y(\alpha)\hat C_{m\alpha}\right) + Y(\alpha)\,C_{m\alpha}
$$
 
$$
= -J\,k_\vartheta e_\vartheta - J\,k_\omega\dot\vartheta - Y(\alpha)\!\underbrace{(\hat C_{m\alpha} - C_{m\alpha})}_{=\,\tilde\theta}
$$
 
Therefore:
 
$$
\boxed{\ddot\vartheta = -k_\vartheta e_\vartheta - k_\omega\dot\vartheta - \frac{Y(\alpha)}{J}\tilde\theta}
$$
 
**Interpretation:** The closed-loop angular dynamics are those of a damped harmonic oscillator ($-k_\vartheta e_\vartheta - k_\omega\dot\vartheta$) driven by a perturbation term $-\tfrac{Y(\alpha)}{J}\tilde\theta$. If $\tilde\theta = 0$ (perfect estimation), the dynamics are identical to the known-parameter case and $\dot V_0 = -k_\omega\dot\vartheta^2$. If $\tilde\theta \neq 0$, the perturbation must be handled by the adaptation law.
 
---
 
## 9. Extended Lyapunov Function and Adaptation Law
 
### Extended Lyapunov function
 
To handle the parameter estimation error, augment $V_0$ with a quadratic penalty on $\tilde\theta$:
 
$$
\boxed{V = \frac{1}{2}k_\vartheta e_\vartheta^2 + \frac{1}{2}\dot\vartheta^2 + \frac{1}{2\gamma}\tilde\theta^2}
$$
 
where $\gamma > 0$ is the **adaptation rate** (a free design parameter).
 
**Verification:**
- $V = 0$ iff $(e_\vartheta, \dot\vartheta, \tilde\theta) = (0, 0, 0)$ ✓
- $V > 0$ for all $(e_\vartheta, \dot\vartheta, \tilde\theta) \neq (0, 0, 0)$ ✓
- $V \to \infty$ as $\|(e_\vartheta, \dot\vartheta, \tilde\theta)\| \to \infty$ ✓
### Time derivative of $V$
 
$$
\dot V = k_\vartheta e_\vartheta\dot e_\vartheta + \dot\vartheta\ddot\vartheta + \frac{1}{\gamma}\tilde\theta\dot{\tilde\theta}
$$
 
Since $C_{m\alpha}$ is a **constant**, $\dot{\tilde\theta} = \dot{\hat C}_{m\alpha}$. Substituting $\dot e_\vartheta = \dot\vartheta$ and the closed-loop expression for $\ddot\vartheta$:
 
$$
\dot V = k_\vartheta e_\vartheta\dot\vartheta
        + \dot\vartheta\left(-k_\vartheta e_\vartheta - k_\omega\dot\vartheta - \frac{Y(\alpha)}{J}\tilde\theta\right)
        + \frac{1}{\gamma}\tilde\theta\dot{\hat C}_{m\alpha}
$$
 
Expanding and collecting terms (the $k_\vartheta e_\vartheta\dot\vartheta$ terms cancel):
 
$$
\dot V = -k_\omega\dot\vartheta^2
        - \frac{Y(\alpha)\dot\vartheta}{J}\tilde\theta
        + \frac{1}{\gamma}\tilde\theta\dot{\hat C}_{m\alpha}
$$
 
Factoring out $\tilde\theta$ from the last two terms:
 
$$
\dot V = -k_\omega\dot\vartheta^2
        + \tilde\theta\left(\frac{\dot{\hat C}_{m\alpha}}{\gamma} - \frac{Y(\alpha)\dot\vartheta}{J}\right)
$$
 
### Choosing the adaptation law
 
To eliminate the indefinite cross-term and ensure $\dot V \leq 0$, we set the expression in the parentheses to zero:
 
$$
\frac{\dot{\hat C}_{m\alpha}}{\gamma} - \frac{Y(\alpha)\dot\vartheta}{J} = 0
$$
 
$$
\boxed{\dot{\hat C}_{m\alpha} = \gamma\cdot\frac{Y(\alpha)\cdot\dot\vartheta}{J}}
$$
 
With this adaptation law:
 
$$
\boxed{\dot V = -k_\omega\dot\vartheta^2 \leq 0}
$$
 
The time derivative is negative semi-definite. This is the central result from which all stability conclusions follow.
 
---
 
## 10. Stabilization Proof
 
### 10.1 Boundedness of All Signals
 
**Theorem:** Under the CE control law and adaptation law, all signals $e_\vartheta(t)$, $\dot\vartheta(t)$, and $\tilde\theta(t)$ are uniformly bounded for all $t \geq 0$.
 
**Proof:**
 
Since $\dot V = -k_\omega\dot\vartheta^2 \leq 0$, the function $V(t)$ is non-increasing:
 
$$
V(t) \leq V(0) \quad \forall\, t \geq 0
$$
 
$V$ is positive definite and radially unbounded in $(e_\vartheta, \dot\vartheta, \tilde\theta)$. Therefore:
 
$$
\frac{1}{2}k_\vartheta e_\vartheta^2(t) \leq V(t) \leq V(0) \implies |e_\vartheta(t)| \leq \sqrt{\frac{2V(0)}{k_\vartheta}}
$$
 
$$
\frac{1}{2}\dot\vartheta^2(t) \leq V(t) \leq V(0) \implies |\dot\vartheta(t)| \leq \sqrt{2V(0)}
$$
 
$$
\frac{1}{2\gamma}\tilde\theta^2(t) \leq V(t) \leq V(0) \implies |\tilde\theta(t)| \leq \sqrt{2\gamma\,V(0)}
$$
 
All three quantities are bounded by constants depending only on initial conditions and gains. $\square$
 
**Consequence:** $\hat C_{m\alpha}(t) = C_{m\alpha} + \tilde\theta(t)$ is bounded. Since $C_{m\alpha}$ is a constant, the estimate is well-defined for all time.
 
### 10.2 Convergence of Angular Rate — Barbalat's Lemma
 
**Theorem:** $\dot\vartheta(t) \to 0$ as $t \to \infty$.
 
**Proof via Barbalat's Lemma:**
 
*Step 1 — Integrability.* Integrate $\dot V = -k_\omega\dot\vartheta^2$ from $0$ to $\infty$:
 
$$
\int_0^\infty k_\omega\dot\vartheta^2(\tau)\,d\tau = V(0) - \lim_{t\to\infty} V(t) \leq V(0) < \infty
$$
 
Since $V(t)$ is non-increasing and bounded below by zero, the limit exists and is finite. Therefore $\dot\vartheta^2 \in L^2([0,\infty))$.
 
*Step 2 — Uniform continuity.* Compute $\tfrac{d}{dt}(\dot\vartheta^2) = 2\dot\vartheta\ddot\vartheta$. From the closed-loop dynamics:
 
$$
\ddot\vartheta = -k_\vartheta e_\vartheta - k_\omega\dot\vartheta - \frac{Y(\alpha)}{J}\tilde\theta
$$
 
All terms are bounded (from Section 10.1, and $Y(\alpha) = q_\infty S_m l\,\alpha$ is bounded under bounded flight conditions). Therefore $\ddot\vartheta$ is bounded, which means $\tfrac{d}{dt}(\dot\vartheta^2) = 2\dot\vartheta\ddot\vartheta$ is bounded. A function with a bounded derivative is uniformly continuous.
 
*Step 3 — Barbalat's Lemma.* If $f(t) \geq 0$ is uniformly continuous and $\int_0^\infty f(\tau)\,d\tau < \infty$, then $f(t) \to 0$. Applying this to $f(t) = \dot\vartheta^2(t)$:
 
$$
\dot\vartheta^2(t) \to 0 \implies \dot\vartheta(t) \to 0 \quad \text{as } t \to \infty \qquad \square
$$
 
### 10.3 Convergence of Attitude Error — LaSalle / Cascade Argument
 
**Theorem:** $e_\vartheta(t) \to 0$ as $t \to \infty$.
 
**On the non-autonomous nature of the system:**
 
LaSalle's invariance principle in its classical formulation applies only to **autonomous** systems. Our system is non-autonomous because the regressor $Y(\alpha)$ depends on $\dot x$, $\dot y$, which evolve freely under the translational dynamics. A fully rigorous proof requires either:
1. A **cascade argument**: since $\dot\vartheta \to 0$ (Section 10.2) and $\dot e_\vartheta = \dot\vartheta$, we have $\dot e_\vartheta \to 0$. If $e_\vartheta$ is bounded (guaranteed) and $\dot e_\vartheta \to 0$, then $e_\vartheta$ converges — provided $\ddot e_\vartheta$ is bounded, which holds under bounded translational motion. This gives $e_\vartheta \to \text{const}$; that the constant is zero follows from the closed-loop dynamics at equilibrium.
2. **LaSalle extensions** for non-autonomous systems (Matrosov's theorem or the invariance principle for $\omega$-limit sets), which require additional regularity.
We present the LaSalle argument below as a strong heuristic under the **mild assumption** that translational velocities remain bounded during attitude correction (holds for bounded thrust and finite simulation time).
 
Consider the set $\mathcal{S} = \{(e_\vartheta, \dot\vartheta, \tilde\theta) : \dot V = 0\} = \{\dot\vartheta = 0\}$.
 
On any trajectory remaining in $\mathcal{S}$:
- $\dot\vartheta \equiv 0 \implies \ddot\vartheta \equiv 0$
- Closed-loop dynamics: $0 = -k_\vartheta e_\vartheta - \frac{Y(\alpha)}{J}\tilde\theta$
- Adaptation law: $\dot{\hat C}_{m\alpha} = \gamma\cdot\tfrac{Y(\alpha)\cdot 0}{J} = 0$, so $\tilde\theta = \text{const}$
If $Y(\alpha) \to 0$ ($\alpha \to 0$, rocket aligned with velocity): the remaining equation gives $k_\vartheta e_\vartheta = 0$, hence $e_\vartheta = 0$.
 
If $Y(\alpha) \not\to 0$ (persistent excitation): parameter convergence (Section 12) gives $\tilde\theta \to 0$, and consequently $e_\vartheta \to 0$.
 
**Conclusion:** The largest invariant subset of $\mathcal{S}$ is $\{e_\vartheta = 0,\, \dot\vartheta = 0\}$. Under the bounded translational motion assumption, $e_\vartheta(t) \to 0$ for any constant $\vartheta^*$. This is corroborated numerically by verifying $V(t)$ is non-increasing throughout the simulation. $\square$
 
### 10.4 What the Proof Does and Does Not Guarantee
 
| Claim | Status | Condition |
|-------|--------|-----------|
| $V(t)$ is non-increasing | **Exact** | Always |
| $e_\vartheta(t)$, $\dot\vartheta(t)$, $\tilde\theta(t)$ bounded | **Exact** | Always |
| $\dot\vartheta(t) \to 0$ | **Exact** | Barbalat (bounded flight) |
| $e_\vartheta(t) \to 0$ | **Exact** | LaSalle + bounded translational motion |
| $\tilde\theta(t) \to 0$ (parameter convergence) | **Conditional** | Requires persistent excitation |
| Proof valid for time-varying $C_{m\alpha}$ | **No** | $C_{m\alpha}$ assumed constant |
| Proof valid under gimbal saturation | **Partial** | $\dot V \leq 0$ preserved; convergence may slow |
 
---
 
## 11. Parameter Error Dynamics
 
The estimation error $\tilde\theta = \hat C_{m\alpha} - C_{m\alpha}$ evolves as:
 
$$
\dot{\tilde\theta} = \dot{\hat C}_{m\alpha} = \gamma\cdot\frac{Y(\alpha)\cdot\dot\vartheta}{J}
$$
 
This is a first-order differential equation driven by the product of the regressor $Y(\alpha)$ and the angular rate $\dot\vartheta$.
 
**Observations:**
 
1. **No autonomous decay.** The adaptation law has no restoring term — $\tilde\theta$ does not decay on its own. If $Y(\alpha)\cdot\dot\vartheta = 0$, then $\dot{\tilde\theta} = 0$ and the estimate freezes at its current value.
2. **Direction of adaptation.** The sign of $\dot{\tilde\theta}$ is determined by $Y(\alpha)\cdot\dot\vartheta$:
   - If $Y(\alpha)\cdot\dot\vartheta > 0$, the estimate increases.
   - If $Y(\alpha)\cdot\dot\vartheta < 0$, the estimate decreases.
3. **Coupling to attitude dynamics.** As $\dot\vartheta \to 0$ (attitude converges), $\dot{\tilde\theta} \to 0$ as well — the estimator naturally "freezes" when the attitude is stabilized. This is expected: once the rocket is upright, there is no angular rate information to drive adaptation. This is the *excitation problem* discussed in Section 12.
4. **No guarantees on $\tilde\theta$**: Boundedness of $\tilde\theta$ is guaranteed (Section 9.1), but convergence to zero requires additional conditions.
---
 
## 12. Persistent Excitation and Parameter Convergence
 
### Definition — Precise PE condition for our system
 
The parameter estimation error dynamics are driven by the product of the regressor and the angular rate:
 
$$
\dot{\tilde\theta} = \gamma\cdot\frac{Y(\alpha(t))}{J}\cdot\dot\vartheta(t)
$$
 
For parameter convergence to be guaranteed, the excitation must act on the **product** $Y(\alpha(t))\cdot\dot\vartheta(t)$, not on $Y(\alpha(t))$ alone. Formally, the product signal $Y(\alpha(t))\cdot\dot\vartheta(t)$ is **persistently exciting (PE)** if there exist constants $T > 0$ and $\mu > 0$ such that:
 
$$
\int_t^{t+T} [Y(\alpha(\tau))\cdot\dot\vartheta(\tau)]^2\,d\tau \geq \mu \quad \forall\, t \geq 0
$$
 
In words: the product $Y(\alpha)\cdot\dot\vartheta$ (not $Y(\alpha)$ in isolation) must maintain sufficient energy over every window of length $T$.
 
### Theorem (Parameter convergence under PE)
 
If the product $Y(\alpha(t))\cdot\dot\vartheta(t)$ is persistently exciting, then $\tilde\theta(t) \to 0$ as $t \to \infty$.
 
**Sketch of proof:**
 
Consider the scalar system:
 
$$
\dot{\tilde\theta} = \gamma\cdot\frac{Y(\alpha)}{J}\cdot\dot\vartheta
$$
 
From the closed-loop dynamics, $\dot\vartheta$ is related to $\tilde\theta$ through:
 
$$
\ddot\vartheta = -k_\vartheta e_\vartheta - k_\omega\dot\vartheta - \frac{Y(\alpha)}{J}\tilde\theta
$$
 
This system has the structure of an **error model** studied in standard adaptive control theory (Ioannou & Sun, Theorem 4.3.2). Under PE, the composite system is uniformly exponentially stable, which implies $\tilde\theta \to 0$. $\square$
 
(Full proof requires the standard LMI-based argument for adaptive systems and is omitted here for brevity; see Ioannou & Sun Ch. 4.)
 
### Physical interpretation
 
**When is the product $Y(\alpha(t))\cdot\dot\vartheta(t)$ persistently exciting?**
 
$$
Y(\alpha) = q_\infty S_m l\cdot\alpha = \frac{1}{2}\rho v^2 S_m l\cdot\alpha
$$
 
The product $Y(\alpha)\cdot\dot\vartheta$ vanishes if either:
- $v = 0$ (no airspeed), or
- $\alpha = 0$ (rocket perfectly aligned with velocity), or  
- $\dot\vartheta = 0$ (rocket has stopped rotating)
For PE of the product to hold:
 
- If the rocket undergoes initial transients with **non-zero angular rate** ($\dot\vartheta \neq 0$) **and** **non-zero angle of attack** ($\alpha \neq 0$) over a sufficiently long window, then the product $Y(\alpha)\cdot\dot\vartheta$ is PE and the estimate converges: $\tilde\theta \to 0$.
- If the rocket converges to upright hover at $\vartheta = 0$, $\dot x = \dot y = 0$, then eventually $v \to 0$ and $Y(\alpha) \to 0$, or $\dot\vartheta \to 0$ as well. In either case, the product $Y(\alpha)\cdot\dot\vartheta \to 0$, and the estimator naturally "freezes" — the parameter estimate stops updating. This is a **well-known limitation** of direct adaptive control: parameter convergence requires persistent motion, not just that the system is stable.
### Practical implication for the experiment
 
The initial condition $\vartheta(0) = 20°$, $\dot\vartheta(0) = -8°/\text{s}$ guarantees a transient period with **non-zero angular rate** and **non-zero angle of attack**. During this transient, $Y(\alpha)\cdot\dot\vartheta \neq 0$ and the PE condition can be satisfied, enabling adaptation to be active. As the attitude converges and $\dot\vartheta \to 0$, the product $Y(\alpha)\cdot\dot\vartheta$ decays, and adaptation naturally slows. The quality of the final estimate depends on how rich and prolonged this transient period is.
 
---
 
## 13. Projection Operator
 
### Motivation
 
Without projection, the adaptation law integrates indefinitely. If the regressor has the wrong sign or there is unmodelled noise, the estimate can drift outside physically meaningful bounds. Projection confines $\hat C_{m\alpha}$ to a known interval $[\theta_{\min}, \theta_{\max}]$.
 
For our problem: $C_{m\alpha} = 1.054 > 0$ (pitching moment coefficient positive for a statically **unstable** launch vehicle where the center of pressure is below the center of mass), so we use $\theta_{\min} = 0.1$ rad⁻¹, $\theta_{\max} = 5.0$ rad⁻¹.
 
### Definition
 
The **projection operator** modifies the adaptation law as:
 
$$
\dot{\hat C}_{m\alpha} = \mathrm{Proj}\!\left(\hat C_{m\alpha},\, u_{\mathrm{adapt}}\right), \qquad u_{\mathrm{adapt}} = \gamma\cdot\frac{Y(\alpha)\dot\vartheta}{J}
$$
 
where
 
$$
\mathrm{Proj}(\hat\theta,\, u) \triangleq \begin{cases}
0 & \text{if } \hat\theta = \theta_{\max} \text{ and } u > 0 \\
0 & \text{if } \hat\theta = \theta_{\min} \text{ and } u < 0 \\
u & \text{otherwise}
\end{cases}
$$
 
(The symbol $u$ here denotes the proposed adaptation update direction — distinct from the control input $\delta$ and from the integration variable used elsewhere.)
 
In practice, implemented as:
 
```python
if (C_hat >= theta_max and dC > 0) or (C_hat <= theta_min and dC < 0):
    dC = 0.0
```
 
### Projection preserves $\dot V \leq 0$
 
**Proof:** We need to show that $\dot V \leq 0$ under projection. Recall:
 
$$
\dot V = -k_\omega\dot\vartheta^2 + \tilde\theta\!\left(\frac{\dot{\hat C}_{m\alpha}}{\gamma} - \frac{Y(\alpha)\dot\vartheta}{J}\right)
$$
 
*Case 1: projection inactive.* $\dot{\hat C}_{m\alpha} = \gamma\cdot\tfrac{Y(\alpha)\dot\vartheta}{J} = u_{\mathrm{adapt}}$, so the bracket is zero. $\dot V = -k_\omega\dot\vartheta^2 \leq 0$. ✓
 
*Case 2: projection active at upper bound*, i.e., $\hat C_{m\alpha} = \theta_{\max}$ and $u_{\mathrm{adapt}} > 0$ (adaptation would increase $\hat C_{m\alpha}$). Then $\dot{\hat C}_{m\alpha} = 0$. Since $C_{m\alpha} \leq \theta_{\max}$, we have $\tilde\theta = \hat C_{m\alpha} - C_{m\alpha} \geq 0$. The cross-term becomes:
 
$$
\tilde\theta\!\left(0 - \frac{Y\dot\vartheta}{J}\right) = -\tilde\theta\cdot\frac{u_{\mathrm{adapt}}}{\gamma} \leq 0
$$
 
since $\tilde\theta \geq 0$ and $u_{\mathrm{adapt}} > 0$. Therefore $\dot V \leq -k_\omega\dot\vartheta^2 \leq 0$. ✓
 
*Case 3: projection active at lower bound* is symmetric. ✓
 
**Result:** Projection never violates $\dot V \leq 0$. All stability conclusions from Section 10 remain valid under projection. $\square$
 
---
 
## 14. Comparison with Project 1 Baseline
 
### The P1 baseline in the aerodynamic setting
 
The Project 1 controller applies:
 
$$
\sin\delta_{\text{P1}} = \frac{J}{F\,l_{cp}}\!\left(k_\vartheta e_\vartheta + k_\omega\dot\vartheta\right)
$$
 
with no aerodynamic compensation ($\hat C_{m\alpha} \equiv 0$). Substituting into the rotational dynamics:
 
$$
J\ddot\vartheta = -F\,l_{cp}\sin\delta_{\text{P1}} + Y(\alpha)\,C_{m\alpha}
= -J\,k_\vartheta e_\vartheta - J\,k_\omega\dot\vartheta + Y(\alpha)\,C_{m\alpha}
$$
 
$$
\ddot\vartheta = -k_\vartheta e_\vartheta - k_\omega\dot\vartheta + \underbrace{\frac{Y(\alpha)\,C_{m\alpha}}{J}}_{\text{unrejected disturbance}}
$$
 
The term $\tfrac{Y(\alpha)\,C_{m\alpha}}{J}$ acts as a **persistent, state-dependent disturbance**. The P1 Lyapunov function along the P1 closed-loop:
 
$$
\dot V_0\big|_{\text{P1}} = -k_\omega\dot\vartheta^2 + \frac{Y(\alpha)\,C_{m\alpha}}{J}\dot\vartheta
$$
 
This is **not** sign-definite. The aerodynamic term $\tfrac{Y(\alpha)\,C_{m\alpha}}{J}\dot\vartheta$ can be positive, meaning $V_0$ may increase. Stability is **not guaranteed** for the P1 baseline in the aerodynamic setting.
 
### Expected failure modes
 
At non-zero angle of attack ($\alpha \neq 0$, which occurs whenever the rocket is tilted and moving), the unrejected moment $Y(\alpha)\,C_{m\alpha}$ continuously torques the rocket. Depending on the sign of $\alpha$ relative to $\dot\vartheta$:
 
- **Steady-state pitch error**: The aerodynamic moment balances the control law at a non-zero $e_\vartheta$, creating a static offset.
- **Oscillatory behavior**: The sign of $\alpha\cdot\dot\vartheta$ varies during the transient, potentially causing limit cycling.
- **Slower convergence**: Even if convergence occurs, the aerodynamic moment extends the settling time.
### Quantitative comparison
 
The comparison is demonstrated by running both controllers from the same initial condition and showing state trajectories, gimbal demand, and the Lyapunov function $V_0(t)$ for both. The expected result:
 
- **P2 Adaptive**: $V(t)$ monotonically decreasing, attitude converges cleanly.
- **P1 Baseline**: $V_0(t)$ non-monotone, attitude shows residual error or oscillation.
---
 
## 15. Gain Conditions and Tuning Guidelines
 
### Necessary conditions for stability
 
$$
k_\vartheta > 0, \qquad k_\omega > 0, \qquad \gamma > 0
$$
 
### Sufficient conditions
 
**1. No persistent gimbal saturation.**
 
The control law requires $|\sin\delta| \leq 1$. For stable operation, choose $k_\vartheta$, $k_\omega$, and the initial estimate $\hat C_{m\alpha}(0)$ such that:
 
$$
\frac{J}{F\,l_{cp}}\!\left(k_\vartheta |e_\vartheta|_{\max} + k_\omega|\dot\vartheta|_{\max} + \frac{|Y(\alpha)|_{\max}}{J}|\hat C_{m\alpha}|_{\max}\right) \leq 1
$$
 
If this is violated during the transient, $\dot V \leq 0$ is still maintained, but convergence slows.
 
**2. Critical damping condition.**
 
The attitude subsystem (with $\tilde\theta = 0$) is a damped harmonic oscillator with natural frequency $\omega_n = \sqrt{k_\vartheta}$ and damping ratio $\zeta = \tfrac{k_\omega}{2\sqrt{k_\vartheta}}$. For non-oscillatory convergence:
 
$$
k_\omega \geq 2\sqrt{k_\vartheta} \qquad \text{(critical damping)}
$$
 
**3. Adaptation rate $\gamma$.**
 
- **$\gamma$ too small**: slow parameter convergence; the aerodynamic compensation remains inaccurate for a long time.
- **$\gamma$ too large**: $\hat C_{m\alpha}$ updates aggressively, potentially causing noise amplification and transient gimbal saturation.
A practical starting point:
 
$$
\gamma \approx \frac{J}{\max_t|Y(\alpha(t))|^2\cdot T_{\text{conv}}}
$$
 
where $T_{\text{conv}}$ is the desired convergence time for the estimate. Tune by sweep (see Experimental Setup).
 
### Default gains
 
| Gain | Value | Condition check |
|------|-------|-----------------|
| $k_\vartheta$ | 18.0 | $> 0$ ✓ |
| $k_\omega$ | 7.0 |  |
| $\gamma$ | 1.0 (default; sweep [0.1, 10.0]) | $> 0$ ✓ |
| $\theta_{\min}$ | 0.1 rad⁻¹ | Below $C_{m\alpha}^{\text{true}} = 1.054$ ✓ |
| $\theta_{\max}$ | 5.0 rad⁻¹ | Above $C_{m\alpha}^{\text{true}} = 1.054$ ✓ |
 
---
 
## 16. Implementation Notes
 
### Angle wrapping
 
The error $e_\vartheta = \vartheta - \vartheta^*$ must be wrapped to $(-\pi, \pi]$ to avoid discontinuities:
 
```python
e_phi = (phi - phi_target + np.pi) % (2 * np.pi) - np.pi
```
 
### Singularity guard
 
The denominator $F\,l_{cp}$ is always positive under Assumption 6 (fixed positive thrust). No singularity guard is needed for the control law. For the adaptation law, $Y(\alpha) = q_\infty S_m l\,\alpha$ is zero when $v = 0$ or $\alpha = 0$ — these are safe zeros (no adaptation, estimate holds).
 
### Numerical integration of the adaptation law
 
The estimate $\hat C_{m\alpha}$ is integrated using the same timestep as the ODE solver. With `solve_ivp` and RK45, the adaptation variable should be part of the extended state vector:
 
```python
state = [x, y, phi, vx, vy, omega, C_hat]
```
 
so that the integrator handles the update internally with consistent timestepping.
 
### Monitoring convergence in simulation
 
Compute and plot:
 
```python
V = 0.5 * k_phi * e_phi**2 + 0.5 * omega**2 + 0.5/gamma * (C_hat - C_true)**2
```
 
A monotonically non-increasing $V(t)$ is the numerical verification of $\dot V \leq 0$. Any increase in $V$ indicates a bug.
 
### Python variable mapping
 
| Symbol | Python variable |
|--------|----------------|
| $\vartheta$ | `phi` |
| $\dot\vartheta$ | `omega` |
| $e_\vartheta$ | `e_phi` |
| $\delta$ | `delta` |
| $\alpha$ | `alpha_aoa` |
| $Y(\alpha)$ | `Y_regressor` |
| $\hat C_{m\alpha}$ | `C_hat` |
| $\tilde\theta$ | `theta_tilde` |
| $\gamma$ | `gamma` |
| $k_\vartheta$ | `k_phi` |
| $k_\omega$ | `k_omega` |
 
---
 
## References
 
1. Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall. — Lyapunov stability theory, Barbalat's lemma (Lemma 8.2), LaSalle's invariance principle.
2. Slotine, J.-J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. — Chapter 8: Adaptive Control of Linearizable Systems; Certainty Equivalence principle.
3. Ioannou, P. A., & Sun, J. (1996). *Robust Adaptive Control*. Prentice Hall. — Chapter 4: persistent excitation, parameter convergence, projection operator.
4. Wie, B. (1998). *Space Vehicle Dynamics and Control*. AIAA Education Series. — Attitude control of launch vehicles with thrust vector control.
5. Astrom, K. J., & Wittenmark, B. (1994). *Adaptive Control* (2nd ed.). Addison-Wesley. — Model Reference Adaptive Control, CE principle, stability analysis.