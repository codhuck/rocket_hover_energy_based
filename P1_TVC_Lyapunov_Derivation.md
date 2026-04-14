# Project 1 — Planar TVC Rocket: Model Verification & Lyapunov Control Law

**System:** Planar thrust-vector-controlled rocket with variable mass and drag  
**Method:** Lyapunov-based stabilization  
**Document purpose:** Step-by-step verification of the physical model, derivation of the Lyapunov function and control law, with all assumptions flagged explicitly.

---

## 0. Notation Table

| Symbol | Meaning | Units |
|--------|---------|-------|
| x, y | Inertial horizontal / vertical position | m |
| φ | Pitch angle from vertical, positive rightward | rad |
| ẋ, ẏ | Inertial velocities | m/s |
| φ̇ | Angular rate | rad/s |
| m | Total instantaneous mass | kg |
| m_dry | Dry (structural) mass | kg |
| m_f | Remaining fuel mass, m − m_dry | kg |
| α | Throttle command, α ∈ [0, 1] | — |
| δ | Gimbal deflection from body axis, positive rightward | rad |
| δ_max | Maximum gimbal angle | rad |
| ṁ_max | Maximum mass flow rate (positive constant) | kg/s |
| v_e | Effective exhaust velocity | m/s |
| F_max | Maximum thrust = ṁ_max · v_e | N |
| F | Instantaneous thrust = α · F_max | N |
| β | Translational drag coefficient | kg/m |
| g | Gravitational acceleration | m/s² |
| h_dry | Height of dry structure CoM above nozzle exit | m |
| h_tank | Height of fuel tank center above nozzle exit | m |
| L_tank | Length of fuel tank | m |
| h_com | Height of combined CoM above nozzle exit | m |
| l_cp | Control moment arm (nozzle exit to CoM) | m |
| J | Total moment of inertia about CoM | kg·m² |
| J_dry_cm | Dry structure MOI about its own CoM | kg·m² |

---

## 1. Model Verification — Equation by Equation

### 1.1 Equation (1): Mass depletion

**Proposed:** ṁ = −α · ṁ_max

**Verdict: ✓ Correct.**

This is the standard mass depletion model. When throttle α = 1, mass decreases at the maximum rate. When α = 0, no fuel is consumed. The negative sign ensures mass decreases. This is consistent with the rocket equation convention.

---

### 1.2 Equation (5): Thrust

**Proposed:** F = α · ṁ_max · v_e = α · F_max

**Verdict: ✓ Correct.**

Standard rocket thrust relation. F_max = ṁ_max · v_e is the maximum thrust. Throttle α linearly scales the thrust. This is exact for a system with constant exhaust velocity (no pressure-dependent thrust correction), which is standard for a first-principles model.

---

### 1.3 Equations (2)–(3): Translational dynamics

**Proposed:**
```
ẍ = (F/m)·sin(φ+δ) − (β/m)·ẋ·√(ẋ²+ẏ²)                   (2)
ÿ = (F/m)·cos(φ+δ) − g − (β/m)·ẏ·√(ẋ²+ẏ²)               (3)
```

#### Claim 1: "The thrust angle from vertical is (φ+δ) for all φ and δ."

**Verdict: ✓ Correct — this is exact, not a small-angle approximation.**

Reasoning: The body axis is rotated by φ from vertical. The thrust vector is deflected by δ from the body axis. Therefore the thrust direction relative to the inertial vertical is (φ + δ). The decomposition into horizontal and vertical components gives:

- Horizontal thrust component: F · sin(φ + δ)
- Vertical thrust component: F · cos(φ + δ)

This is a kinematic identity using angle addition — no small-angle assumption is needed. It holds for arbitrary φ and δ, including the full nonlinear range. **Important sign convention note:** with φ measured from the +y (vertical) axis and positive rightward (clockwise in standard orientation), sin(φ + δ) correctly gives the horizontal projection and cos(φ + δ) gives the vertical projection.

#### Claim 6: "Writing drag as β·ẋ·v and β·ẏ·v avoids division by zero at v = 0."

**Verdict: ✓ Correct — mathematically equivalent and numerically safer.**

The physical drag force is F_drag = β · v² · (−v̂), where v̂ is the unit velocity vector. In component form:

- F_drag,x = −β · v² · (ẋ/v) = −β · ẋ · v
- F_drag,y = −β · v² · (ẏ/v) = −β · ẏ · v

where v = √(ẋ² + ẏ²). The two forms are algebraically identical when v ≠ 0. At v = 0, the ẋ·v form naturally gives zero (since ẋ = ẏ = 0 ⟹ v = 0, so the product is 0·0 = 0), whereas the ẋ/v form produces 0/0. **Good numerical practice.**

#### Claim 7: "No ṁ·ẋ or ṁ·ẏ terms because F already accounts for mass ejection."

**Verdict: ✓ Correct — no double-counting.**

The Meshchersky (variable-mass) equation for translation is:

```
m · a = F_ext + ṁ · v_rel
```

where v_rel is the exhaust velocity relative to the rocket. The thrust F = ṁ_max · α · v_e already equals −ṁ · v_e (since ṁ < 0), which *is* the ṁ · v_rel term. Adding a separate ṁ · ẋ term (which would represent momentum carried away at the rocket's own velocity) would be double-counting. In the standard derivation, one applies conservation of momentum to the rocket+exhaust system, and the result is exactly F = −ṁ · v_e acting on the rocket. The rocket's velocity drops out. **No correction needed.**

**Equations (2)–(3) verdict: ✓ Correct as written.**

---

### 1.4 Equation (4): Rotational dynamics

**Proposed:**
```
φ̈ = [−2·F·l_cp·sin(δ) − J̇·φ̇] / J                       (4)
```

This equation requires the most careful analysis. There are three claims to verify.

#### Claim 2: "The factor of 2 arises from the Meshchersky transport term."

**Verdict: ✗ INCORRECT — the factor of 2 is physically wrong. It should be 1, not 2.**

**Detailed reasoning:**

The proposed argument is that the gimballed exhaust carries angular momentum away, contributing an *additional* torque equal to F·l_cp·sin(δ) beyond the direct gimbal force moment. Let's examine this carefully.

**The direct gimbal torque:** When thrust F is applied at angle δ from the body axis at distance l_cp from the CoM, the torque about the CoM is:

```
τ_gimbal = −F · l_cp · sin(δ)
```

The negative sign is because positive δ (rightward gimbal) deflects thrust rightward, creating a restoring (negative, leftward) torque when the nozzle is below the CoM.

**The angular momentum transport (Meshchersky) argument:** For a variable-mass system, the angular momentum equation is:

```
d/dt(J·φ̇) = τ_ext + ṁ · (r × v_rel)_z
```

where the second term represents angular momentum carried away by the exhaust. However, **this term is already implicitly included in τ_gimbal**. Here's why:

The thrust force F = −ṁ · v_e acts along the exhaust direction. The torque τ_gimbal = r × F, where r is the vector from the CoM to the nozzle exit and F is the thrust force. Since F already equals the momentum flux −ṁ · v_e, the torque τ_gimbal = r × (−ṁ · v_e) already contains the Meshchersky transport contribution. 

To see this another way: in the standard rocket angular momentum analysis, one writes the angular momentum balance for the rocket body:

```
d/dt(J·φ̇) = −F · l_cp · sin(δ) − J̇ · φ̇    [INCORRECT if 2× is added]
```

The exhaust angular momentum transport is already captured by treating the thrust as an external force applied at the nozzle. Adding it again as a separate −F·l_cp·sin(δ) term would be **double-counting the same physical effect**.

The easiest way to confirm: consider the special case δ = 0 (no gimbal). The thrust is along the body axis and passes through the nozzle, not through the CoM. With l_cp ≠ 0, one might expect a moment — but when δ = 0, sin(δ) = 0, and both the single-factor and double-factor versions give zero torque. So this special case doesn't distinguish them.

Now consider an angular momentum balance from first principles for a small δ:

- The force F acts at the nozzle, perpendicular component ≈ F·δ (for small δ)
- Moment arm is l_cp
- Torque ≈ F · l_cp · δ

There is only *one* force acting at *one* point. There is no separate "momentum transport torque" — the momentum transport *is* the force. The factor should be **1, not 2**.

**Correction:**
```
τ_thrust = −F · l_cp · sin(δ)        [factor of 1, not 2]
```

#### Claim 3: "The J̇·φ̇ term sign is correct."

**Verdict: ✓ Correct sign, given the correction above.**

From the angular momentum equation:

```
d/dt(J·φ̇) = τ_net
J·φ̈ + J̇·φ̇ = τ_net
φ̈ = (τ_net − J̇·φ̇) / J
```

Since the rocket is losing mass, J̇ < 0 (moment of inertia decreases over time). Therefore −J̇·φ̇ is a positive quantity times φ̇. This means:

- If φ̇ > 0, the −J̇·φ̇ term contributes positively to φ̈ (angular velocity increases as the "ice skater" pulls arms in)
- This is physically correct: decreasing J at constant angular momentum means increasing φ̇

The sign is correct as written (with the minus sign in the numerator).

#### Corrected Equation (4):

```
φ̈ = [−F · l_cp · sin(δ) − J̇ · φ̇] / J                    (4*)
```

**The factor of 2 is removed.**

---

### 1.5 Equations (6)–(7): Center of mass

**Proposed:**
```
m_f(t) = m(t) − m_dry                                       (6)
h_com(t) = [m_dry · h_dry + m_f(t) · h_tank] / m(t)        (7)
```

#### Claim 4: "h_com is the mass-weighted average for a two-component system."

**Verdict: ✓ Correct.**

For a system of two rigid bodies with masses m₁, m₂ at positions h₁, h₂ along the body axis, the center of mass is:

```
h_com = (m₁·h₁ + m₂·h₂) / (m₁ + m₂)
```

Here m₁ = m_dry, h₁ = h_dry, m₂ = m_f, h₂ = h_tank, and m₁ + m₂ = m. This is the standard definition.

**Modeling assumption noted:** This treats the fuel as a point mass at h_tank (the tank center), meaning fuel burns uniformly from the tank without shifting the fuel CoM. This is a reasonable simplification — in practice, propellant depletion may shift the fuel CoM, but for a course project this is standard.

---

### 1.6 Equation (8): Moment arm

**Proposed:** l_cp(t) = h_com(t)

**Verdict: ✓ Correct, given coordinate origin at the nozzle exit.**

If the nozzle exit is at the origin of the body-frame vertical coordinate, then the distance from the nozzle to the CoM is simply h_com. This is the control moment arm for the gimbal torque.

---

### 1.7 Equation (9): Moment of inertia

**Proposed:**
```
J(t) = J_dry_cm + m_dry·(h_com − h_dry)²
     + (1/12)·m_f·L_tank² + m_f·(h_com − h_tank)²          (9)
```

#### Claim 5: "Parallel axis theorem is correctly applied for both components."

**Verdict: ✓ Correct.**

By the parallel axis theorem, the MOI of each component about the system CoM is:

```
J_i = J_i,cm + m_i · d_i²
```

where d_i is the distance from the component's own CoM to the system CoM.

- **Dry structure:** J_dry = J_dry_cm + m_dry · (h_com − h_dry)² ✓
- **Fuel:** J_fuel = J_fuel_cm + m_f · (h_com − h_tank)² where J_fuel_cm = (1/12)·m_f·L_tank² assumes the fuel is a uniform rod/cylinder of length L_tank ✓

The total J = J_dry + J_fuel. **Correctly applied.**

**Note for computing J̇:** Under the P1 simplification (J constant at midpoint mass), this is not needed. But for future projects, J̇ can be obtained by differentiating (9) with respect to time, using ṁ_f = ṁ = −α·ṁ_max and ḣ_com from (7).

---

### 1.8 P1 Simplifying Assumptions

1. **J and l_cp constant at midpoint mass:** Reasonable for Project 1. Under this assumption, J̇ = 0, and the J̇·φ̇ term drops out of Eq. (4*). This removes the variable-inertia coupling and makes the Lyapunov analysis significantly cleaner. It is a good first approximation when fuel mass is a moderate fraction of total mass.

2. **No rotational aerodynamic drag:** Reasonable at low speed and for a body with small cross-section at typical aspect ratios.

3. **m(t) known exactly:** Since Eq. (1) is integrated alongside the other states, m is part of the state vector and is known. This is exact in simulation; in practice it would require a propellant gauge.

4. **Quasi-static Lyapunov analysis for slowly varying mass:** This is the standard approach. The Lyapunov function is constructed assuming constant m, and then one argues that since m changes slowly compared to the controller bandwidth, the stability guarantee approximately holds. This is a common and accepted engineering approximation. It is not rigorous in a strict mathematical sense — a rigorous treatment would use parameter-dependent Lyapunov functions or converse Lyapunov arguments. **Flag this as an approximation in your report.**

---

## 2. Corrected Final Equation Set

With the factor-of-2 error corrected and P1 simplifications applied (J, l_cp constant; J̇ = 0):

### State vector:
```
q = [x, y, φ, ẋ, ẏ, φ̇, m]ᵀ
```

### Control inputs:
```
u = [α, δ]ᵀ,   α ∈ [0,1],   |δ| ≤ δ_max
```

### Equations of motion:

```
ṁ  = −α · ṁ_max                                              (1)

ẍ  = (α·F_max/m)·sin(φ+δ) − (β/m)·ẋ·√(ẋ²+ẏ²)              (2)

ÿ  = (α·F_max/m)·cos(φ+δ) − g − (β/m)·ẏ·√(ẋ²+ẏ²)          (3)

φ̈  = −(α·F_max·l_cp)·sin(δ) / J                             (4*)

F   = α · F_max                                               (5)
```

where J and l_cp are constants evaluated at midpoint mass m₀ − m_fuel_init/2.

### Parameters (constant under P1):
```
l_cp = h_com(m_mid)        where  m_mid = m₀ − m_fuel_init/2
J    = J(m_mid)            evaluated from Eq. (9) at m_mid
```

---

## 3. Lyapunov Control Law Derivation

### 3.1 Control Objective

Stabilize the rocket to a hover condition at a desired position (x_d, y_d) with zero pitch angle:

```
Target: x → x_d,  y → y_d,  φ → 0,  ẋ → 0,  ẏ → 0,  φ̇ → 0
```

At hover, the thrust must exactly balance gravity: α_hover = m·g / F_max (assuming φ = 0, δ = 0, zero drag).

### 3.2 Error Coordinates

Define errors relative to the hover target:

```
e_x  = x − x_d          ė_x = ẋ
e_y  = y − y_d          ė_y = ẏ
e_φ  = φ                ė_φ = φ̇
```

The error dynamics inherit the equations of motion directly:

```
ë_x = (α·F_max/m)·sin(φ+δ) − (β/m)·ẋ·v                     (E1)
ë_y = (α·F_max/m)·cos(φ+δ) − g − (β/m)·ẏ·v                  (E2)
ë_φ = −(α·F_max·l_cp)·sin(δ) / J                             (E3)
```

where v = √(ẋ² + ẏ²).

### 3.3 Lyapunov Candidate

We propose a candidate of the form:

```
V = ½·k_x·e_x² + ½·k_y·e_y² + ½·k_φ·e_φ²
  + ½·ẋ² + ½·ẏ² + ½·(J/m)·φ̇²
  + c_x·e_x·ẋ + c_y·e_y·ẏ + c_φ·e_φ·φ̇
```

**However, this general form makes the algebra very heavy.** For a clean, provable first project, we use a **decoupled, two-layer approach** that is standard in TVC literature. The idea is:

**Layer 1 (outer loop):** Treat the desired thrust direction as a virtual control for position. Determine what angles the thrust should point to stabilize (x, y).

**Layer 2 (inner loop):** Use the gimbal δ to drive φ toward the desired angle from Layer 1, stabilizing attitude.

This separation is justified when the attitude dynamics are faster than the translational dynamics (achievable by gain selection).

---

### 3.4 Outer Loop — Position Stabilization

**Step 1: Define desired accelerations.**

We want the position errors to converge. Choose desired accelerations as PD-like damped responses:

```
a_x_des = −k_px·e_x − k_dx·ẋ                                (O1)
a_y_des = −k_py·e_y − k_dy·ẏ + g                             (O2)
```

where k_px, k_dx, k_py, k_dy > 0 are gains. The +g term in (O2) provides gravity compensation so that the equilibrium corresponds to hover.

**Step 2: Compute required thrust magnitude and direction.**

If we could directly command the thrust vector, we would need:

```
(F/m)·sin(φ+δ) = a_x_des + (β/m)·ẋ·v                        
(F/m)·cos(φ+δ) = a_y_des + g + (β/m)·ẏ·v = a_y_des_total    
```

Define the total desired acceleration vector (including drag compensation):

```
A_x = a_x_des + (β/m)·ẋ·v = −k_px·e_x − k_dx·ẋ + (β/m)·ẋ·v
A_y = a_y_des + (β/m)·ẏ·v + g = −k_py·e_y − k_dy·ẏ + (β/m)·ẏ·v + g
```

Wait — let me be more careful. Substituting the desired accelerations into the EOM:

From (E1): We need (α·F_max/m)·sin(φ+δ) − (β/m)·ẋ·v = a_x_des

So: (α·F_max/m)·sin(φ+δ) = a_x_des + (β/m)·ẋ·v

Similarly from (E2): (α·F_max/m)·cos(φ+δ) = a_y_des + g + (β/m)·ẏ·v

**But we want drag to be naturally damping, not compensated.** For Lyapunov stability, it is better to *not* cancel drag (it helps stability). So redefine:

```
A_x = −k_px·e_x − k_dx·ẋ                                    (desired horizontal specific thrust)
A_y = g − k_py·e_y − k_dy·ẏ                                  (desired vertical specific thrust)
```

These are what we want (α·F_max/m)·sin(φ+δ) and (α·F_max/m)·cos(φ+δ) to be. The drag terms −(β/m)·ẋ·v and −(β/m)·ẏ·v then appear as additional damping in the closed-loop error dynamics, which only helps V̇ < 0.

**Step 3: Throttle and desired pitch angle.**

The required specific thrust magnitude and direction are:

```
T_des = √(A_x² + A_y²)                                       (O3)

φ_des = arctan2(A_x, A_y)                                     (O4)
```

Note: arctan2(A_x, A_y) gives the angle from the +y axis (vertical) — matching our φ convention.

The throttle command is:

```
α = clamp(m · T_des / F_max, 0, 1)                            (O5)
```

**Lyapunov verification for the outer loop** (assuming perfect tracking φ = φ_des, δ = 0):

Define V_pos = ½·k_px·e_x² + ½·ẋ² + ½·k_py·e_y² + ½·ẏ²

Then:
```
V̇_pos = k_px·e_x·ẋ + ẋ·ẍ + k_py·e_y·ẏ + ẏ·ÿ
```

With perfect tracking (thrust direction = desired direction), ẍ = −k_px·e_x − k_dx·ẋ − (β/m)·ẋ·v, so:

```
ẋ·ẍ = ẋ·(−k_px·e_x − k_dx·ẋ − (β/m)·ẋ·v)
     = −k_px·e_x·ẋ − k_dx·ẋ² − (β/m)·ẋ²·v
```

Therefore:
```
V̇_pos = k_px·e_x·ẋ − k_px·e_x·ẋ − k_dx·ẋ² − (β/m)·ẋ²·v
       + k_py·e_y·ẏ − k_py·e_y·ẏ − k_dy·ẏ² − (β/m)·ẏ²·v

       = −k_dx·ẋ² − k_dy·ẏ² − (β/m)·v·(ẋ² + ẏ²)

       = −k_dx·ẋ² − k_dy·ẏ² − (β/m)·v³
```

Since k_dx, k_dy, β > 0, we have **V̇_pos ≤ 0**, with equality only when ẋ = ẏ = 0. By LaSalle's invariance principle (applied to the set where V̇_pos = 0), convergence of the velocities to zero follows, and then ẍ = −k_px·e_x forces e_x → 0, and similarly e_y → 0.

**Result: The outer loop is Lyapunov-stable with asymptotic convergence of position and velocity errors, under the assumption of perfect attitude tracking.** ✓

---

### 3.5 Inner Loop — Attitude Stabilization

**Step 4: Attitude error.**

Define the attitude error:

```
e_φ = φ − φ_des                                               (I1)
```

We want to drive e_φ → 0 using the gimbal angle δ. The attitude dynamics are:

```
φ̈ = −(α·F_max·l_cp / J)·sin(δ)                              (from 4*)
```

**Step 5: Lyapunov function for attitude.**

```
V_att = ½·k_φ·e_φ² + ½·φ̇²                                   (I2)
```

Wait — we need to be more careful. We want to track φ_des(t), which is time-varying. Define:

```
ω_err = φ̇ − φ̇_des                                           (I3)
```

where φ̇_des = dφ_des/dt is computed from the time derivative of (O4). In practice, this can be computed numerically or approximated.

For simplicity in P1, and following standard practice, we use a simpler approach: since φ_des varies slowly compared to the attitude dynamics (by gain separation), we treat φ_des as approximately constant and set φ̇_des ≈ 0. This gives:

```
V_att = ½·k_φ·e_φ² + ½·ė_φ²                                 (I2')
```

where ė_φ = φ̇ (since φ̇_des ≈ 0).

**Step 6: Compute V̇_att.**

```
V̇_att = k_φ·e_φ·ė_φ + ė_φ·ë_φ
       = k_φ·e_φ·φ̇ + φ̇·φ̈
       = φ̇·(k_φ·e_φ + φ̈)
       = φ̇·(k_φ·e_φ − (α·F_max·l_cp/J)·sin(δ))             (I4)
```

**Step 7: Choose δ to make V̇_att < 0.**

We want the expression in parentheses to have the opposite sign of φ̇. The simplest approach: set the term in parentheses proportional to −φ̇:

```
k_φ·e_φ − (α·F_max·l_cp/J)·sin(δ) = −k_ω·φ̇                (I5)
```

where k_ω > 0 is a damping gain. Then:

```
V̇_att = −k_ω·φ̇²  ≤  0                                      (I6)
```

Solving (I5) for δ:

```
sin(δ) = (J / (α·F_max·l_cp)) · (k_φ·e_φ + k_ω·φ̇)         (I7)

δ = arcsin(clamp((J / (α·F_max·l_cp)) · (k_φ·e_φ + k_ω·φ̇), −1, 1))    (I8)
```

Then apply the gimbal saturation:

```
δ = clamp(δ, −δ_max, δ_max)                                   (I9)
```

**Singularity warning:** When α → 0, the denominator α·F_max·l_cp → 0. In implementation, enforce a minimum throttle α_min > 0 (e.g., α_min = 0.1) to avoid division by zero. Physically, with zero thrust there is no gimbal authority, so this is sensible.

---

### 3.6 Convergence Analysis for Inner Loop

From (I6), V̇_att = −k_ω·φ̇² ≤ 0 with equality only when φ̇ = 0.

Apply LaSalle's invariance principle: In the largest invariant set where φ̇ = 0, we have φ̈ = 0, which from (I5) gives k_φ·e_φ = 0, so e_φ = 0. Therefore φ → φ_des and φ̇ → 0 asymptotically.

**This proof is exact** (not approximate) under the assumptions that:
- J and l_cp are constant (P1 assumption)
- α > α_min (minimum throttle)
- The gimbal does not saturate (|δ| < δ_max)
- φ_des is treated as constant (time-scale separation)

When gimbal saturation occurs, V̇_att ≤ 0 is still guaranteed (the saturation limits the corrective torque but doesn't reverse it), but asymptotic convergence to e_φ = 0 is only guaranteed if the torque demand doesn't persistently exceed the gimbal authority.

---

## 4. Complete Control Law

### Algorithm (evaluated at each timestep):

**Inputs:** State q = [x, y, φ, ẋ, ẏ, φ̇, m], target (x_d, y_d), gains k_px, k_dx, k_py, k_dy, k_φ, k_ω, parameters F_max, l_cp, J, β, g, δ_max, α_min

**Step 1 — Desired accelerations:**
```
A_x = −k_px·(x − x_d) − k_dx·ẋ
A_y = g − k_py·(y − y_d) − k_dy·ẏ
```

**Step 2 — Throttle:**
```
T_des = √(A_x² + A_y²)
α = clamp(m·T_des / F_max,  α_min,  1)
```

**Step 3 — Desired pitch angle:**
```
φ_des = arctan2(A_x, A_y)
```

**Step 4 — Attitude error:**
```
e_φ = φ − φ_des
```

**Step 5 — Gimbal angle:**
```
sin_δ = (J / (α·F_max·l_cp)) · (k_φ·e_φ + k_ω·φ̇)
δ = arcsin(clamp(sin_δ, −1, 1))
δ = clamp(δ, −δ_max, δ_max)
```

**Outputs:** u = [α, δ]

---

## 5. Gain Conditions for Stability

### Necessary conditions:
```
k_px, k_dx, k_py, k_dy, k_φ, k_ω > 0
α_min > 0   (ensures gimbal authority)
```

### Sufficient conditions for the proofs to hold:
1. **Position gains:** k_dx, k_dy large enough relative to k_px, k_py to ensure adequate damping. A reasonable starting point: k_dx ≥ 2·√k_px and k_dy ≥ 2·√k_py (critical damping of the linearized position subsystem).

2. **Attitude gains:** k_ω large enough for fast attitude convergence. The effective natural frequency of the attitude loop is approximately √(k_φ · α·F_max·l_cp / J). For time-scale separation, this should be 5–10× the position loop bandwidth: k_φ · α_hover · F_max · l_cp / J >> k_px.

3. **Gimbal authority:** The gains k_φ, k_ω must be chosen so that the argument of arcsin in (I8) stays within [−1, 1] during normal operation. This requires:
```
(J / (α_min · F_max · l_cp)) · (k_φ · |e_φ|_max + k_ω · |φ̇|_max) ≤ 1
```
If this is violated, the gimbal saturates and convergence slows but V̇ ≤ 0 is maintained.

4. **Throttle authority:** The hover throttle α_hover = m·g/F_max must satisfy α_hover < 1 with margin for maneuvering. Typically F_max ≥ 1.5·m₀·g (thrust-to-weight ratio ≥ 1.5).

---

## 6. Summary: Exact vs. Approximate

| Aspect | Status |
|--------|--------|
| Translational EOM (Eqs. 1–3, 5) | **Exact** (no small-angle assumptions) |
| Rotational EOM (Eq. 4*, corrected) | **Exact** under constant-J assumption |
| Factor of 2 in original Eq. 4 | **Error corrected** — factor is 1 |
| CoM and J formulas (Eqs. 6–9) | **Exact** for two-body model |
| Drag formulation | **Exact** (algebraically equivalent to standard form) |
| Outer-loop Lyapunov proof (V̇_pos ≤ 0) | **Exact** assuming perfect attitude tracking |
| Inner-loop Lyapunov proof (V̇_att ≤ 0) | **Exact** under constant J, l_cp, non-zero α, no gimbal saturation |
| Time-scale separation (φ_des ≈ const) | **Approximate** — valid when attitude loop is much faster than position loop |
| Quasi-static mass assumption | **Approximate** — valid when mass changes slowly vs. controller bandwidth |
| Thrust-to-acceleration mapping at hover | **Approximate** — linearized around φ ≈ 0 for the arctan2 mapping; exact geometry is maintained in the actual EOM |

---

## 7. Notes for Implementation

1. **Angle wrapping:** Ensure e_φ = φ − φ_des is wrapped to [−π, π] to avoid discontinuities.

2. **Numerical φ̇_des:** For improved tracking, compute φ̇_des numerically (finite difference of φ_des across timesteps) and use ω_err = φ̇ − φ̇_des in the inner loop instead of φ̇. This removes the time-scale separation approximation from the inner loop.

3. **RK45 integration:** The system is 7-dimensional (state q). The control law is evaluated at each function call of the RHS passed to `solve_ivp`. Use `max_step` to limit the integrator step size for adequate control bandwidth.

4. **Saturation handling:** Implement clamp operations as `np.clip`. The control law is well-defined even under saturation — V̇ ≤ 0 is preserved.

5. **Fuel depletion guard:** Stop integration or switch to coast mode when m ≤ m_dry + ε.

6. **Consistent notation with code:** Map every symbol in this document to a Python variable name. Suggested mapping provided below.

### Suggested Python variable mapping:

| Symbol | Python variable |
|--------|----------------|
| x, y, φ | `x, y, phi` |
| ẋ, ẏ, φ̇ | `vx, vy, omega` |
| m | `mass` |
| α, δ | `alpha, delta` |
| k_px, k_dx | `k_px, k_dx` |
| k_py, k_dy | `k_py, k_dy` |
| k_φ, k_ω | `k_phi, k_omega` |
| F_max | `F_max` |
| l_cp | `l_cp` |
| J | `J_const` |
| β | `beta_drag` |
| φ_des | `phi_des` |
| e_φ | `e_phi` |
