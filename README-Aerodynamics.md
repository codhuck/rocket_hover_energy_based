# Rocket Aerodynamics

This section describes the aerodynamic model used in the project.

## Body-Fixed Coordinate Frame

All aerodynamic forces and moments are expressed in the **body-fixed coordinate frame**, attached to the rocket and moving with it. The frame is defined as follows:

- **Origin** — at the center of mass of the rocket.
- **`X_b` axis** — along the longitudinal axis of the rocket, pointing from the tail toward the nose.
- **`Y_b` axis** — orthogonal to `X_b`, lying in the plane of symmetry of the rocket.
- **`Z_b` axis** — perpendicular to the plane of motion, completing the right-handed triad. Since the model is planar, all rotation occurs about this axis: the pitch angle `φ` and the pitching moment `M_b^z` are both measured relative to `Z_b`.

The pitch angle `φ` is measured from the vertical (inertial) axis to the rocket's `X_b` axis, with **positive values corresponding to a rightward tilt** (consistent with Project 1).

![Body-fixed coordinate frame](docs/body_frame.png)

> 📌 *Replace this placeholder with the actual diagram of the body-fixed frame.*

## Forces and Moment

In the body-fixed coordinate frame, the rocket experiences three aerodynamic quantities:

$$
\begin{cases}
X_b = -C_x(M) \cdot \dfrac{\rho V^2}{2} \cdot S_m, & \text{[N]} \\
Y_b = C_y(\alpha) \cdot \dfrac{\rho V^2}{2} \cdot S_m, & \text{[N]} \\
M_b^z = m_z(\alpha) \cdot \dfrac{\rho V^2}{2} \cdot S_m \cdot l, & \text{[N·m]}
\end{cases}
$$

- **`X_b`** — drag force, directed against the velocity vector.
- **`Y_b`** — normal (lift) force, arising at non-zero angle of attack.
- **`M_b^z`** — pitching moment about the center of mass.

All three quantities share the same structure: a dimensionless coefficient multiplied by the dynamic pressure $q_\infty = \tfrac{1}{2}\rho V^2$ and a characteristic area $S_m$ (with an additional length $l$ for the moment).

## Notation

| Symbol | Meaning | Units |
|---|---|---|
| `C_x(M)` | drag coefficient (function of Mach number) | — |
| `C_y(α)` | normal force coefficient (function of angle of attack) | — |
| `m_z(α)` | pitching moment coefficient (function of angle of attack) | — |
| `ρ` | air density | kg/m³ |
| `V` | airspeed | m/s |
| `S_m` | reference cross-sectional area | m² |
| `l` | reference length (rocket total length) | m |
| `α` | angle of attack (between rocket axis and velocity vector) | rad / deg |

## Analytical Approximations

The dimensionless coefficients are obtained from reference tables (wind tunnel data, CFD, or empirical formulas). For the project, we use the following analytical fits, valid in two velocity regimes:

**Normal force coefficient:**

$$
C_y(\alpha) =
\begin{cases}
0.05403\,\alpha, & V \in [100, 500] \text{ m/s}, \\
0.02599\,\alpha + 0.008257\,\alpha^2, & V \in [500, 2200] \text{ m/s},
\end{cases}
$$

**Pitching moment coefficient:**

$$
m_z(\alpha) =
\begin{cases}
0.01840\,\alpha, & V \in [100, 500] \text{ m/s}, \\
0.02113\,\alpha - 0.0006463\,\alpha^2, & V \in [500, 2200] \text{ m/s},
\end{cases}
$$

where `α` is expressed in **degrees**.

## Operating Regime

This project considers low-altitude flight at standard atmospheric conditions (`ρ = 1.225 kg/m³`) and airspeeds up to `V ≤ 100 m/s`. In this regime:

- Mach number `M < 0.3` (incompressible flow), and `C_x` is approximately constant: `C_x ≈ 0.30 … 0.38`.
- The pitching moment approximation is **strictly linear**: `m_z(α) = 0.01840 · α`.
- The proportionality coefficient

$$
C_{m\alpha} = 0.01840 \text{ deg}^{-1} \approx 1.054 \text{ rad}^{-1}
$$

is a **physical constant** of the rocket in this regime, independent of the flight state.

This constant `C_{m\alpha}` is the central quantity that the adaptive controller estimates online via Certainty Equivalence.
