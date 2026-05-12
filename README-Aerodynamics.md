# Rocket Aerodynamics

This section describes the aerodynamic model used in the project.

## Body-Fixed Coordinate Frame

All aerodynamic forces and moments are expressed in the **body-fixed coordinate frame**, attached to the rocket and moving with it. The frame is defined as follows:

- **Origin** — at the center of mass of the rocket.
- **$X_{BF}$ axis** — along the longitudinal axis of the rocket, pointing from the tail toward the nose.
- **$Y_{BF}$ axis** — orthogonal to $X_{B}$, lying in the plane of symmetry of the rocket.
- **$Z_{BF}$ axis** — perpendicular to the plane of motion, completing the right-handed triad.

For 2D case the pitch angle $\vartheta$ is measured from the vertical (inertial) axis to the rocket's $X_{B}$ axis, with **positive values corresponding to a rightward tilt**.

![Body-fixed coordinate frame](figures/BF_Sys.png)

The figure illustrates the body-fixed frame in the general 3D case, with the rocket inclined relative to the velocity vector $V_c$ of the center of mass. Two angles characterize this orientation:

- **$\alpha$** — angle of attack, between the longitudinal axis $X_b$ and the projection of $V_c$ onto the $X_b$–$Y_b$ plane.
- **$\beta$** — sideslip angle, between $V_c$ and the $X_b$–$Y_b$ plane. *In our planar model $\beta = 0$ and only $\alpha$ is relevant.*

The total aerodynamic force $R$ is applied at point $D$ (the center of pressure) and is decomposed along the body axes into components $X_b$, $Y_b$, $Z_b$; $R_{xz}$ denotes its projection onto the $X_b$–$Z_b$ plane. In the planar setting, the resultant lies entirely in the $X_b$–$Y_b$ plane, so the $Z_b$ component vanishes and only $X_b$ (drag) and $Y_b$ (normal force) contribute.

## Forces and Moment

In the body-fixed coordinate frame, the rocket experiences three aerodynamic quantities:

$$
\begin{cases}
X_b = -C_x(M) \cdot \dfrac{\rho V^2}{2} \cdot S_m, & \text{[N]} \\
Y_b = C_y(\alpha) \cdot \dfrac{\rho V^2}{2} \cdot S_m, & \text{[N]} \\
M_b^z = m_z(\alpha) \cdot \dfrac{\rho V^2}{2} \cdot S_m \cdot l, & \text{[N·m]}
\end{cases}
$$

- **$X_b$** — drag force, directed against the velocity vector.
- **$Y_b$** — normal (lift) force, arising at non-zero angle of attack.
- **$M_b^z$** — pitching moment about the center of mass.

All three quantities share the same structure: a dimensionless coefficient multiplied by the dynamic pressure $q_\infty = \tfrac{1}{2}\rho V^2$ and a characteristic area $S_m$ (with an additional length $l$ for the moment).

## Notation

| Symbol | Meaning | Units |
|---|---|---|
| $C_x(M)$ | drag coefficient (function of Mach number) | — |
| $C_y(\alpha)$ | normal force coefficient (function of angle of attack) | — |
| $m_z(\alpha)$ | pitching moment coefficient (function of angle of attack) | — |
| $\rho$ | air density | kg/m³ |
| $V$ | airspeed | m/s |
| $S_m$ | reference cross-sectional area | m² |
| $l$ | reference length (rocket total length) | m |
| $\alpha$ | angle of attack (between rocket axis and velocity vector) | rad / deg |

## Analytical Approximations

The dimensionless coefficients are obtained from reference tables (wind tunnel data, CFD, or empirical formulas). For the project, we use the following analytical fits, valid in two velocity regimes:

**Drag force coefficient:**

In the low-speed incompressible regime (Mach $< 0.3$) $C_x$ is independent of Mach number and angle of attack, so:

$$
C_x = 0.358
$$

![Drag coefficient $C_x$](README_figures/Cx.png)

**Normal force coefficient:**

$$
C_y(\alpha) =
\begin{cases}
0.05403\cdot\alpha, & V \in [0,\,500] \text{ m/s}, \\
0.02599\cdot\alpha + 0.008257\cdot\alpha^2, & V \in [500,\,2200] \text{ m/s},
\end{cases}
$$

![Normal force coefficient $C_y(\alpha)$](README_figures/Cy.png)

**Pitching moment coefficient:**

$$
m_z(\alpha) =
\begin{cases}
0.01840\cdot\alpha, & V \in [0,\,500] \text{ m/s}, \\
0.02113\cdot\alpha - 0.0006463\cdot\alpha^2, & V \in [500,\,2200] \text{ m/s},
\end{cases}
$$

![Pitching moment coefficient $m_z(\alpha)$](README_figures/mz.png)

where $\alpha$ is expressed in **degrees**.

The linear formulas (first branch) are valid throughout the incompressible regime $V \in [0, 500]$ m/s. At low speeds (Mach $< 0.3$, i.e., $V < 102$ m/s under standard conditions) thin-airfoil theory predicts linear dependence of lift and moment on angle of attack, so no separate formula is needed for $V < 100$ m/s — the same linear coefficients apply.

## Operating Regime

This project considers low-altitude flight at standard atmospheric conditions ($\rho = 1.225$ kg/m³) and airspeeds up to $V \leq 100$ m/s. In this regime:

- Mach number $M < 0.3$ (incompressible flow). Compressibility corrections are negligible and $C_x$ is constant.
- The pitching moment approximation is **strictly linear**: $m_z(\alpha) = C_{m\alpha}\cdot\alpha$ (with $\alpha$ in degrees).
- The proportionality coefficient

$$
C_{m\alpha} = 0.01840\ \text{deg}^{-1} \approx 1.054\ \text{rad}^{-1}
$$

is a **physical constant** of the rocket in this regime, independent of airspeed or angle of attack magnitude.

This constant $C_{m\alpha}$ appears only in the rotational dynamics — specifically in the pitching moment term $f_2 = C_{m\alpha}\,\alpha\,q_\infty S_m l / J$ that the backstepping controller cancels exactly in the Step 3 virtual control (VC2). The translational coefficients $C_x$ and $C_{y\alpha}$ affect $(x, y)$ motion only and enter the translational equations of motion as bounded disturbances.


## Frame Transformation: Body-Fixed to Inertial
 
The aerodynamic forces are computed in the body-fixed frame (along `X_b` and `Y_b` axes), but the equations of motion for the center of mass are written in the **inertial frame**. A rotation by the pitch angle $\vartheta$ converts vectors between the two frames.
 
### Geometry of the transformation
 
The pitch angle $\vartheta$ is measured from the vertical (inertial $y$) axis to the rocket's longitudinal axis $X_b$, with positive values corresponding to a rightward tilt. As a result:
 
- The body axis $X_b$ is aligned with the inertial direction $(\sin\vartheta,\ \cos\vartheta)$.
- The body axis $Y_b$ is aligned with the inertial direction $(\cos\vartheta,\ -\sin\vartheta)$.

### Rotation matrix
 
For any vector $ \mathbf{F}_{\text{BF}} = (X)^T $ expressed in the body frame, the corresponding inertial-frame components are obtained as:
 
$$
\begin{pmatrix} F_x \\ F_y \end{pmatrix}=
\underbrace{\begin{pmatrix} \sin\vartheta & \cos\vartheta \\ \cos\vartheta & -\sin\vartheta \end{pmatrix}}_{R(\vartheta)}
\begin{pmatrix} F_{X_b} \\ F_{Y_b} \end{pmatrix}
$$
 
In component form:
 
$$
\begin{aligned}
F_x &= F_{X_b}\sin\vartheta + F_{Y_b}\cos\vartheta, \\
F_y &= F_{X_b}\cos\vartheta - F_{Y_b}\sin\vartheta.
\end{aligned}
$$

The aerodynamic forces $(X_b, Y_b)$ computed in the body frame are substituted into the inertial-frame equations of motion using the rotation above. The pitching moment $M_b^z$ acts about the $Z_b$ axis and enters the rotational dynamics directly without requiring a frame rotation.



All aerodynamic forces and the regressor $Y(\alpha) = q_\infty S_m l\,\alpha$ scale proportionally with the product $S_m \cdot l$. This value must be **consistent across the simulator, the controller, and all formula evaluations**.
