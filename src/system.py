from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

# ── MPC plant state indices ───────────────────────────────────────────────────
# s = [x, y, vx, vy, theta, omega, delta]^T
IDX_X = 0
IDX_Y = 1
IDX_VX = 2
IDX_VY = 3
IDX_PHI = 4       # pitch angle theta/vartheta, positive right from vertical
IDX_OMEGA = 5     # pitch rate
IDX_DELTA = 6     # actual nozzle deflection

STATE_DIM = 7

# Backward-compatible aliases used by a few older modules/docstrings.
IDX_THETA = IDX_PHI
IDX_THETA_DOT = IDX_OMEGA
IDX_ALPHA2F = 7
IDX_TS_DOT = 8
IDX_A1_DOT = 9
IDX_C_HAT = -1


@dataclass(frozen=True)
class RocketParams:
    """Physical constants for the 2D TVC rocket.

    Mass and moment of inertia are intentionally constant, as required by the
    MPC problem statement.  The controller input is the absolute thrust F [N],
    not throttle; throttle/sigma is only used for plotting.
    """

    g: float = 9.81
    rho: float = 1.225
    mass: float = 13_000.0
    J_const: float = 318_196.0
    l_cp: float = 10.3
    l: float = 18.0
    S_mid: float = 4.5216

    F_max: float = 191_763.0
    F_min: float = 0.0
    delta_max: float = math.radians(15.0)
    delta_max_cmd: float = math.radians(15.0)
    theta_max: float = math.radians(75.0)
    tau_delta: float = 0.08

    C_x_const: float = 0.358

    @property
    def F_hover(self) -> float:
        return self.mass * self.g

    @property
    def throttle_hover(self) -> float:
        return float(np.clip(self.F_hover / max(self.F_max, 1e-9), 0.0, 1.0))

    @staticmethod
    def from_config(cfg: Dict) -> "RocketParams":
        r = cfg.get("rocket", {})
        mass = float(r.get("mass", 13_000.0))
        g = float(r.get("g", 9.81))
        F_max = float(r.get("F_max", 1.5 * mass * g))
        F_min_default = float(r.get("sigma_min", 0.20)) * F_max
        return RocketParams(
            g=g,
            rho=float(r.get("rho", 1.225)),
            mass=mass,
            J_const=float(r.get("J_const", r.get("J", 318_196.0))),
            l_cp=float(r.get("l_cp", 10.3)),
            l=float(r.get("l", 18.0)),
            S_mid=float(r.get("S_mid", r.get("S_m", 4.5216))),
            F_max=F_max,
            F_min=float(r.get("F_min", F_min_default)),
            delta_max=math.radians(float(r.get("delta_max_deg", 15.0))),
            delta_max_cmd=math.radians(float(r.get("delta_max_cmd_deg", r.get("delta_max_deg", 15.0)))),
            theta_max=math.radians(float(r.get("theta_max_deg", 75.0))),
            tau_delta=float(r.get("tau_delta", 0.08)),
            C_x_const=float(r.get("C_x", 0.358)),
        )


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def aerodynamic_coefficients(alpha_rad: float, speed: float, params: RocketParams) -> Tuple[float, float, float]:
    """Return (C_x, C_y, m_z).

    The supplied polynomial fits use alpha in degrees.  The state model keeps all
    angles in radians, so alpha is converted before evaluating C_y(alpha) and
    m_z(alpha).
    """

    # The polynomial fits are intended for a practical aerodynamic range.
    # Clipping prevents numerical explosions during optimizer trial rollouts.
    alpha_deg = float(np.clip(math.degrees(alpha_rad), -35.0, 35.0))
    C_x = params.C_x_const
    if speed < 500.0:
        C_y = 0.05403 * alpha_deg
        m_z = 0.01840 * alpha_deg
    else:
        C_y = 0.02599 * alpha_deg + 0.008257 * alpha_deg**2
        m_z = 0.02113 * alpha_deg - 0.0006463 * alpha_deg**2
    return C_x, C_y, m_z


def aero_forces_and_moment(state: np.ndarray, params: RocketParams) -> Dict[str, float]:
    """Aerodynamics in the body frame and the pitching moment about CoM."""

    vx = float(state[IDX_VX])
    vy = float(state[IDX_VY])
    theta = float(state[IDX_PHI])
    speed = math.hypot(vx, vy)
    if not math.isfinite(speed):
        speed = 1.0e6
    # The aerodynamic fit is intended for the modeled flight envelope; cap the
    # dynamic pressure used inside optimizer trial rollouts to keep the NLP
    # numerically well-conditioned.
    speed_q = min(speed, 600.0)
    q_inf = 0.5 * params.rho * speed_q**2

    if speed < 1e-6:
        alpha = 0.0
    else:
        # Velocity angle is measured from the inertial vertical axis.
        # Positive alpha means the relative wind is to the right of the body axis.
        # This sign makes the supplied positive m_z(alpha) fit act as a restoring
        # aerodynamic moment for a positive pitch angle at near-vertical flight.
        alpha = wrap_angle(math.atan2(vx, vy) - theta)

    C_x, C_y, m_z = aerodynamic_coefficients(alpha, speed, params)
    X_b = -C_x * q_inf * params.S_mid
    Y_b = C_y * q_inf * params.S_mid
    M_b = m_z * q_inf * params.S_mid * params.l
    # Numerical safety for optimizer trial trajectories far outside the design
    # envelope.  The nominal project operates well below these limits.
    force_cap = 4.0 * params.F_max
    moment_cap = 4.0 * params.F_max * params.l
    X_b = float(np.clip(X_b, -force_cap, force_cap))
    Y_b = float(np.clip(Y_b, -force_cap, force_cap))
    M_b = float(np.clip(M_b, -moment_cap, moment_cap))
    return {
        "V": speed,
        "alpha": alpha,
        "q_inf": q_inf,
        "C_x": C_x,
        "C_y": C_y,
        "m_z": m_z,
        "X_b": X_b,
        "Y_b": Y_b,
        "M_b": M_b,
    }


def rocket_dynamics(state: np.ndarray, control: np.ndarray, params: RocketParams) -> np.ndarray:
    """Continuous-time nonlinear dynamics for MPC.

    control = [delta_cmd, F], where F is thrust in newtons.
    """

    x, y, vx, vy, theta, omega, delta = [float(v) for v in state]
    delta_cmd = float(control[0])
    F = float(control[1])

    aero = aero_forces_and_moment(state, params)
    X_b = aero["X_b"]
    Y_b = aero["Y_b"]
    M_b = aero["M_b"]

    sin_t = math.sin(theta)
    cos_t = math.cos(theta)

    # Small-nozzle-deflection translational model from the MPC statement.
    x_ddot = (F * sin_t + F * delta * cos_t + X_b * sin_t + Y_b * cos_t) / params.mass
    y_ddot = (F * cos_t - params.mass * params.g - F * delta * sin_t + X_b * cos_t - Y_b * sin_t) / params.mass

    # Rotational model: J*theta_ddot = F*l_cp*delta + M_b.
    theta_ddot = (F * params.l_cp * delta + M_b) / params.J_const

    delta_dot = (-delta + delta_cmd) / max(params.tau_delta, 1e-6)

    return np.array([vx, vy, x_ddot, y_ddot, omega, theta_ddot, delta_dot], dtype=float)


def rk4_step(state: np.ndarray, control: np.ndarray, dt: float, params: RocketParams) -> np.ndarray:
    """One fixed-step integration step.

    The nozzle actuator time constant can be much smaller than the MPC sampling
    time.  Integrating ``tau_delta*delta_dot = -delta + delta_cmd`` with a
    coarse RK4 step is numerically fragile.  We therefore update the actuator
    exactly for a zero-order-held command and integrate the rigid-body states
    using the average nozzle angle during the interval.
    """

    s = np.asarray(state, dtype=float).copy()
    u = np.asarray(control, dtype=float).copy()
    delta0 = float(np.clip(s[IDX_DELTA], -params.delta_max, params.delta_max))
    delta_cmd = float(np.clip(u[0], -params.delta_max_cmd, params.delta_max_cmd))
    tau = max(params.tau_delta, 1.0e-6)
    a = math.exp(-dt / tau)
    delta_next = delta_cmd + (delta0 - delta_cmd) * a
    # Exact average of delta(t) over the sample.
    delta_eff = delta_cmd + (delta0 - delta_cmd) * (1.0 - a) * tau / max(dt, 1.0e-9)
    delta_eff = float(np.clip(delta_eff, -params.delta_max, params.delta_max))

    s_eff = s.copy()
    s_eff[IDX_DELTA] = delta_eff
    u_eff = np.array([delta_eff, float(u[1])], dtype=float)

    k1 = rocket_dynamics(s_eff, u_eff, params)
    k1[IDX_DELTA] = 0.0
    k2_state = s_eff + 0.5 * dt * k1
    k2_state[IDX_DELTA] = delta_eff
    k2 = rocket_dynamics(k2_state, u_eff, params)
    k2[IDX_DELTA] = 0.0
    k3_state = s_eff + 0.5 * dt * k2
    k3_state[IDX_DELTA] = delta_eff
    k3 = rocket_dynamics(k3_state, u_eff, params)
    k3[IDX_DELTA] = 0.0
    k4_state = s_eff + dt * k3
    k4_state[IDX_DELTA] = delta_eff
    k4 = rocket_dynamics(k4_state, u_eff, params)
    k4[IDX_DELTA] = 0.0

    out = s_eff + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    out[IDX_PHI] = wrap_angle(out[IDX_PHI])
    out[IDX_DELTA] = float(np.clip(delta_next, -params.delta_max, params.delta_max))
    return out


# Compatibility wrapper: old solve_ivp-based code called rocket_rhs(t, state,
# controller, params).  The MPC simulator does not use this, but keeping a small
# wrapper makes exploratory notebooks less fragile.
def rocket_rhs(_t: float, state: np.ndarray, controller, params: RocketParams) -> np.ndarray:
    control, _cache = controller.compute_control(state, params)
    return rocket_dynamics(state, np.array([control["delta_cmd"], control["F"]]), params)
