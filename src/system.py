from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np


# Index helpers — keeps state-vector layout in one place.
IDX_X = 0
IDX_Y = 1
IDX_THETA = 2
IDX_VX = 3
IDX_VY = 4
IDX_THETA_DOT = 5
IDX_C_HAT = 6           # online estimate of C_m_alpha
STATE_DIM = 7


@dataclass(frozen=True)
class RocketParams:
    # --- Physics ---
    g: float
    rho: float

    # --- Geometry / mass ---
    mass: float
    J_const: float
    l_cp: float
    l: float
    S_mid: float

    # --- Actuator ---
    F_max: float
    delta_max: float
    throttle_hover: float

    # --- Aerodynamics ---
    C_x: float
    C_yn: float

    # The TRUE pitching moment coefficient used by the simulator only.
    # The controller works with the estimate carried in the state vector.
    C_m_alpha_true: float

    def C_y(self, alpha: float) -> float:
        return self.C_yn * alpha

    @staticmethod
    def from_config(cfg: Dict) -> "RocketParams":
        rocket_cfg = cfg["rocket"]
        mass = float(rocket_cfg["mass"])
        F_max = float(rocket_cfg["F_max"])
        g = float(rocket_cfg["g"])
        throttle_hover = float(
            np.clip((mass * g) / max(F_max, 1e-9), 0.0, 1.0)
        )
        return RocketParams(
            g=g,
            rho=float(rocket_cfg.get("rho", 1.225)),
            mass=mass,
            J_const=float(rocket_cfg["J_const"]),
            l_cp=float(rocket_cfg["l_cp"]),
            l=float(rocket_cfg["l"]),
            S_mid=float(rocket_cfg["S_mid"]),
            F_max=F_max,
            delta_max=math.radians(float(rocket_cfg["delta_max_deg"])),
            throttle_hover=throttle_hover,
            C_x=float(rocket_cfg["C_x"]),
            C_yn=float(rocket_cfg["C_yn"]),
            C_m_alpha_true=float(rocket_cfg["C_m_alpha_true"]),
        )


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rocket_rhs(
    _t: float,
    state: np.ndarray,
    controller,
    params: RocketParams,
) -> np.ndarray:
    """Right-hand side of the augmented system:
        physical state (6) + parameter estimate (1) = 7 components.

    Controller is expected to return:
        (throttle, delta, c_hat_dot, cache)
    where c_hat_dot is the time-derivative of the estimate produced by the
    adaptation law (with projection if applicable).
    """
    x, y, theta, vx, vy, theta_dot, c_hat = state

    # --- Controller (reads c_hat from state, returns its derivative) ---
    throttle, delta, c_hat_dot, controller_cache = controller.compute_control(
        state, params
    )
    thrust = throttle * params.F_max

    # --- Aerodynamic auxiliary quantities ---
    V = math.hypot(vx, vy)
    q_inf = 0.5 * params.rho * V * V

    if V > 1e-6:
        alpha = theta - math.atan2(vx, vy)
    else:
        alpha = 0.0

    # --- Aerodynamic forces (body frame) and moment (TRUE C_m_alpha) ---
    X_b = -params.C_x * q_inf * params.S_mid
    Y_b = params.C_y(alpha) * q_inf * params.S_mid
    M_b_z = params.C_m_alpha_true * alpha * q_inf * params.S_mid * params.l

    # --- Body-to-inertial rotation (theta from vertical) ---
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    F_aero_x = X_b * sin_t + Y_b * cos_t
    F_aero_y = X_b * cos_t - Y_b * sin_t

    # --- Translational dynamics ---
    x_ddot = (thrust * math.sin(theta + delta) + F_aero_x) / params.mass
    y_ddot = (thrust * math.cos(theta + delta) + F_aero_y) / params.mass - params.g

    # --- Rotational dynamics ---
    theta_ddot = (
        -thrust * params.l_cp * math.sin(delta) + M_b_z
    ) / params.J_const

    controller.last_cache = controller_cache

    return np.array(
        [vx, vy, theta_dot, x_ddot, y_ddot, theta_ddot, c_hat_dot],
        dtype=float,
    )
