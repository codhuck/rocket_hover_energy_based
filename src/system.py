from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import math
import numpy as np


@dataclass(frozen=True)
class RocketParams:
    g: float
    mass: float
    J_const: float
    l_cp: float
    F_max: float
    delta_max: float
    throttle: float
    C_x: float
    C_yn: float
    
    def C_x(M):
        return 0.358
    def C_y(alpha):
        return C_yn*alpha

    
    @staticmethod
    def from_config(cfg: Dict) -> "RocketParams":
        rocket_cfg = cfg['rocket']
        mass = float(rocket_cfg['mass'])
        F_max = float(rocket_cfg['F_max'])
        throttle_hover = float(np.clip((mass * float(rocket_cfg['g'])) / max(F_max, 1e-9), 0.0, 1.0))
        return RocketParams(
            g=float(rocket_cfg['g']),
            mass=mass,
            J_const=float(rocket_cfg['J_const']),
            l_cp=float(rocket_cfg['l_cp']),
            l = float(rocket_cfg["l"])
            S_mid = float(rocket_cfg["S_mid"])
            F_max=F_max,
            delta_max=math.radians(float(rocket_cfg['delta_max_deg'])),
            throttle_hover=throttle_hover,
        )


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rocket_rhs(
    _t: float,
    state: np.ndarray,
    controller,
    params: RocketParams,
) -> np.ndarray:
    x, y, theta, vx, vy, theta_dot = state

    throttle, delta, controller_cache = controller.compute_control(state, params)
    thrust = throttle * params.F_max

    V = math.hypot(vx, vy)
    q_inf = 0.5 * params.rho * V * V

    if V > 1e-6:
        alpha = theta - math.atan2(vx, vy)
    else:
        alpha = 0.0  # undefined at zero airspeed

    X_b = -params.C_x * q_inf * params.S_m
    Y_b = params.C_y(alpha) * q_inf * params.S_m

    M_b_z = params.C_m_alpha * alpha * q_inf * params.S_m * params.l

    sin_t, cos_t = math.sin(theta), math.cos(theta)
    F_aero_x = X_b * sin_t + Y_b * cos_t
    F_aero_y = X_b * cos_t - Y_b * sin_t

    x_ddot = (thrust * math.sin(theta + delta) + F_aero_x) / params.mass
    y_ddot = (thrust * math.cos(theta + delta) + F_aero_y) / params.mass - params.g

    theta_ddot = (
        -thrust * params.l_cp * math.sin(delta) + M_b_z
    ) / params.J_const

    controller.last_cache = controller_cache
    return np.array([vx, vy, theta_dot, x_ddot, y_ddot, theta_ddot], dtype=float)
