from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple
import math
import numpy as np


@dataclass(frozen=True)
class RocketParams:
    g: float
    beta_drag: float
    m_dry: float
    m_dot_max: float
    v_e: float
    h_dry: float
    h_tank: float
    L_tank: float
    J_dry_cm: float
    delta_max: float
    alpha_min: float
    l_cp: float
    J_const: float
    F_max: float

    @staticmethod
    def from_config(cfg: Dict, initial_mass: float) -> "RocketParams":
        rocket_cfg = cfg['rocket']
        F_max = rocket_cfg['m_dot_max'] * rocket_cfg['v_e']
        m_mid = 0.5 * (initial_mass + rocket_cfg['m_dry'])
        m_f_mid = max(m_mid - rocket_cfg['m_dry'], 0.0)
        h_com = (
            rocket_cfg['m_dry'] * rocket_cfg['h_dry']
            + m_f_mid * rocket_cfg['h_tank']
        ) / max(m_mid, 1e-9)
        J_const = (
            rocket_cfg['J_dry_cm']
            + rocket_cfg['m_dry'] * (h_com - rocket_cfg['h_dry']) ** 2
            + (1.0 / 12.0) * m_f_mid * rocket_cfg['L_tank'] ** 2
            + m_f_mid * (h_com - rocket_cfg['h_tank']) ** 2
        )
        return RocketParams(
            g=float(rocket_cfg['g']),
            beta_drag=float(rocket_cfg['beta_drag']),
            m_dry=float(rocket_cfg['m_dry']),
            m_dot_max=float(rocket_cfg['m_dot_max']),
            v_e=float(rocket_cfg['v_e']),
            h_dry=float(rocket_cfg['h_dry']),
            h_tank=float(rocket_cfg['h_tank']),
            L_tank=float(rocket_cfg['L_tank']),
            J_dry_cm=float(rocket_cfg['J_dry_cm']),
            delta_max=math.radians(float(rocket_cfg['delta_max_deg'])),
            alpha_min=float(rocket_cfg['alpha_min']),
            l_cp=float(h_com),
            J_const=float(J_const),
            F_max=float(F_max),
        )


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rocket_rhs(
    _t: float,
    state: np.ndarray,
    controller,
    params: RocketParams,
    target: Dict[str, float],
) -> np.ndarray:
    x, y, phi, vx, vy, omega, mass = state
    alpha, delta, controller_cache = controller.compute_control(state, target, params)

    # Fuel depletion guard: no more burn below dry mass.
    if mass <= params.m_dry + 1e-6:
        alpha = 0.0
        delta = 0.0
        mass = params.m_dry

    thrust = alpha * params.F_max
    speed = math.hypot(vx, vy)

    x_ddot = (thrust / mass) * math.sin(phi + delta) - (params.beta_drag / mass) * vx * speed
    y_ddot = (thrust / mass) * math.cos(phi + delta) - params.g - (params.beta_drag / mass) * vy * speed
    phi_ddot = -(thrust * params.l_cp / params.J_const) * math.sin(delta)
    m_dot = -alpha * params.m_dot_max if mass > params.m_dry else 0.0

    controller.last_cache = controller_cache
    return np.array([vx, vy, omega, x_ddot, y_ddot, phi_ddot, m_dot], dtype=float)
