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
    alpha_hover: float

    @staticmethod
    def from_config(cfg: Dict) -> "RocketParams":
        rocket_cfg = cfg['rocket']
        mass = float(rocket_cfg['mass'])
        F_max = float(rocket_cfg['F_max'])
        alpha_hover = float(np.clip((mass * float(rocket_cfg['g'])) / max(F_max, 1e-9), 0.0, 1.0))
        return RocketParams(
            g=float(rocket_cfg['g']),
            mass=mass,
            J_const=float(rocket_cfg['J_const']),
            l_cp=float(rocket_cfg['l_cp']),
            F_max=F_max,
            delta_max=math.radians(float(rocket_cfg['delta_max_deg'])),
            alpha_hover=alpha_hover,
        )


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rocket_rhs(
    _t: float,
    state: np.ndarray,
    controller,
    params: RocketParams,
) -> np.ndarray:
    x, y, phi, vx, vy, omega = state
    alpha, delta, controller_cache = controller.compute_control(state, params)

    thrust = alpha * params.F_max
    x_ddot = (thrust / params.mass) * math.sin(phi + delta)
    y_ddot = (thrust / params.mass) * math.cos(phi + delta) - params.g
    phi_ddot = -(thrust * params.l_cp / params.J_const) * math.sin(delta)

    controller.last_cache = controller_cache
    return np.array([vx, vy, omega, x_ddot, y_ddot, phi_ddot], dtype=float)