from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple
import math
import numpy as np

from .system import RocketParams, wrap_angle


@dataclass
class LyapunovController:
    k_px: float
    k_dx: float
    k_py: float
    k_dy: float
    k_phi: float
    k_omega: float
    phi_des_limit: float
    last_cache: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_config(cfg: Dict) -> "LyapunovController":
        ctl = cfg['controller']
        return LyapunovController(
            k_px=float(ctl['k_px']),
            k_dx=float(ctl['k_dx']),
            k_py=float(ctl['k_py']),
            k_dy=float(ctl['k_dy']),
            k_phi=float(ctl['k_phi']),
            k_omega=float(ctl['k_omega']),
            phi_des_limit=math.radians(float(ctl['phi_des_limit_deg'])),
        )

    def compute_control(
        self,
        state: np.ndarray,
        target: Dict[str, float],
        params: RocketParams,
    ) -> Tuple[float, float, Dict[str, float]]:
        x, y, phi, vx, vy, omega, mass = [float(v) for v in state]
        x_d = float(target['x_d'])
        y_d = float(target['y_d'])

        e_x = x - x_d
        e_y = y - y_d

        A_x = -self.k_px * e_x - self.k_dx * vx
        A_y = params.g - self.k_py * e_y - self.k_dy * vy
        T_des = math.hypot(A_x, A_y)

        alpha = np.clip((mass * T_des) / max(params.F_max, 1e-9), params.alpha_min, 1.0)

        phi_des = math.atan2(A_x, A_y)
        phi_des = float(np.clip(phi_des, -self.phi_des_limit, self.phi_des_limit))
        e_phi = wrap_angle(phi - phi_des)

        authority = max(alpha * params.F_max * params.l_cp, 1e-8)
        sin_delta = (params.J_const / authority) * (self.k_phi * e_phi + self.k_omega * omega)
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))
        delta = math.asin(sin_delta)
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        cache = {
            'alpha': alpha,
            'delta': delta,
            'phi_des': phi_des,
            'e_x': e_x,
            'e_y': e_y,
            'e_phi': e_phi,
            'A_x': A_x,
            'A_y': A_y,
            'T_des': T_des,
        }
        return alpha, delta, cache
