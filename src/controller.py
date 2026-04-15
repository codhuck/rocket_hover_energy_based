from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple
import math
import numpy as np

from .system import RocketParams, wrap_angle


@dataclass
class AttitudeLyapunovController:
    k_phi: float
    k_omega: float
    phi_target: float = 0.0
    last_cache: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_config(cfg: Dict) -> "AttitudeLyapunovController":
        ctl = cfg['controller']
        return AttitudeLyapunovController(
            k_phi=float(ctl['k_phi']),
            k_omega=float(ctl['k_omega']),
            phi_target=math.radians(float(ctl.get('phi_target_deg', 0.0))),
        )

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, Dict[str, float]]:
        phi = float(state[2])
        omega = float(state[5])

        alpha = params.alpha_hover
        e_phi = wrap_angle(phi - self.phi_target)

        authority = max(alpha * params.F_max * params.l_cp, 1e-8)
        sin_delta = (params.J_const / authority) * (self.k_phi * e_phi + self.k_omega * omega)
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))
        delta = math.asin(sin_delta)
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        cache = {
            'alpha': alpha,
            'delta': delta,
            'phi_target': self.phi_target,
            'e_phi': e_phi,
        }
        return alpha, delta, cache


@dataclass
class CrossTermLyapunovController:
    k_phi: float
    k_omega: float
    k_c: float
    c: float
    phi_target: float = 0.0
    eps: float = 1e-6
    last_cache: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_config(cfg: Dict) -> "CrossTermLyapunovController":
        ctl = cfg['controller_crossterm']
        return CrossTermLyapunovController(
            k_phi=float(ctl['k_phi']),
            k_omega=float(ctl['k_omega']),
            k_c=float(ctl['k_c']),
            c=float(ctl['c']),
            phi_target=math.radians(float(ctl.get('phi_target_deg', 0.0))),
        )

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, Dict[str, float]]:
        phi = float(state[2])
        omega = float(state[5])

        alpha = params.alpha_hover
        e_phi = wrap_angle(phi - self.phi_target)

        authority = max(alpha * params.F_max * params.l_cp, 1e-8)
        pd_term = self.k_phi * e_phi + self.k_omega * omega

        numerator = self.k_phi * e_phi * omega + (self.c + self.k_omega) * (omega ** 2) + self.k_c * (e_phi ** 2)
        denominator = omega + self.c * e_phi

        fallback_to_pd = abs(denominator) < self.eps
        if fallback_to_pd:
            sin_delta = (params.J_const / authority) * pd_term
        else:
            sin_delta = (params.J_const / authority) * (numerator / denominator)
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))

        delta = math.asin(sin_delta)
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        cache = {
            'alpha': alpha,
            'delta': delta,
            'phi_target': self.phi_target,
            'e_phi': e_phi,
            'denominator': float(denominator),
            'fallback_to_pd': float(fallback_to_pd),
        }
        return alpha, delta, cache