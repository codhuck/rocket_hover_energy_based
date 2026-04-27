from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

from .system import (
    IDX_THETA, IDX_VX, IDX_VY, IDX_THETA_DOT, IDX_C_HAT,
    RocketParams, wrap_angle,
)


# Common return type: (throttle, delta, c_hat_dot, cache).
# Non-adaptive controllers return c_hat_dot = 0.0 so that the augmented
# state simply carries c_hat as a constant through the integrator.


@dataclass
class AttitudeLyapunovController:
    """Project 1 baseline: pure Lyapunov attitude controller.

    Does NOT compensate for aerodynamic moment, does NOT adapt parameters.
    Used for direct comparison against the adaptive controller in Project 2.
    """
    k_theta: float
    k_omega: float
    theta_target: float = 0.0
    last_cache: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_config(cfg: Dict) -> "AttitudeLyapunovController":
        ctl = cfg["controller"]
        return AttitudeLyapunovController(
            k_theta=float(ctl["k_theta"]),
            k_omega=float(ctl["k_omega"]),
            theta_target=math.radians(float(ctl.get("theta_target_deg", 0.0))),
        )

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, float, Dict[str, float]]:
        theta = float(state[IDX_THETA])
        theta_dot = float(state[IDX_THETA_DOT])

        throttle = params.throttle_hover
        e_theta = wrap_angle(theta - self.theta_target)

        authority = max(throttle * params.F_max * params.l_cp, 1e-8)
        sin_delta = (params.J_const / authority) * (
            self.k_theta * e_theta + self.k_omega * theta_dot
        )
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))
        delta = math.asin(sin_delta)
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        cache = {
            "throttle": throttle,
            "delta": delta,
            "theta_target": self.theta_target,
            "e_theta": e_theta,
        }
        return throttle, delta, 0.0, cache


@dataclass
class AdaptiveCEController:
    """Project 2: Lyapunov-based attitude controller with Certainty Equivalence
    adaptation of the pitching moment coefficient C_m_alpha.

    The estimate `c_hat` lives in the simulator state (index IDX_C_HAT).
    The controller reads it from the state and returns its derivative
    `c_hat_dot` from the adaptation law (with projection bounds).
    """
    k_theta: float
    k_omega: float
    gamma: float
    theta_target: float = 0.0
    c_hat_min: float = 0.1
    c_hat_max: float = 5.0
    last_cache: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_config(cfg: Dict) -> "AdaptiveCEController":
        ctl = cfg["controller"]
        return AdaptiveCEController(
            k_theta=float(ctl["k_theta"]),
            k_omega=float(ctl["k_omega"]),
            gamma=float(ctl["gamma"]),
            theta_target=math.radians(float(ctl.get("theta_target_deg", 0.0))),
            c_hat_min=float(ctl.get("c_hat_min", 0.1)),
            c_hat_max=float(ctl.get("c_hat_max", 5.0)),
        )

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, float, Dict[str, float]]:
        theta = float(state[IDX_THETA])
        vx = float(state[IDX_VX])
        vy = float(state[IDX_VY])
        theta_dot = float(state[IDX_THETA_DOT])
        c_hat = float(state[IDX_C_HAT])

        throttle = params.throttle_hover
        e_theta = wrap_angle(theta - self.theta_target)

        # Aerodynamic regressor Y = q_inf * S_mid * l * alpha
        V = math.hypot(vx, vy)
        q_inf = 0.5 * params.rho * V * V
        alpha = theta - math.atan2(vx, vy) if V > 1e-6 else 0.0
        Y_reg = q_inf * params.S_mid * params.l * alpha

        # Control law (Certainty Equivalence): Lyapunov + Y/J * c_hat term
        authority = max(throttle * params.F_max * params.l_cp, 1e-8)
        sin_delta = (params.J_const / authority) * (
            self.k_theta * e_theta
            + self.k_omega * theta_dot
            + Y_reg * c_hat / params.J_const
        )
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))
        delta = math.asin(sin_delta)
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        # Adaptation law: c_hat_dot = gamma * Y * theta_dot / J
        c_hat_dot_unprojected = self.gamma * Y_reg * theta_dot / params.J_const

        # Projection: freeze adaptation if it would push c_hat past bounds.
        if c_hat >= self.c_hat_max and c_hat_dot_unprojected > 0:
            c_hat_dot = 0.0
        elif c_hat <= self.c_hat_min and c_hat_dot_unprojected < 0:
            c_hat_dot = 0.0
        else:
            c_hat_dot = c_hat_dot_unprojected

        cache = {
            "throttle": throttle,
            "delta": delta,
            "theta_target": self.theta_target,
            "e_theta": e_theta,
            "alpha_aero": alpha,
            "Y_reg": Y_reg,
            "c_hat": c_hat,
            "c_hat_dot": c_hat_dot,
        }
        return throttle, delta, c_hat_dot, cache
