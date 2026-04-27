from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from rocket_dynamics import (
    IDX_THETA, IDX_VX, IDX_VY, IDX_THETA_DOT, IDX_C_HAT,
    RocketParams,
)


@dataclass
class AdaptiveCEController:
    """Lyapunov-based attitude controller with Certainty Equivalence
    adaptation of the pitching moment coefficient C_m_alpha.

    The estimate `c_hat` lives in the simulator state vector (index IDX_C_HAT)
    so it integrates synchronously with the physical states. The controller
    only computes its derivative `c_hat_dot` from the adaptation law.
    """

    # Lyapunov gains (attitude loop)
    k_theta: float
    k_omega: float

    # Adaptation gain
    gamma: float

    # Target attitude
    theta_target: float = 41.0

    # Projection bounds for c_hat (physically meaningful range)
    c_hat_min: float = 0.1
    c_hat_max: float = 5.0

    # Latest auxiliary values (for logging / debugging)
    last_cache: dict = None

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, float, dict]:
        theta = state[IDX_THETA]
        vx = state[IDX_VX]
        vy = state[IDX_VY]
        theta_dot = state[IDX_THETA_DOT]
        c_hat = state[IDX_C_HAT]

        # --- Tracking error ---
        e_theta = theta - self.theta_target

        # --- Aerodynamic regressor ---
        V = math.hypot(vx, vy)
        q_inf = 0.5 * params.rho * V * V
        alpha = theta - math.atan2(vx, vy) if V > 1e-6 else 0.0
        Y_reg = q_inf * params.S_mid * params.l * alpha   # regressor

        # --- Control law (Certainty Equivalence) ---
        # sin(delta) = J/(m*g*l_cp) * ( k_theta*e + k_omega*theta_dot + Y/J * c_hat )
        sin_delta = (params.J_const / (params.mass * params.g * params.l_cp)) * (
            self.k_theta * e_theta
            + self.k_omega * theta_dot
            + Y_reg * c_hat / params.J_const
        )
        sin_delta = float(np.clip(sin_delta, -1.0, 1.0))
        delta = math.asin(sin_delta)

        # Saturate to actuator limits.
        delta = float(np.clip(delta, -params.delta_max, params.delta_max))

        # --- Adaptation law ---
        # c_hat_dot = gamma * Y * theta_dot / J
        c_hat_dot_unprojected = self.gamma * Y_reg * theta_dot / params.J_const

        # Projection: freeze adaptation if it would push c_hat past bounds.
        if c_hat >= self.c_hat_max and c_hat_dot_unprojected > 0:
            c_hat_dot = 0.0
        elif c_hat <= self.c_hat_min and c_hat_dot_unprojected < 0:
            c_hat_dot = 0.0
        else:
            c_hat_dot = c_hat_dot_unprojected

        # --- Throttle: fixed hover ---
        throttle = params.throttle_hover

        cache = {
            "alpha": alpha,
            "Y_reg": Y_reg,
            "e_theta": e_theta,
            "c_hat": c_hat,
            "c_hat_dot": c_hat_dot,
        }

        return throttle, delta, c_hat_dot, cache
