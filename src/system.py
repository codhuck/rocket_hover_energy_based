from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np

# ── State indices ─────────────────────────────────────────────────────────────
IDX_X        = 0
IDX_Y        = 1
IDX_VX       = 2
IDX_VY       = 3
IDX_PHI      = 4   # pitch angle  ϑ
IDX_OMEGA    = 5   # pitch rate   ϑ̇
IDX_DELTA    = 6   # nozzle angle δ  (first-order actuator state)
IDX_ALPHA2F  = 7   # command-filter state α2f
IDX_TS_DOT   = 8   # filtered derivative of theta_star
IDX_A1_DOT   = 9   # filtered derivative of alpha1

STATE_DIM = 10

# Backward-compat aliases
IDX_THETA     = IDX_PHI
IDX_THETA_DOT = IDX_OMEGA
IDX_C_HAT     = -1


@dataclass(frozen=True)
class RocketParams:
    g: float
    rho: float
    mass: float
    J_const: float
    l_cp: float
    l: float
    S_mid: float

    F_max: float
    delta_max: float
    delta_max_cmd: float
    sigma_min: float
    tau_delta: float

    C_x: float
    C_yn: float
    C_m_alpha: float

    throttle_hover: float

    @staticmethod
    def from_config(cfg: Dict) -> "RocketParams":
        r = cfg["rocket"]
        mass  = float(r["mass"])
        F_max = float(r["F_max"])
        g     = float(r["g"])
        throttle_hover = float(np.clip(mass * g / max(F_max, 1e-9), 0.0, 1.0))
        sigma_min = float(r.get("sigma_min", throttle_hover * 0.5))
        return RocketParams(
            g=g,
            rho=float(r.get("rho", 1.225)),
            mass=mass,
            J_const=float(r["J_const"]),
            l_cp=float(r["l_cp"]),
            l=float(r["l"]),
            S_mid=float(r["S_mid"]),
            F_max=F_max,
            delta_max=math.radians(float(r["delta_max_deg"])),
            delta_max_cmd=math.radians(float(r.get("delta_max_cmd_deg", r["delta_max_deg"]))),
            sigma_min=sigma_min,
            tau_delta=float(r.get("tau_delta", 0.05)),
            C_x=float(r["C_x"]),
            C_yn=float(r["C_yn"]),
            C_m_alpha=float(r.get("C_m_alpha_true", r.get("C_m_alpha", 1.054))),
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
    x, y, vx, vy, phi, omega, delta, alpha2f, ts_dot, a1_dot = state

    sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache = \
        controller.compute_control(state, params)

    F = sigma * params.F_max

    # Aerodynamics
    V = math.hypot(vx, vy)
    if V < 0.1:
        alpha_aoa = phi
        q_inf = 0.0
    else:
        alpha_aoa = phi - math.atan2(vx, vy)
        q_inf = 0.5 * params.rho * V * V

    X_b = -params.C_x  * q_inf * params.S_mid
    Y_b =  params.C_yn * alpha_aoa * q_inf * params.S_mid
    M_b =  params.C_m_alpha * alpha_aoa * q_inf * params.S_mid * params.l

    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    F_aero_x = X_b * sin_phi + Y_b * cos_phi
    F_aero_y = X_b * cos_phi - Y_b * sin_phi

    # Translational EOM
    x_ddot = (F * math.sin(phi + delta) + F_aero_x) / params.mass
    y_ddot = (F * math.cos(phi + delta) + F_aero_y) / params.mass - params.g

    # Rotational EOM: delta>0 tilts thrust right of body axis → torque increases phi
    phi_ddot = (F * params.l_cp * math.sin(delta) + M_b) / params.J_const

    # Nozzle actuator
    delta_dot = (-delta + delta_cmd) / params.tau_delta

    controller.last_cache = cache
    return np.array(
        [vx, vy, x_ddot, y_ddot, omega, phi_ddot,
         delta_dot, alpha2f_dot, ts_dot_dot, a1_dot_dot],
        dtype=float,
    )
