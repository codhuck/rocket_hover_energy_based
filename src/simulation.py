from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np
from scipy.integrate import solve_ivp

from .controller import AttitudeLyapunovController, AdaptiveCEController
from .system import (
    IDX_X, IDX_Y, IDX_THETA, IDX_VX, IDX_VY, IDX_THETA_DOT, IDX_C_HAT,
    STATE_DIM,
    RocketParams, rocket_rhs,
)


@dataclass
class SimulationResult:
    t: np.ndarray
    state: np.ndarray
    controls: Dict[str, np.ndarray]
    derived: Dict[str, np.ndarray]
    summary: Dict[str, float]
    params: RocketParams
    config: Dict[str, Any]


def build_controller(cfg: Dict):
    """Factory: pick controller type from config['controller']['type']."""
    ctrl_cfg = cfg['controller']
    ctrl_type = ctrl_cfg.get('type', 'adaptive_ce')

    if ctrl_type == 'lyapunov':
        return AttitudeLyapunovController.from_config(cfg)
    elif ctrl_type == 'adaptive_ce':
        return AdaptiveCEController.from_config(cfg)
    else:
        raise ValueError(f"Unknown controller type: {ctrl_type}")


def build_initial_state(cfg: Dict) -> np.ndarray:
    """Builds the augmented initial state (7-dim).

    Physical states come from cfg['initial_state'].
    The initial parameter estimate c_hat(0) comes from
    cfg['controller']['c_hat_initial'] (or 0.0 for non-adaptive controllers).
    """
    init = cfg['initial_state']
    ctrl_cfg = cfg.get('controller', {})

    state = np.zeros(STATE_DIM)
    state[IDX_X] = float(init['x'])
    state[IDX_Y] = float(init['y'])
    state[IDX_THETA] = math.radians(float(init['theta_deg']))
    state[IDX_VX] = float(init['vx'])
    state[IDX_VY] = float(init['vy'])
    state[IDX_THETA_DOT] = math.radians(float(init['omega_deg_s']))
    state[IDX_C_HAT] = float(ctrl_cfg.get('c_hat_initial', 0.0))
    return state


def compute_settling_time(
    t: np.ndarray,
    signal_abs: np.ndarray,
    threshold: float,
) -> float | None:
    for i, t_i in enumerate(t):
        if np.all(signal_abs[i:] < threshold):
            return float(t_i)
    return None


def simulate(cfg: Dict) -> SimulationResult:
    state0 = build_initial_state(cfg)
    params = RocketParams.from_config(cfg)
    controller = build_controller(cfg)

    exp = cfg['experiment']
    analysis_cfg = cfg.get('analysis', {})
    theta_error_band_rad = float(analysis_cfg.get('theta_error_band_rad', 0.1))
    t_eval = np.linspace(0.0, float(exp['t_final']), int(exp['sample_count']))

    sol = solve_ivp(
        fun=lambda t, y: rocket_rhs(t, y, controller, params),
        t_span=(0.0, float(exp['t_final'])),
        y0=state0,
        t_eval=t_eval,
        max_step=float(exp['max_step']),
        rtol=1e-7,
        atol=1e-9,
    )
    if not sol.success:
        raise RuntimeError(f"Integration failed: {sol.message}")

    state = sol.y.T
    n = state.shape[0]

    #Recompute control trajectories from the integrated state
    throttle = np.zeros(n)
    delta = np.zeros(n)
    c_hat_dot = np.zeros(n)
    e_theta = np.zeros(n)
    speed = np.zeros(n)
    vertical_accel = np.zeros(n)
    horiz_accel = np.zeros(n)

    for i in range(n):
        thr, d, c_dot, cache = controller.compute_control(state[i], params)
        throttle[i] = thr
        delta[i] = d
        c_hat_dot[i] = c_dot
        e_theta[i] = cache['e_theta']

        vx, vy = state[i, IDX_VX], state[i, IDX_VY]
        theta_i = state[i, IDX_THETA]
        speed[i] = math.hypot(vx, vy)
        thrust_i = thr * params.F_max
        horiz_accel[i] = (thrust_i / params.mass) * math.sin(theta_i + d)
        vertical_accel[i] = (thrust_i / params.mass) * math.cos(theta_i + d) - params.g

    t = sol.t
    settling_time = compute_settling_time(t, np.abs(e_theta), theta_error_band_rad)

    summary = {
        'final_theta_deg': float(np.degrees(state[-1, IDX_THETA])),
        'final_theta_dot_deg_s': float(np.degrees(state[-1, IDX_THETA_DOT])),
        'final_c_hat': float(state[-1, IDX_C_HAT]),
        'c_hat_true': float(params.C_m_alpha_true),
        'c_hat_error': float(state[-1, IDX_C_HAT] - params.C_m_alpha_true),
        'max_abs_theta_deg': float(np.max(np.abs(np.degrees(state[:, IDX_THETA])))),
        'max_abs_delta_deg': float(np.max(np.abs(np.degrees(delta)))),
        'max_speed': float(np.max(speed)),
        'theta_error_band_rad': theta_error_band_rad,
        'settling_time_theta_error_band_s': settling_time,
        'simulation_time': float(t[-1]),
    }

    derived = {
        'speed': speed,
        'horiz_accel': horiz_accel,
        'vertical_accel': vertical_accel,
        'c_hat': state[:, IDX_C_HAT].copy(),
        'c_hat_dot': c_hat_dot,
    }

    controls = {
        'throttle': throttle,
        'delta': delta,
        'e_theta': e_theta,
        'theta_target': np.zeros_like(t) + getattr(controller, 'theta_target', 0.0),
    }

    return SimulationResult(
        t=t,
        state=state,
        controls=controls,
        derived=derived,
        summary=summary,
        params=params,
        config=cfg,
    )
