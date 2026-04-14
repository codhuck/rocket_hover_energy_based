from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Any
import math
import numpy as np
from scipy.integrate import solve_ivp

from .controller import LyapunovController
from .system import RocketParams, rocket_rhs


@dataclass
class SimulationResult:
    t: np.ndarray
    state: np.ndarray
    controls: Dict[str, np.ndarray]
    derived: Dict[str, np.ndarray]
    summary: Dict[str, float]
    params: RocketParams
    target: Dict[str, float]
    config: Dict[str, Any]


def build_initial_state(cfg: Dict) -> np.ndarray:
    init = cfg['initial_state']
    return np.array(
        [
            float(init['x']),
            float(init['y']),
            math.radians(float(init['phi_deg'])),
            float(init['vx']),
            float(init['vy']),
            math.radians(float(init['omega_deg_s'])),
            float(init['mass']),
        ],
        dtype=float,
    )


def simulate(cfg: Dict) -> SimulationResult:
    state0 = build_initial_state(cfg)
    params = RocketParams.from_config(cfg, initial_mass=float(state0[6]))
    controller = LyapunovController.from_config(cfg)
    target = cfg['target']
    exp = cfg['experiment']

    t_eval = np.linspace(0.0, float(exp['t_final']), int(exp['sample_count']))

    sol = solve_ivp(
        fun=lambda t, y: rocket_rhs(t, y, controller, params, target),
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
    alpha = np.zeros(n)
    delta = np.zeros(n)
    phi_des = np.zeros(n)
    e_x = np.zeros(n)
    e_y = np.zeros(n)
    e_phi = np.zeros(n)
    speed = np.zeros(n)
    thrust = np.zeros(n)
    tracking_error = np.zeros(n)

    for i in range(n):
        a, d, cache = controller.compute_control(state[i], target, params)
        if state[i, 6] <= params.m_dry + 1e-6:
            a, d = 0.0, 0.0
        alpha[i] = a
        delta[i] = d
        phi_des[i] = cache['phi_des']
        e_x[i] = cache['e_x']
        e_y[i] = cache['e_y']
        e_phi[i] = cache['e_phi']
        speed[i] = math.hypot(state[i, 3], state[i, 4])
        thrust[i] = a * params.F_max
        tracking_error[i] = math.hypot(e_x[i], e_y[i])

    t = sol.t
    fuel_used = max(float(state0[6] - state[-1, 6]), 0.0)
    summary = {
        'final_x': float(state[-1, 0]),
        'final_y': float(state[-1, 1]),
        'final_phi_deg': float(np.degrees(state[-1, 2])),
        'final_speed': float(speed[-1]),
        'final_tracking_error': float(tracking_error[-1]),
        'max_tracking_error': float(np.max(tracking_error)),
        'max_abs_phi_deg': float(np.max(np.abs(np.degrees(state[:, 2])))),
        'max_abs_delta_deg': float(np.max(np.abs(np.degrees(delta)))),
        'fuel_used': fuel_used,
        'min_mass': float(np.min(state[:, 6])),
        'hover_alpha_initial': float((state0[6] * params.g) / params.F_max),
        'hover_alpha_final': float((state[-1, 6] * params.g) / params.F_max),
        'params_l_cp': float(params.l_cp),
        'params_J_const': float(params.J_const),
        'F_max': float(params.F_max),
        'simulation_time': float(t[-1]),
    }
    derived = {
        'speed': speed,
        'thrust': thrust,
        'tracking_error': tracking_error,
        'target_x': np.full_like(t, float(target['x_d'])),
        'target_y': np.full_like(t, float(target['y_d'])),
    }
    controls = {
        'alpha': alpha,
        'delta': delta,
        'phi_des': phi_des,
        'e_x': e_x,
        'e_y': e_y,
        'e_phi': e_phi,
    }
    return SimulationResult(
        t=t,
        state=state,
        controls=controls,
        derived=derived,
        summary=summary,
        params=params,
        target=target,
        config=cfg,
    )
