from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Any
import math
import numpy as np
from scipy.integrate import solve_ivp

from .controller import AttitudeLyapunovController
from .system import RocketParams, rocket_rhs


@dataclass
class SimulationResult:
    t: np.ndarray
    state: np.ndarray
    controls: Dict[str, np.ndarray]
    derived: Dict[str, np.ndarray]
    summary: Dict[str, float]
    params: RocketParams
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
        ],
        dtype=float,
    )


def simulate(cfg: Dict) -> SimulationResult:
    state0 = build_initial_state(cfg)
    params = RocketParams.from_config(cfg)
    controller = AttitudeLyapunovController.from_config(cfg)
    exp = cfg['experiment']

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
    alpha = np.zeros(n)
    delta = np.zeros(n)
    e_phi = np.zeros(n)
    speed = np.zeros(n)
    vertical_accel = np.zeros(n)
    horiz_accel = np.zeros(n)

    for i in range(n):
        a, d, cache = controller.compute_control(state[i], params)
        alpha[i] = a
        delta[i] = d
        e_phi[i] = cache['e_phi']
        speed[i] = math.hypot(state[i, 3], state[i, 4])
        thrust = a * params.F_max
        horiz_accel[i] = (thrust / params.mass) * math.sin(state[i, 2] + d)
        vertical_accel[i] = (thrust / params.mass) * math.cos(state[i, 2] + d) - params.g

    t = sol.t
    summary = {
        'final_x': float(state[-1, 0]),
        'final_y': float(state[-1, 1]),
        'final_phi_deg': float(np.degrees(state[-1, 2])),
        'final_omega_deg_s': float(np.degrees(state[-1, 5])),
        'final_speed': float(speed[-1]),
        'max_abs_phi_deg': float(np.max(np.abs(np.degrees(state[:, 2])))),
        'max_abs_delta_deg': float(np.max(np.abs(np.degrees(delta)))),
        'max_speed': float(np.max(speed)),
        'hover_alpha': float(params.alpha_hover),
        'params_l_cp': float(params.l_cp),
        'params_J_const': float(params.J_const),
        'F_max': float(params.F_max),
        'mass': float(params.mass),
        'simulation_time': float(t[-1]),
    }
    derived = {
        'speed': speed,
        'horiz_accel': horiz_accel,
        'vertical_accel': vertical_accel,
    }
    controls = {
        'alpha': alpha,
        'delta': delta,
        'e_phi': e_phi,
        'phi_target': np.zeros_like(t) + controller.phi_target,
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