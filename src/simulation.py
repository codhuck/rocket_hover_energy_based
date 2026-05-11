from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np
from scipy.integrate import solve_ivp

from .controller import BacksteppingController
from .system import (
    IDX_X, IDX_Y, IDX_VX, IDX_VY, IDX_PHI, IDX_OMEGA,
    IDX_DELTA, IDX_ALPHA2F, IDX_TS_DOT, IDX_A1_DOT,
    STATE_DIM, RocketParams, rocket_rhs,
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


def build_controller(cfg: Dict) -> BacksteppingController:
    ctrl_type = cfg["controller"].get("type", "backstepping")
    if ctrl_type != "backstepping":
        raise ValueError(f"Only 'backstepping' supported; got '{ctrl_type}'")
    return BacksteppingController.from_config(cfg)


def build_initial_state(cfg: Dict) -> np.ndarray:
    init = cfg["initial_state"]
    state = np.zeros(STATE_DIM)
    state[IDX_X]       = float(init["x"])
    state[IDX_Y]       = float(init["y"])
    state[IDX_VX]      = float(init["vx"])
    state[IDX_VY]      = float(init["vy"])
    state[IDX_PHI]     = math.radians(float(init["theta_deg"]))
    state[IDX_OMEGA]   = math.radians(float(init.get("omega_deg_s", 0.0)))
    state[IDX_DELTA]   = math.radians(float(init.get("delta_deg", 0.0)))
    state[IDX_ALPHA2F] = math.radians(float(init.get("delta_deg", 0.0)))
    # ts_dot, a1_dot start at zero
    return state


def simulate(cfg: Dict) -> SimulationResult:
    state0     = build_initial_state(cfg)
    params     = RocketParams.from_config(cfg)
    controller = build_controller(cfg)

    exp    = cfg["experiment"]
    t_span = (0.0, float(exp["t_final"]))
    t_eval = np.linspace(t_span[0], t_span[1], int(exp["sample_count"]))

    # Stop when y <= 0.3 m (the controller's exponential approach never crosses y=0
    # exactly, so we declare touchdown at 30 cm above the pad).
    def touchdown(t, y):
        return y[IDX_Y] - 0.3
    touchdown.terminal  = True
    touchdown.direction = -1   # only trigger when y is decreasing

    sol = solve_ivp(
        fun=lambda t, y: rocket_rhs(t, y, controller, params),
        t_span=t_span,
        y0=state0,
        t_eval=t_eval,
        max_step=float(exp.get("max_step", 0.02)),
        rtol=1e-5,
        atol=1e-7,
        events=touchdown,
    )
    if not sol.success and "Required step size" not in sol.message:
        raise RuntimeError(f"Integration failed: {sol.message}")

    state = sol.y.T
    t     = sol.t
    n     = len(t)

    # Reconstruct diagnostics by replaying controller on saved trajectory.
    # Controller is stateless so this is safe and fast.
    sigma      = np.zeros(n)
    delta_cmd  = np.zeros(n)
    theta_star = np.zeros(n)
    alpha1     = np.zeros(n)
    alpha2f    = np.zeros(n)
    z_phi      = np.zeros(n)
    z_omega    = np.zeros(n)
    z_delta    = np.zeros(n)
    P_c        = np.zeros(n)
    Q_c        = np.zeros(n)

    for i in range(n):
        sig, dcmd, _, _, _, cache = controller.compute_control(state[i], params)
        sigma[i]      = sig
        delta_cmd[i]  = dcmd
        theta_star[i] = cache["theta_star"]
        alpha1[i]     = cache["alpha1"]
        alpha2f[i]    = cache["alpha2f"]
        z_phi[i]      = cache["z_phi"]
        z_omega[i]    = cache["z_omega"]
        z_delta[i]    = cache["z_delta"]
        P_c[i]        = cache["P_coupling"]
        Q_c[i]        = cache["Q_coupling"]

    speed  = np.hypot(state[:, IDX_VX], state[:, IDX_VY])
    ex_arr = state[:, IDX_X] - controller.x_d
    ey_arr = state[:, IDX_Y]
    lyap_V = (
        0.5 * controller.k_px * ex_arr**2 + 0.5 * state[:, IDX_VX]**2 +
        0.5 * controller.k_py * ey_arr**2 + 0.5 * state[:, IDX_VY]**2 +
        0.5 * z_phi**2 + 0.5 * z_omega**2 + 0.5 * z_delta**2
    )

    # Landing time from terminal event
    landing_time = float(sol.t_events[0][0]) if len(sol.t_events[0]) > 0 else None

    summary = {
        "controller_type":    "backstepping",
        "final_x":            float(state[-1, IDX_X]),
        "final_y":            float(state[-1, IDX_Y]),
        "final_vx":           float(state[-1, IDX_VX]),
        "final_vy":           float(state[-1, IDX_VY]),
        "final_phi_deg":      float(np.degrees(state[-1, IDX_PHI])),
        "final_omega_deg_s":  float(np.degrees(state[-1, IDX_OMEGA])),
        "final_delta_deg":    float(np.degrees(state[-1, IDX_DELTA])),
        "final_speed":        float(speed[-1]),
        "max_speed":          float(np.max(speed)),
        "max_abs_phi_deg":    float(np.max(np.abs(np.degrees(state[:, IDX_PHI])))),
        "max_abs_delta_deg":  float(np.max(np.abs(np.degrees(state[:, IDX_DELTA])))),
        "lyap_V_initial":     float(lyap_V[0]),
        "lyap_V_final":       float(lyap_V[-1]),
        "landing_time_s":     landing_time,
        "simulation_time":    float(t[-1]),
    }

    controls = {
        "sigma":      sigma,
        "delta_cmd":  delta_cmd,
        "theta_star": theta_star,
        "alpha1":     alpha1,
        "alpha2f":    alpha2f,
    }

    derived = {
        "speed":      speed,
        "z_phi":      z_phi,
        "z_omega":    z_omega,
        "z_delta":    z_delta,
        "P_coupling": P_c,
        "Q_coupling": Q_c,
        "lyap_V":     lyap_V,
    }

    return SimulationResult(
        t=t, state=state, controls=controls, derived=derived,
        summary=summary, params=params, config=cfg,
    )
