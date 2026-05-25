from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List

import numpy as np

from .mpc_controller import MPCController
from .system import (
    IDX_DELTA,
    IDX_OMEGA,
    IDX_PHI,
    IDX_VX,
    IDX_VY,
    IDX_X,
    IDX_Y,
    STATE_DIM,
    RocketParams,
    aero_forces_and_moment,
    rk4_step,
)


@dataclass
class SimulationResult:
    t: np.ndarray
    state: np.ndarray
    controls: Dict[str, np.ndarray]
    derived: Dict[str, np.ndarray]
    summary: Dict[str, Any]
    params: RocketParams
    config: Dict[str, Any]


def build_controller(cfg: Dict) -> MPCController:
    ctrl_type = cfg.get("controller", {}).get("type", "mpc")
    if ctrl_type != "mpc":
        raise ValueError(
            f"This MPC version expects controller.type: 'mpc'; got {ctrl_type!r}. "
            "Use configs/mpc.yaml."
        )
    return MPCController.from_config(cfg)


def build_initial_state(cfg: Dict) -> np.ndarray:
    init = cfg.get("initial_state", {})
    mission = cfg.get("mission", {})
    state = np.zeros(STATE_DIM, dtype=float)
    state[IDX_X] = float(init.get("x", mission.get("x_start", 0.0)))
    state[IDX_Y] = float(init.get("y", 0.0))
    state[IDX_VX] = float(init.get("vx", 0.0))
    state[IDX_VY] = float(init.get("vy", 0.0))
    state[IDX_PHI] = math.radians(float(init.get("theta_deg", 0.0)))
    state[IDX_OMEGA] = math.radians(float(init.get("omega_deg_s", 0.0)))
    state[IDX_DELTA] = math.radians(float(init.get("delta_deg", 0.0)))
    return state


def _append_cache(cache: Dict, derived_lists: Dict[str, List[float]]) -> None:
    for key in ["V", "alpha", "q_inf", "C_x", "C_y", "m_z", "X_b", "Y_b", "M_b", "cost", "nit"]:
        value = cache.get(key, np.nan)
        if isinstance(value, np.ndarray):
            value = np.nan
        derived_lists.setdefault(key, []).append(float(value))
    derived_lists.setdefault("phase", []).append(0.0 if cache.get("phase") == "ascent" else 1.0)
    derived_lists.setdefault("solver_success", []).append(1.0 if cache.get("success", False) else 0.0)


def simulate(cfg: Dict) -> SimulationResult:
    params = RocketParams.from_config(cfg)
    controller = build_controller(cfg)
    state = build_initial_state(cfg)

    exp = cfg.get("experiment", {})
    dt = float(exp.get("dt", controller.dt))
    t_final = float(exp.get("t_final", 60.0))
    max_steps = int(math.ceil(t_final / dt))

    t_values: List[float] = [0.0]
    states: List[np.ndarray] = [state.copy()]
    delta_cmd: List[float] = []
    thrust: List[float] = []
    sigma: List[float] = []
    phases: List[float] = []
    derived_lists: Dict[str, List[float]] = {}

    landing_time = None

    for k in range(max_steps):
        control, cache = controller.compute_control(state, params)
        u = np.array([control["delta_cmd"], control["F"]], dtype=float)

        delta_cmd.append(float(control["delta_cmd"]))
        thrust.append(float(control["F"]))
        sigma.append(float(control["sigma"]))
        phases.append(0.0 if cache.get("phase") == "ascent" else 1.0)
        _append_cache(cache, derived_lists)

        next_state = rk4_step(state, u, dt, params)

        # Contact with the landing pad is terminal.  The previous prototype
        # clamped y to zero and kept flying, which could hide a hard or tilted
        # touchdown.  Now we stop and record whether the contact was acceptable.
        contact = controller.phase == "descent" and next_state[IDX_Y] <= 0.0
        if contact:
            next_state[IDX_Y] = 0.0

        state = next_state
        controller.update_phase(state)

        t_next = (k + 1) * dt
        t_values.append(t_next)
        states.append(state.copy())

        if contact:
            landing_time = t_next
            break

        if controller.mission_done(state):
            landing_time = t_next
            break

    # Duplicate the last command so every state sample has a command value.
    if delta_cmd:
        delta_cmd.append(delta_cmd[-1])
        thrust.append(thrust[-1])
        sigma.append(sigma[-1])
        phases.append(phases[-1])
        # Add final-state aero diagnostics.
        final_aero = aero_forces_and_moment(state, params)
        final_cache = {**controller.last_cache, **final_aero}
        _append_cache(final_cache, derived_lists)
    else:
        delta_cmd = [0.0]
        thrust = [params.F_hover]
        sigma = [params.throttle_hover]
        phases = [0.0]
        _append_cache(aero_forces_and_moment(state, params), derived_lists)

    t = np.asarray(t_values, dtype=float)
    x_arr = np.vstack(states)
    speed = np.hypot(x_arr[:, IDX_VX], x_arr[:, IDX_VY])

    # Convert derived lists to arrays and align lengths.
    derived = {key: np.asarray(value[: len(t)], dtype=float) for key, value in derived_lists.items()}
    derived["speed"] = speed
    derived["alpha_deg"] = np.degrees(derived.get("alpha", np.zeros_like(t)))
    derived["phase"] = np.asarray(phases[: len(t)], dtype=float)

    controls = {
        "delta_cmd": np.asarray(delta_cmd[: len(t)], dtype=float),
        "F": np.asarray(thrust[: len(t)], dtype=float),
        "sigma": np.asarray(sigma[: len(t)], dtype=float),
        "phase": np.asarray(phases[: len(t)], dtype=float),
    }

    tq = controller.touchdown_quality(x_arr[-1])
    touchdown_success = bool(tq["success"])
    if touchdown_success:
        termination_reason = "successful_landing"
    elif landing_time is not None and x_arr[-1, IDX_Y] <= controller.mission.eps_land:
        termination_reason = "hard_or_off_target_landing"
    elif controller.phase == "ascent":
        termination_reason = "time_limit_before_descent"
    else:
        termination_reason = "time_limit_in_descent"

    summary: Dict[str, Any] = {
        "controller_type": "mpc",
        "termination_reason": termination_reason,
        "touchdown_success": touchdown_success,
        "landing_x_error": float(tq["x_error"]),
        "landing_vx_abs": float(tq["vx_abs"]),
        "landing_vy_abs": float(tq["vy_abs"]),
        "landing_theta_abs_deg": float(np.degrees(tq["theta_abs"])),
        "landing_omega_abs_deg_s": float(np.degrees(tq["omega_abs"])),
        "final_phase": "descent" if controller.phase == "descent" else "ascent",
        "final_x": float(x_arr[-1, IDX_X]),
        "final_y": float(x_arr[-1, IDX_Y]),
        "final_vx": float(x_arr[-1, IDX_VX]),
        "final_vy": float(x_arr[-1, IDX_VY]),
        "final_phi_deg": float(np.degrees(x_arr[-1, IDX_PHI])),
        "final_omega_deg_s": float(np.degrees(x_arr[-1, IDX_OMEGA])),
        "final_delta_deg": float(np.degrees(x_arr[-1, IDX_DELTA])),
        "final_speed": float(speed[-1]),
        "max_speed": float(np.max(speed)),
        "max_altitude": float(np.max(x_arr[:, IDX_Y])),
        "max_abs_phi_deg": float(np.max(np.abs(np.degrees(x_arr[:, IDX_PHI])))),
        "max_abs_delta_deg": float(np.max(np.abs(np.degrees(x_arr[:, IDX_DELTA])))),
        "landing_time_s": landing_time,
        "simulation_time": float(t[-1]),
        "solver_success_rate": float(np.mean(derived.get("solver_success", np.ones_like(t)))),
    }

    return SimulationResult(t=t, state=x_arr, controls=controls, derived=derived, summary=summary, params=params, config=cfg)
