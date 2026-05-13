from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

from .system import (
    IDX_X, IDX_Y, IDX_VX, IDX_VY, IDX_PHI, IDX_OMEGA,
    IDX_DELTA, IDX_ALPHA2F, IDX_TS_DOT, IDX_A1_DOT,
    RocketParams, wrap_angle,
)


@dataclass
class PIDCascadeController:
    """Cascaded PID landing controller — 3 nested loops, no feedforward.

    Structurally identical to backstepping but WITHOUT feedforward terms:
      - No theta_star_dot in omega_des  (no knowledge of how fast desired angle changes)
      - No f2 in alpha2               (no aero-moment cancellation)
      - No alpha1_dot in alpha2        (no rate-of-change feedforward)
      - No Q coupling in delta_cmd     (no velocity-nozzle coupling cancellation)

    This is the "fair comparison": same cascade structure, same actuator model,
    same gains — only the physics-based feedforward terms are removed.

    IDX_TS_DOT is repurposed as ∫(x − x_d) dt  (integral gain k_ix).
    IDX_A1_DOT is repurposed as ∫y dt           (integral gain k_iy).

    Returns: (sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache)
    """
    # Outer position loop
    k_px: float
    k_dx: float
    k_ix: float
    k_py: float
    k_dy: float
    k_iy: float
    # Middle attitude loop — PI (D is implicit: omega = d(phi)/dt handled by rate loop)
    k_phi: float
    k_i_phi: float         # integral of phi error (IDX_ALPHA2F slot)
    # Inner rate / actuator loop — PD
    k_omega: float
    k_delta: float
    tau_filter: float
    omega_des_max: float   # saturation on desired angular rate [rad/s]

    # Anti-windup limits for all integrators (set to large value = effectively off)
    i_x_max: float = 50.0    # clamp for I_x [m·s]
    i_y_max: float = 100.0   # clamp for I_y [m·s]
    i_phi_max: float = 1.0   # clamp for I_phi [rad·s]

    x_d: float = 0.0

    last_cache: Dict = field(default_factory=dict, repr=False)

    @staticmethod
    def from_config(cfg: Dict) -> "PIDCascadeController":
        ctl = cfg["controller"]
        return PIDCascadeController(
            k_px=float(ctl["k_px"]),
            k_dx=float(ctl["k_dx"]),
            k_ix=float(ctl.get("k_ix", 0.0)),
            k_py=float(ctl["k_py"]),
            k_dy=float(ctl["k_dy"]),
            k_iy=float(ctl.get("k_iy", 0.0)),
            k_phi=float(ctl["k_phi"]),
            k_i_phi=float(ctl.get("k_i_phi", 0.0)),
            k_omega=float(ctl["k_omega"]),
            k_delta=float(ctl["k_delta"]),
            tau_filter=float(ctl.get("tau_filter", 0.02)),
            omega_des_max=float(ctl.get("omega_des_max", 1.5)),
            i_x_max=float(ctl.get("i_x_max", 50.0)),
            i_y_max=float(ctl.get("i_y_max", 100.0)),
            i_phi_max=float(ctl.get("i_phi_max", 1.0)),
            x_d=float(ctl.get("x_d", 0.0)),
        )

    def compute_control(
        self,
        state: np.ndarray,
        params: RocketParams,
    ) -> Tuple[float, float, float, float, float, Dict]:
        """Return (sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache)."""
        x       = float(state[IDX_X])
        y       = float(state[IDX_Y])
        vx      = float(state[IDX_VX])
        vy      = float(state[IDX_VY])
        phi     = float(state[IDX_PHI])
        omega   = float(state[IDX_OMEGA])
        delta   = float(state[IDX_DELTA])
        I_phi   = float(state[IDX_ALPHA2F])  # ∫e_phi dt  (attitude integral)
        I_x     = float(state[IDX_TS_DOT])   # ∫(x - x_d) dt
        I_y     = float(state[IDX_A1_DOT])   # ∫y dt

        # ── Loop 1: position → sigma + theta_des (same Block A as backstepping) ──
        e_x = x - self.x_d
        e_y = y

        I_x_c = float(np.clip(I_x, -self.i_x_max, self.i_x_max))
        I_y_c = float(np.clip(I_y, -self.i_y_max, self.i_y_max))
        A_x = -(self.k_px * e_x + self.k_dx * vx + self.k_ix * I_x_c)
        A_y = -(self.k_py * e_y + self.k_dy * vy + self.k_iy * I_y_c)

        phi_lim = math.radians(30.0)
        a_vert  = max(A_y + params.g,
                      abs(A_x) / math.tan(phi_lim),
                      0.5)
        A_y     = a_vert - params.g
        A_x     = float(np.clip(A_x, -a_vert * math.tan(phi_lim),
                                       a_vert * math.tan(phi_lim)))

        T_des     = math.hypot(A_x, A_y + params.g)
        sigma     = float(np.clip(params.mass * T_des / params.F_max,
                                   params.sigma_min, 1.0))
        F         = sigma * params.F_max
        theta_des = math.atan2(A_x, A_y + params.g)

        # ── Nozzle gain (g2) — needed for correct inner-loop scaling ─────────
        g2 = F * params.l_cp / params.J_const   # [rad/s² per rad]

        # ── Loop 2: angle error → desired angular rate ────────────────────────
        e_phi     = wrap_angle(phi - theta_des)
        # PI on attitude: P + I  (D implicit — omega = d(phi)/dt is handled by rate loop)
        I_phi_c   = float(np.clip(I_phi, -self.i_phi_max, self.i_phi_max))
        omega_des = float(np.clip(
            -(self.k_phi * e_phi + self.k_i_phi * I_phi_c),
            -self.omega_des_max, self.omega_des_max,
        ))

        # ── Loop 3 / Step 3: desired nozzle angle (alpha2) ───────────────────
        # Backstepping: alpha2 = (1/g2)*(-k_omega*z_omega - z_phi - f2 + a1_dot)
        # PID omits f2 (aero moment) and a1_dot (feedforward of rate-of-rate):
        e_omega    = omega - omega_des
        alpha2_raw = (1.0 / g2) * (-self.k_omega * e_omega - e_phi)
        alpha2_sat = float(np.clip(alpha2_raw, -params.delta_max, params.delta_max))

        # ── Attitude integral and nozzle tracking error ───────────────────────
        # alpha2f_dot drives IDX_ALPHA2F: repurposed as d/dt[I_phi] = e_phi
        alpha2f_dot = e_phi
        # Use alpha2_sat directly as the nozzle reference (no command filter in PID)
        z_delta     = delta - alpha2_sat

        # ── Loop 4 / Step 4: delta_cmd ────────────────────────────────────────
        # Drive nozzle toward alpha2_sat, damp on rate error and nozzle-tracking error
        delta_cmd = delta + params.tau_delta * (
            -g2 * e_omega - self.k_delta * z_delta
        )
        delta_cmd = float(np.clip(delta_cmd, -params.delta_max_cmd, params.delta_max_cmd))

        # ── ODE state derivatives ─────────────────────────────────────────────
        ts_dot_dot = e_x   # d/dt[I_x] = e_x
        a1_dot_dot = e_y   # d/dt[I_y] = e_y

        cache = {
            "sigma":          sigma,
            "delta_cmd":      delta_cmd,
            "theta_star":     theta_des,
            "theta_star_dot": 0.0,
            "alpha1":         omega_des,
            "alpha1_dot":     0.0,
            "alpha2_raw":     alpha2_raw,
            "alpha2_sat":     alpha2_sat,
            "alpha2f":        alpha2_sat,   # show desired nozzle angle in plots
            "alpha2f_dot":    alpha2f_dot,
            "z_phi":          e_phi,
            "z_omega":        e_omega,
            "z_delta":        z_delta,
            "P_coupling":     0.0,
            "Q_coupling":     0.0,
            "f2":             0.0,
            "g2":             g2,
            "e_x":            e_x,
            "e_y":            e_y,
        }
        self.last_cache = cache
        return sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache
