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
class BacksteppingController:
    """Full-system backstepping landing controller — fully stateless.

    theta_star_dot and alpha1_dot are ODE states (IDX_TS_DOT, IDX_A1_DOT),
    propagated as first-order low-pass filters:
        τ_d · ẋ_dot = -x_dot + (new_value - prev_value) / τ_d
    implemented as:
        d/dt [x_dot_filt] = N * (dx/dt_instant - x_dot_filt)
    where dx/dt_instant is computed algebraically each step.

    Returns: (sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache)
    """
    k_px: float
    k_dx: float
    k_py: float
    k_dy: float
    k_phi:   float
    k_omega: float
    k_delta: float
    tau_filter: float
    N_deriv: float = 50.0    # derivative filter bandwidth [rad/s]
    alpha1_max: float = 2.0  # virtual rate saturation [rad/s]
    x_d: float = 0.0

    last_cache: Dict = field(default_factory=dict, repr=False)

    @staticmethod
    def from_config(cfg: Dict) -> "BacksteppingController":
        ctl = cfg["controller"]
        return BacksteppingController(
            k_px=float(ctl["k_px"]),
            k_dx=float(ctl["k_dx"]),
            k_py=float(ctl["k_py"]),
            k_dy=float(ctl["k_dy"]),
            k_phi=float(ctl["k_phi"]),
            k_omega=float(ctl["k_omega"]),
            k_delta=float(ctl["k_delta"]),
            tau_filter=float(ctl.get("tau_filter", 0.02)),
            N_deriv=float(ctl.get("N_deriv", 50.0)),
            alpha1_max=float(ctl.get("alpha1_max", 2.0)),
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
        alpha2f = float(state[IDX_ALPHA2F])
        ts_dot  = float(state[IDX_TS_DOT])   # filtered theta_star_dot
        a1_dot  = float(state[IDX_A1_DOT])   # filtered alpha1_dot

        # ── Block A: Step 1 — theta_star, sigma ──────────────────────────────
        e_x = x - self.x_d
        e_y = y

        A_x = -self.k_px * e_x - self.k_dx * vx
        A_y = -self.k_py * e_y - self.k_dy * vy

        # Keep net vertical demand positive; limit tilt to 30°.
        # a_vert must be large enough to also satisfy the horizontal demand at
        # the tilt limit: |A_x| <= a_vert * tan(phi_lim).
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
        theta_star = math.atan2(A_x, A_y + params.g)

        # ── Block B: Aerodynamics ─────────────────────────────────────────────
        V_spd = math.hypot(vx, vy)
        if V_spd < 0.1:
            alpha_aoa = phi
            q_inf     = 0.0
        else:
            alpha_aoa = phi - math.atan2(vx, vy)
            q_inf     = 0.5 * params.rho * V_spd ** 2

        f2 = (params.C_m_alpha * alpha_aoa * q_inf
              * params.S_mid * params.l / params.J_const)
        # g2 > 0: positive delta → positive phi_ddot (matches README convention)
        g2 = F * params.l_cp / params.J_const

        # ── Block C: Step 2 — alpha1 ──────────────────────────────────────────
        z_phi = wrap_angle(phi - theta_star)

        P_coupling = (F / params.mass) * (
            vx * math.cos(theta_star) - vy * math.sin(theta_star)
        )

        # ts_dot is the ODE-integrated filtered theta_star_dot.
        # P_coupling (F/m)*(vx*cosθ* - vy*sinθ*) is the theoretical backstepping
        # feedforward, but with high entry velocity it reaches 60+ rad/s and
        # saturates alpha1 permanently, destroying the inner loops.  We use the
        # cascade form (no P) which is well-conditioned for all entry speeds.
        alpha1_raw = ts_dot - self.k_phi * z_phi
        alpha1     = float(np.clip(alpha1_raw, -self.alpha1_max, self.alpha1_max))
        z_omega    = omega - alpha1

        # ── Analytical derivatives of theta_star and alpha1 ──────────────────
        # theta_star = atan2(A_x, A_y+g)
        # d/dt[theta_star] = ((A_y+g)*dAx - A_x*dAy) / (A_x^2 + (A_y+g)^2)
        # dAx/dt = -k_px*vx - k_dx*ax,  dAy/dt = -k_py*vy - k_dy*ay
        # Use current thrust as dominant acceleration estimate.
        ax_approx  = F * math.sin(phi) / params.mass
        ay_approx  = F * math.cos(phi) / params.mass - params.g
        dAx_dt = -self.k_px * vx - self.k_dx * ax_approx
        dAy_dt = -self.k_py * vy - self.k_dy * ay_approx
        # Clamp denominator to avoid singularity when a_vert is near its minimum.
        # Use a_vert² as the floor (a_vert >= 0.5 m/s² by construction above).
        denom  = A_x**2 + (A_y + params.g)**2
        ts_dot_instant = ((A_y + params.g) * dAx_dt - A_x * dAy_dt) / max(denom, a_vert**2)
        # Cap to alpha1_max so it can't immediately saturate alpha1.
        ts_dot_instant = float(np.clip(ts_dot_instant, -self.alpha1_max, self.alpha1_max))

        # alpha1_raw = ts_dot - k_phi*z_phi  (cascade form, no P_coupling)
        # d/dt[alpha1_raw] ≈ d/dt[ts_dot] - k_phi*(omega - ts_dot)
        a1_dot_instant = ts_dot_instant - self.k_phi * (omega - ts_dot)

        # First-order filters: drive ODE states toward instantaneous values
        ts_dot_dot = self.N_deriv * (ts_dot_instant - ts_dot)
        a1_dot_dot = self.N_deriv * (a1_dot_instant - a1_dot)

        # a1_dot is the ODE-integrated filtered alpha1_dot
        # ── Block D: Step 3 — alpha2 ──────────────────────────────────────────
        alpha2_raw = (1.0 / g2) * (-self.k_omega * z_omega - z_phi - f2 + a1_dot)
        alpha2_sat = float(np.clip(alpha2_raw, -params.delta_max, params.delta_max))

        # ── Block E: Command filter ───────────────────────────────────────────
        alpha2f_dot = (alpha2_sat - alpha2f) / self.tau_filter
        z_delta     = delta - alpha2f

        # ── Block F: Step 4 — delta_cmd ──────────────────────────────────────
        Q_coupling = (F / params.mass) * (
            vx * math.cos(phi) - vy * math.sin(phi)
        )

        # Q_coupling (F/m)*(vx*cosφ - vy*sinφ) is the theoretical nozzle-position
        # cross term. With high entry velocities it reaches 50+ rad/s; multiplied
        # by tau_delta it adds ±100 deg to delta_cmd which destroys the command.
        # We use cascade-style delta_cmd (no Q) for robustness.
        delta_cmd = delta + params.tau_delta * (
            alpha2f_dot - g2 * z_omega - self.k_delta * z_delta
        )
        delta_cmd = float(np.clip(delta_cmd,
                                   -params.delta_max_cmd, params.delta_max_cmd))

        cache = {
            "sigma":          sigma,
            "delta_cmd":      delta_cmd,
            "theta_star":     theta_star,
            "theta_star_dot": ts_dot,
            "alpha1":         alpha1,
            "alpha1_dot":     a1_dot,
            "alpha2_raw":     alpha2_raw,
            "alpha2_sat":     alpha2_sat,
            "alpha2f":        alpha2f,
            "alpha2f_dot":    alpha2f_dot,
            "z_phi":          z_phi,
            "z_omega":        z_omega,
            "z_delta":        z_delta,
            "P_coupling":     P_coupling,
            "Q_coupling":     Q_coupling,
            "f2": f2,
            "g2": g2,
            "e_x": e_x,
            "e_y": e_y,
        }
        self.last_cache = cache
        return sigma, delta_cmd, alpha2f_dot, ts_dot_dot, a1_dot_dot, cache
