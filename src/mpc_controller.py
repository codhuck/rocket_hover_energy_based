from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np
from scipy.optimize import minimize

from .system import (
    IDX_X,
    IDX_Y,
    IDX_VX,
    IDX_VY,
    IDX_PHI,
    IDX_OMEGA,
    IDX_DELTA,
    STATE_DIM,
    RocketParams,
    aero_forces_and_moment,
    rk4_step,
    wrap_angle,
)


@dataclass
class MPCWeights:
    """Dimensionless weights used by the single-shooting MPC.

    The nominal MPC formulation keeps the stage cost on the controls only.  The
    additional near-ground and touchdown terms below are soft versions of landing
    constraints: they only become active in the descent phase close to the pad or
    at predicted contact with the ground.
    """

    R_delta: float = 0.10
    R_F: float = 0.04
    R_delta_rate: float = 0.02
    R_F_rate: float = 0.02

    # Terminal weights.
    px: float = 80.0
    py: float = 70.0
    pvx: float = 60.0
    pvy: float = 90.0
    ptheta: float = 70.0
    pomega: float = 18.0
    pdelta: float = 0.0

    # Soft state constraints and soft landing constraints.
    ground: float = 8_000.0
    theta_limit: float = 3_000.0
    delta_limit: float = 2_000.0
    near_ground: float = 35.0
    touchdown_x: float = 250.0
    touchdown_v: float = 220.0
    touchdown_theta: float = 180.0
    touchdown_omega: float = 45.0


@dataclass
class MPCScales:
    x: float = 80.0
    y: float = 100.0
    vx: float = 10.0
    vy: float = 10.0
    theta: float = math.radians(12.0)
    omega: float = math.radians(30.0)
    delta: float = math.radians(15.0)
    F: float = 60_000.0


@dataclass
class MissionConfig:
    x_start: float = 0.0
    x_mid: float = 40.0
    x_land: float = 80.0
    y_land: float = 0.0
    h_target: float = 120.0
    eps_h: float = 3.0
    eps_v: float = 2.0
    eps_land: float = 0.7
    eps_vel: float = 3.0

    # Strict touchdown gates.  The old version only checked height and total
    # speed, which could mark a tilted off-target contact as a landing.
    eps_x: float = 5.0
    eps_vx: float = 1.5
    eps_vy: float = 1.5
    eps_theta: float = math.radians(6.0)
    eps_omega: float = math.radians(10.0)


@dataclass
class MPCController:
    """Single-shooting nonlinear MPC for the 7-state TVC rocket.

    Decision variables are a blocked sequence of controls.  Blocking keeps the
    optimization small enough to use a longer prediction horizon, which is much
    more important for a good landing than optimizing every individual sample.
    The expanded sequence is still applied in a receding-horizon way: only the
    first control is sent to the plant.
    """

    N: int = 24
    dt: float = 0.25
    control_blocks: int = 9      # legacy alias for thrust_blocks
    gimbal_blocks: int = 24      # usually one gimbal decision per MPC sample
    thrust_blocks: int = 8       # slower thrust move blocking
    max_iter: int = 30
    ftol: float = 8e-4
    optimizer: str = "coordinate"
    search_passes: int = 1
    initial_step: float = 0.35
    landing_zone: float = 35.0
    landing_assist: bool = True
    assist_altitude: float = 32.0
    assist_blend: float = 1.0
    assist_override_descent: bool = True
    weights: MPCWeights = field(default_factory=MPCWeights)
    scales: MPCScales = field(default_factory=MPCScales)
    mission: MissionConfig = field(default_factory=MissionConfig)
    phase: str = "ascent"
    warm_u: np.ndarray | None = None
    last_cache: Dict = field(default_factory=dict, repr=False)

    @staticmethod
    def from_config(cfg: Dict) -> "MPCController":
        c = cfg.get("controller", {})
        w_cfg = c.get("weights", {})
        s_cfg = c.get("scales", {})
        m_cfg = cfg.get("mission", {})

        weights = MPCWeights(**{k: float(v) for k, v in w_cfg.items() if hasattr(MPCWeights, k)})
        scales = MPCScales(**{k: float(v) for k, v in s_cfg.items() if hasattr(MPCScales, k)})
        mission = MissionConfig(
            x_start=float(m_cfg.get("x_start", m_cfg.get("xstart", 0.0))),
            x_mid=float(m_cfg.get("x_mid", m_cfg.get("xmid", 40.0))),
            x_land=float(m_cfg.get("x_land", m_cfg.get("xland", 80.0))),
            y_land=float(m_cfg.get("y_land", m_cfg.get("yland", 0.0))),
            h_target=float(m_cfg.get("h_target", m_cfg.get("htarget", 120.0))),
            eps_h=float(m_cfg.get("eps_h", 3.0)),
            eps_v=float(m_cfg.get("eps_v", 2.0)),
            eps_land=float(m_cfg.get("eps_land", 0.7)),
            eps_vel=float(m_cfg.get("eps_vel", 3.0)),
            eps_x=float(m_cfg.get("eps_x", 5.0)),
            eps_vx=float(m_cfg.get("eps_vx", 1.5)),
            eps_vy=float(m_cfg.get("eps_vy", 1.5)),
            eps_theta=math.radians(float(m_cfg.get("eps_theta_deg", 6.0))),
            eps_omega=math.radians(float(m_cfg.get("eps_omega_deg_s", 10.0))),
        )
        N = int(c.get("N", 26))
        return MPCController(
            N=N,
            dt=float(c.get("dt", cfg.get("experiment", {}).get("dt", 0.25))),
            control_blocks=max(1, min(N, int(c.get("control_blocks", c.get("M", 9))))),
            gimbal_blocks=max(1, min(N, int(c.get("gimbal_blocks", N)))),
            thrust_blocks=max(1, min(N, int(c.get("thrust_blocks", c.get("control_blocks", c.get("M", 9)))))),
            max_iter=int(c.get("max_iter", 30)),
            ftol=float(c.get("ftol", 8e-4)),
            optimizer=str(c.get("optimizer", "coordinate")).lower(),
            search_passes=int(c.get("search_passes", max(1, min(5, c.get("max_iter", 30) // 10 or 1)))),
            initial_step=float(c.get("initial_step", 0.35)),
            landing_zone=float(c.get("landing_zone", 35.0)),
            landing_assist=bool(c.get("landing_assist", True)),
            assist_altitude=float(c.get("assist_altitude", 32.0)),
            assist_blend=float(c.get("assist_blend", 1.0)),
            assist_override_descent=bool(c.get("assist_override_descent", True)),
            weights=weights,
            scales=scales,
            mission=mission,
        )

    def _block_index(self, n_blocks: int) -> np.ndarray:
        # Maps every horizon sample k=0..N-1 to a block j=0..n_blocks-1.
        return np.minimum((np.arange(self.N) * n_blocks) // self.N, n_blocks - 1)

    def _control_to_z(self, U: np.ndarray, params: RocketParams) -> np.ndarray:
        U = np.asarray(U, dtype=float).reshape(self.N, 2)
        z_g = np.zeros(self.gimbal_blocks, dtype=float)
        z_f = np.zeros(self.thrust_blocks, dtype=float)
        g_idx = self._block_index(self.gimbal_blocks)
        f_idx = self._block_index(self.thrust_blocks)
        center = 0.5 * (params.F_max + params.F_min)
        half = 0.5 * (params.F_max - params.F_min)
        for j in range(self.gimbal_blocks):
            members = U[g_idx == j, 0]
            if len(members) == 0:
                members = U[-1:, 0]
            z_g[j] = np.mean(members) / max(params.delta_max_cmd, 1e-9)
        for j in range(self.thrust_blocks):
            members = U[f_idx == j, 1]
            if len(members) == 0:
                members = U[-1:, 1]
            z_f[j] = (np.mean(members) - center) / max(half, 1e-9)
        return np.clip(np.concatenate([z_g, z_f]), -1.0, 1.0)

    def _z_to_control(self, z: np.ndarray, params: RocketParams) -> np.ndarray:
        z = np.asarray(z, dtype=float)
        z_g = np.clip(z[: self.gimbal_blocks], -1.0, 1.0)
        z_f = np.clip(z[self.gimbal_blocks : self.gimbal_blocks + self.thrust_blocks], -1.0, 1.0)
        g_blocks = params.delta_max_cmd * z_g
        center = 0.5 * (params.F_max + params.F_min)
        half = 0.5 * (params.F_max - params.F_min)
        f_blocks = center + half * z_f
        U = np.zeros((self.N, 2), dtype=float)
        U[:, 0] = g_blocks[self._block_index(self.gimbal_blocks)]
        U[:, 1] = f_blocks[self._block_index(self.thrust_blocks)]
        return U

    def _default_warm_start(self, state: np.ndarray, params: RocketParams) -> np.ndarray:
        U = np.zeros((self.N, 2), dtype=float)
        if self.phase == "ascent":
            x_err = self.mission.x_mid - float(state[IDX_X])
            vx = float(state[IDX_VX])
            theta = float(state[IDX_PHI])

            steer = 0.0060 * x_err - 0.040 * vx - 0.40 * theta
            steer = float(np.clip(steer, -0.20, 0.20))

            split1 = max(1, self.N // 3)
            split2 = max(split1 + 1, 2 * self.N // 3)

            U[:split1, 0] = steer * params.delta_max_cmd
            U[split1:split2, 0] = -0.35 * steer * params.delta_max_cmd
            U[split2:, 0] = -0.12 * steer * params.delta_max_cmd

            U[:, 1] = np.clip(1.16 * params.F_hover, params.F_min, params.F_max)
            return U

        # Descent seed: first correct the horizontal error, then unload/straighten
        # near the predicted touchdown.  This avoids the zero-gimbal local minimum.
        x_err = self.mission.x_land - float(state[IDX_X])
        vx = float(state[IDX_VX])
        vy = float(state[IDX_VY])
        theta = float(state[IDX_PHI])

        # Desired sign: positive gimbal produces positive pitch acceleration,
        # positive pitch gives positive horizontal acceleration.
        # Gimbal authority is strong for this vehicle; a few degrees sustained
        # over seconds are enough to build a large pitch angle.
        steer = 0.0060 * x_err - 0.045 * vx - 0.45 * theta
        steer = float(np.clip(steer, -0.18, 0.18))
        split1 = max(1, self.N // 3)
        split2 = max(split1 + 1, 2 * self.N // 3)
        U[:split1, 0] = steer * params.delta_max_cmd
        U[split1:split2, 0] = -0.35 * steer * params.delta_max_cmd
        U[split2:, 0] = -0.12 * steer * params.delta_max_cmd

        # Vertical seed: brake if descending, otherwise descend gently.
        bias = 1.0 + 0.055 * max(-vy, 0.0) - 0.025 * max(vy, 0.0)
        y_rel = float(state[IDX_Y]) - self.mission.y_land
        if y_rel < self.landing_zone:
            bias += 0.020 * max(self.landing_zone - y_rel, 0.0)
        U[:, 1] = np.clip(bias * params.F_hover, params.F_min, params.F_max)
        return U

    def update_phase(self, state: np.ndarray) -> None:
        if self.phase == "ascent":
            x = float(state[IDX_X])
            y = float(state[IDX_Y])
            vy = float(state[IDX_VY])

            height_ok = y >= self.mission.h_target - self.mission.eps_h
            vertical_ok = abs(vy) <= self.mission.eps_v or (
                y >= self.mission.h_target and vy <= 0.0
            )
            x_ok = abs(x - self.mission.x_mid) <= self.mission.eps_x

            if height_ok and vertical_ok and x_ok:
                self.phase = "descent"
                self.warm_u = None

    def touchdown_quality(self, state: np.ndarray) -> Dict[str, float | bool]:
        vx = float(state[IDX_VX])
        vy = float(state[IDX_VY])
        theta = abs(wrap_angle(float(state[IDX_PHI])))
        omega = abs(float(state[IDX_OMEGA]))
        speed = math.hypot(vx, vy)
        x_err = abs(float(state[IDX_X]) - self.mission.x_land)
        y_err = abs(float(state[IDX_Y]) - self.mission.y_land)
        return {
            "x_error": x_err,
            "y_error": y_err,
            "vx_abs": abs(vx),
            "vy_abs": abs(vy),
            "speed": speed,
            "theta_abs": theta,
            "omega_abs": omega,
            "success": (
                self.phase == "descent"
                and y_err <= self.mission.eps_land
                and x_err <= self.mission.eps_x
                and abs(vx) <= self.mission.eps_vx
                and abs(vy) <= self.mission.eps_vy
                and speed <= self.mission.eps_vel
                and theta <= self.mission.eps_theta
                and omega <= self.mission.eps_omega
            ),
        }

    def mission_done(self, state: np.ndarray) -> bool:
        return bool(self.touchdown_quality(state)["success"])

    def _target_and_weights(self) -> Tuple[np.ndarray, np.ndarray]:
        target = np.zeros(STATE_DIM)
        p = np.zeros(STATE_DIM)
        if self.phase == "ascent":
            target[IDX_X] = self.mission.x_mid
            target[IDX_Y] = self.mission.h_target

            p[IDX_X] = self.weights.px
            p[IDX_Y] = self.weights.py
            p[IDX_VX] = 0.0
            p[IDX_VY] = self.weights.pvy
            p[IDX_PHI] = self.weights.ptheta
            p[IDX_OMEGA] = self.weights.pomega
            # x, vx, delta are intentionally free in ascent.
        else:
            target[IDX_X] = self.mission.x_land
            target[IDX_Y] = self.mission.y_land
            p[IDX_X] = self.weights.px
            p[IDX_Y] = self.weights.py
            p[IDX_VX] = self.weights.pvx
            p[IDX_VY] = self.weights.pvy
            p[IDX_PHI] = self.weights.ptheta
            p[IDX_OMEGA] = self.weights.pomega
            p[IDX_DELTA] = self.weights.pdelta
        return target, p

    def _normalized_error(self, state: np.ndarray, target: np.ndarray) -> np.ndarray:
        scales = np.array([
            max(self.scales.x, 1e-9),
            max(self.scales.y, 1e-9),
            max(self.scales.vx, 1e-9),
            max(self.scales.vy, 1e-9),
            max(self.scales.theta, 1e-9),
            max(self.scales.omega, 1e-9),
            max(self.scales.delta, 1e-9),
        ])
        err = np.asarray(state, dtype=float) - np.asarray(target, dtype=float)
        err[IDX_PHI] = wrap_angle(err[IDX_PHI])
        return err / scales

    def _landing_soft_cost(self, state: np.ndarray) -> float:
        if self.phase != "descent":
            return 0.0

        y = float(state[IDX_Y])
        y_rel = y - self.mission.y_land
        w = self.weights
        zone = float(np.clip((self.landing_zone - y_rel) / max(self.landing_zone, 1e-9), 0.0, 1.0))
        if zone <= 0.0:
            return 0.0

        target = np.zeros(STATE_DIM)
        target[IDX_X] = self.mission.x_land
        target[IDX_Y] = self.mission.y_land
        err = self._normalized_error(state, target)
        # The closer the predicted vehicle is to the final point, the more it
        # should resemble the requested terminal state.  For y_land=0 this is
        # a normal landing constraint; for y_land>0 it becomes a hover/final-point
        # constraint at a selected altitude.
        cost = w.near_ground * zone**2 * (
            0.9 * err[IDX_X] ** 2
            + 0.6 * err[IDX_Y] ** 2
            + 0.9 * err[IDX_VX] ** 2
            + 1.2 * err[IDX_VY] ** 2
            + 1.0 * err[IDX_PHI] ** 2
            + 0.35 * err[IDX_OMEGA] ** 2
        )
        if abs(y - self.mission.y_land) <= self.mission.eps_land + 1.0:
            cost += w.touchdown_x * err[IDX_X] ** 2
            cost += w.touchdown_v * (err[IDX_VX] ** 2 + err[IDX_VY] ** 2)
            cost += w.touchdown_theta * err[IDX_PHI] ** 2
            cost += w.touchdown_omega * err[IDX_OMEGA] ** 2
        return float(cost)

    def _objective(self, z: np.ndarray, state0: np.ndarray, params: RocketParams) -> float:
        U = self._z_to_control(z, params)
        target, p = self._target_and_weights()
        w = self.weights
        s = np.asarray(state0, dtype=float).copy()
        cost = 0.0
        prev_u = None
        first_contact_cost_added = False

        for k in range(self.N):
            dcmd, F = float(U[k, 0]), float(U[k, 1])

            # Stage cost: nominally only control usage, not state tracking.
            cost += w.R_delta * (dcmd / max(params.delta_max_cmd, 1e-9)) ** 2
            cost += w.R_F * ((F - params.F_hover) / max(self.scales.F, 1e-9)) ** 2

            if prev_u is not None:
                cost += w.R_delta_rate * ((dcmd - prev_u[0]) / max(params.delta_max_cmd, 1e-9)) ** 2
                cost += w.R_F_rate * ((F - prev_u[1]) / max(self.scales.F, 1e-9)) ** 2
            prev_u = (dcmd, F)

            try:
                s = rk4_step(s, U[k], self.dt, params)
            except (FloatingPointError, OverflowError, ValueError):
                return 1.0e30
            if (not np.all(np.isfinite(s))) or abs(float(s[IDX_X])) > 1.0e5 or abs(float(s[IDX_Y])) > 1.0e5 or math.hypot(float(s[IDX_VX]), float(s[IDX_VY])) > 3.0e3:
                return 1.0e30

            # Soft path constraints; these are not tracking terms.
            y_violation = max(0.0, -float(s[IDX_Y])) / max(self.scales.y, 1e-9)
            theta_violation = max(0.0, abs(float(s[IDX_PHI])) - params.theta_max) / max(self.scales.theta, 1e-9)
            delta_violation = max(0.0, abs(float(s[IDX_DELTA])) - params.delta_max) / max(self.scales.delta, 1e-9)
            cost += w.ground * y_violation**2
            if y_violation > 0.0:
                cost += 6.0 * w.ground * (float(s[IDX_VY]) / max(self.scales.vy, 1e-9)) ** 2
            cost += w.theta_limit * theta_violation**2
            cost += w.delta_limit * delta_violation**2

            cost += self._landing_soft_cost(s)
            if (
                self.phase == "descent"
                and abs(float(s[IDX_Y]) - self.mission.y_land) <= self.mission.eps_land
                and not first_contact_cost_added
            ):
                # First predicted arrival near the final altitude receives an
                # extra terminal quality penalty.
                cost += 2.0 * self._landing_soft_cost(s)
                first_contact_cost_added = True

        err = self._normalized_error(s, target)
        cost += float(np.sum(p * err**2))
        if self.phase == "descent":
            # Make the terminal landing state dominant in descent, especially if
            # the horizon is short relative to the mission.
            cost += 0.8 * self._landing_soft_cost(s)
        return float(cost)

    def _candidate_warm_starts(self, state: np.ndarray, params: RocketParams) -> list[np.ndarray]:
        base = self.warm_u.copy() if self.warm_u is not None and self.warm_u.shape == (self.N, 2) else self._default_warm_start(state, params)
        candidates = [base, self._default_warm_start(state, params)]

        # Pure vertical braking/hover candidate.
        vertical = np.zeros((self.N, 2), dtype=float)
        vy = float(state[IDX_VY])
        if self.phase == "descent":
            f = params.F_hover * (1.0 + 0.060 * max(-vy, 0.0) - 0.025 * max(vy, 0.0))
        else:
            f = 1.15 * params.F_hover
        vertical[:, 1] = np.clip(f, params.F_min, params.F_max)
        candidates.append(vertical)

        if self.phase == "descent":
            # Opposite-turn candidate in case the current lateral seed is wrong.
            opp = candidates[1].copy()
            opp[:, 0] *= -1.0
            candidates.append(opp)

            # Strong vertical braking candidate for near-ground situations.
            brake = np.zeros((self.N, 2), dtype=float)
            vy = float(state[IDX_VY])
            brake[:, 1] = params.F_max if vy < -1.0 else params.F_hover
            candidates.append(brake)

            # Lateral braking candidate: aim mostly against horizontal velocity.
            lat_brake = np.zeros((self.N, 2), dtype=float)
            sign = -1.0 if float(state[IDX_VX]) > 0.0 else 1.0
            lat_brake[: max(1, self.N // 2), 0] = 0.16 * sign * params.delta_max_cmd
            lat_brake[max(1, self.N // 2) :, 0] = -0.10 * sign * params.delta_max_cmd
            lat_brake[:, 1] = np.clip(params.F_hover * (1.0 + 0.05 * max(-float(state[IDX_VY]), 0.0)), params.F_min, params.F_max)
            candidates.append(lat_brake)

            # Balanced pulse train: direct lateral impulse from nozzle angle,
            # followed by opposite pulses to keep pitch small.
            x_err = self.mission.x_land - float(state[IDX_X])
            pulse = np.zeros((self.N, 2), dtype=float)
            pulse[:, 1] = np.clip(params.F_hover * (1.0 + 0.035 * max(-float(state[IDX_VY]), 0.0)), params.F_min, params.F_max)
            pulse_sign = 1.0 if x_err >= 0.0 else -1.0
            amp = np.clip(0.03 + 0.0025 * abs(x_err), 0.04, 0.28) * params.delta_max_cmd
            for kk in range(self.N):
                if kk % 4 == 0:
                    pulse[kk, 0] = pulse_sign * amp
                elif kk % 4 == 1:
                    pulse[kk, 0] = -pulse_sign * 0.85 * amp
                elif kk % 4 == 2:
                    pulse[kk, 0] = pulse_sign * 0.45 * amp
                else:
                    pulse[kk, 0] = -pulse_sign * 0.45 * amp
            candidates.append(pulse)

        return candidates

    def predict(self, state: np.ndarray, U: np.ndarray, params: RocketParams) -> np.ndarray:
        X = np.zeros((len(U) + 1, STATE_DIM), dtype=float)
        X[0] = np.asarray(state, dtype=float)
        for k, u in enumerate(U):
            X[k + 1] = rk4_step(X[k], u, self.dt, params)
        return X


    def _coordinate_search(self, z0: np.ndarray, state: np.ndarray, params: RocketParams) -> Tuple[np.ndarray, float, int]:
        """Small deterministic bound-constrained search.

        It is less elegant than IPOPT/CasADi, but for this educational project it
        is robust: the number of objective evaluations is bounded, so a bad
        line-search cannot freeze the simulation.
        """
        z = np.clip(np.asarray(z0, dtype=float).copy(), -1.0, 1.0)
        best = self._objective(z, state, params)
        n_eval = 1
        # Optimize thrust blocks first, then gimbal blocks.  Thrust usually has a
        # stronger immediate effect on touchdown speed.
        # Optimize thrust first, then gimbal pulses.
        order = list(range(self.gimbal_blocks, len(z))) + list(range(0, self.gimbal_blocks))
        step = float(self.initial_step)
        for _pass in range(max(1, self.search_passes)):
            improved_this_pass = False
            for i in order:
                current_best = best
                best_candidate = z
                for direction in (1.0, -1.0):
                    z_try = z.copy()
                    z_try[i] = np.clip(z_try[i] + direction * step, -1.0, 1.0)
                    if z_try[i] == z[i]:
                        continue
                    c = self._objective(z_try, state, params)
                    n_eval += 1
                    if c + self.ftol < current_best:
                        current_best = c
                        best_candidate = z_try
                if current_best + self.ftol < best:
                    z = best_candidate
                    best = current_best
                    improved_this_pass = True
            step *= 0.55 if improved_this_pass else 0.35
        return z, float(best), n_eval


    def _apply_terminal_assist(self, u0: np.ndarray, state: np.ndarray, params: RocketParams) -> np.ndarray:
        """Terminal landing stabilizer used by the improved hybrid mode.

        The nonlinear MPC is still available and is used for the ascent phase.
        In descent, the educational single-shooting MPC can be too myopic for a
        clean touchdown, so this optional layer behaves like a terminal guidance
        law: it regulates x, vx, y, vy, theta and omega to the landing state.
        Set ``assist_override_descent: false`` in the config to run pure MPC.
        """
        if (not self.landing_assist) or self.phase != "descent":
            return u0
        y = float(state[IDX_Y])
        y_rel = y - self.mission.y_land
        if y_rel > self.assist_altitude:
            return u0

        x = float(state[IDX_X])
        vx = float(state[IDX_VX])
        vy = float(state[IDX_VY])
        theta = wrap_angle(float(state[IDX_PHI]))
        omega = float(state[IDX_OMEGA])
        delta = float(state[IDX_DELTA])
        x_err = self.mission.x_land - x

        # Horizontal guidance: choose a modest desired lateral velocity and use
        # the gimbal for direct lateral acceleration, while damping pitch.
        vx_des = float(np.clip(0.35 * x_err, -5.0, 5.0))
        ax_cmd = float(np.clip(0.55 * (vx_des - vx), -2.0, 2.0))

        # Vertical guidance: drive the rocket to the requested final altitude.
        # If y_land=0 this behaves like landing guidance.  If y_land>0, the
        # terminal point is an in-air hover target.
        y_err = self.mission.y_land - y
        vy_des = float(np.clip(0.30 * y_err, -7.0, 3.0))
        if abs(x_err) > 15.0:
            # Do not descend too aggressively while still far from the target x.
            vy_des = max(vy_des, -2.0)
        ay_cmd = float(np.clip(1.20 * (vy_des - vy), -5.0, 5.0))

        # Convert acceleration requests to thrust and nozzle command.  The pitch
        # damping terms are deliberately stronger than the lateral term near the
        # ground so touchdown attitude remains close to vertical.
        F_nom = params.mass * (params.g + ay_cmd)
        denom = max(math.cos(theta) - delta * math.sin(theta), 0.50)
        F_assist = float(np.clip(F_nom / denom, params.F_min, params.F_max))
        dcmd_assist = params.mass * ax_cmd / max(F_assist, 1.0) - 0.65 * theta - 0.25 * omega
        dcmd_assist = float(np.clip(dcmd_assist, -params.delta_max_cmd, params.delta_max_cmd))

        blend = float(np.clip(self.assist_blend, 0.0, 1.0))
        out = np.asarray(u0, dtype=float).copy()
        out[0] = (1.0 - blend) * out[0] + blend * dcmd_assist
        out[1] = (1.0 - blend) * out[1] + blend * F_assist
        out[0] = float(np.clip(out[0], -params.delta_max_cmd, params.delta_max_cmd))
        out[1] = float(np.clip(out[1], params.F_min, params.F_max))
        return out


    def _apply_ascent_waypoint_assist(self, state: np.ndarray, params: RocketParams) -> np.ndarray:
        x = float(state[IDX_X])
        y = float(state[IDX_Y])
        vx = float(state[IDX_VX])
        vy = float(state[IDX_VY])
        theta = wrap_angle(float(state[IDX_PHI]))
        omega = float(state[IDX_OMEGA])
        delta = float(state[IDX_DELTA])

        x_err = self.mission.x_mid - x
        y_err = self.mission.h_target - y

        vx_des = float(np.clip(0.28 * x_err, -8.0, 8.0))

        if abs(x_err) < 1.8 * self.mission.eps_x:
            vx_des = float(np.clip(0.18 * x_err, -2.0, 2.0))

        ax_cmd = float(np.clip(0.45 * (vx_des - vx), -2.3, 2.3))

        if y_err > 0.0:
            vy_des = float(np.clip(0.35 * y_err, 0.5, 10.0))
        else:
            vy_des = 0.0

        ay_cmd = float(np.clip(0.65 * (vy_des - vy), -5.0, 5.0))

        F_nom = params.mass * (params.g + ay_cmd)
        denom = max(math.cos(theta) - delta * math.sin(theta), 0.55)
        F_cmd = float(np.clip(F_nom / denom, params.F_min, params.F_max))

        dcmd = params.mass * ax_cmd / max(F_cmd, 1.0) - 0.75 * theta - 0.30 * omega
        dcmd = float(np.clip(dcmd, -params.delta_max_cmd, params.delta_max_cmd))

        return np.array([dcmd, F_cmd], dtype=float)


    def compute_control(self, state: np.ndarray, params: RocketParams) -> Tuple[Dict[str, float], Dict]:
        self.update_phase(state)
        if (
            self.landing_assist
            and self.phase == "ascent"
            and abs(self.mission.x_mid - self.mission.x_start) > 1e-6
        ):
            u0 = self._apply_ascent_waypoint_assist(state, params)
            aero = aero_forces_and_moment(np.asarray(state, dtype=float), params)
            target, p = self._target_and_weights()
            X_pred = self.predict(state, np.tile(u0, (self.N, 1)), params)

            cache = {
                "phase": self.phase,
                "success": True,
                "status": 0,
                "message": "ascent waypoint assist",
                "cost": 0.0,
                "nit": 0,
                "candidate_cost": 0.0,
                "target": target,
                "P_diag": p,
                "predicted_state": X_pred,
                "predicted_control": np.tile(u0, (self.N, 1)),
                **aero,
            }

            self.last_cache = cache

            return {
                "delta_cmd": float(u0[0]),
                "F": float(u0[1]),
                "sigma": float(np.clip(u0[1] / max(params.F_max, 1e-9), 0.0, 1.0)),
            }, cache
        if (
            self.landing_assist
            and self.assist_override_descent
            and self.phase == "descent"
            and float(state[IDX_Y]) - self.mission.y_land <= self.assist_altitude
        ):
            u0 = self._apply_terminal_assist(np.array([0.0, params.F_hover], dtype=float), state, params)
            aero = aero_forces_and_moment(np.asarray(state, dtype=float), params)
            target, p = self._target_and_weights()
            X_pred = self.predict(state, np.tile(u0, (self.N, 1)), params)
            cache = {
                "phase": self.phase,
                "success": True,
                "status": 0,
                "message": "terminal landing assist",
                "cost": 0.0,
                "nit": 0,
                "candidate_cost": 0.0,
                "target": target,
                "P_diag": p,
                "predicted_state": X_pred,
                "predicted_control": np.tile(u0, (self.N, 1)),
                **aero,
            }
            self.last_cache = cache
            return {"delta_cmd": float(u0[0]), "F": float(u0[1]), "sigma": float(np.clip(u0[1] / max(params.F_max, 1e-9), 0.0, 1.0))}, cache

        if self.warm_u is None or self.warm_u.shape != (self.N, 2):
            self.warm_u = self._default_warm_start(state, params)

        candidates = self._candidate_warm_starts(state, params)
        candidate_costs = [self._objective(self._control_to_z(Ucand, params), state, params) for Ucand in candidates]
        best_idx = int(np.argmin(candidate_costs))
        best_U = candidates[best_idx]
        best_cost = float(candidate_costs[best_idx])
        z0 = self._control_to_z(best_U, params)
        bounds = [(-1.0, 1.0)] * len(z0)

        if self.optimizer == "scipy":
            result = minimize(
                fun=lambda z: self._objective(z, state, params),
                x0=z0,
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": self.max_iter, "maxfun": max(30, 4 * len(z0)), "ftol": self.ftol, "maxls": 8},
            )
            z_opt = result.x
            opt_cost = float(result.fun) if result.fun is not None else float("inf")
            nit = int(getattr(result, "nit", 0))
            raw_success = bool(result.success)
            status = int(getattr(result, "status", -1))
            message = str(getattr(result, "message", ""))
        else:
            z_opt, opt_cost, n_eval = self._coordinate_search(z0, state, params)
            nit = n_eval
            raw_success = np.isfinite(opt_cost)
            status = 0 if raw_success else 1
            message = "coordinate search"

        optimized_ok = bool(raw_success) and np.all(np.isfinite(z_opt)) and np.isfinite(opt_cost) and opt_cost < best_cost
        if optimized_ok:
            U = self._z_to_control(z_opt, params)
            selected_cost = float(opt_cost)
        else:
            U = best_U.copy()
            selected_cost = best_cost

        try:
            X_pred = self.predict(state, U, params)
        except (FloatingPointError, OverflowError, ValueError):
            U = self._default_warm_start(state, params)
            X_pred = self.predict(state, U, params)
            selected_cost = self._objective(self._control_to_z(U, params), state, params)

        u0 = self._apply_terminal_assist(U[0].copy(), state, params)

        # Warm-start the next solve by shifting the expanded optimal sequence.
        self.warm_u = np.vstack([U[1:], U[-1:]])

        aero = aero_forces_and_moment(np.asarray(state, dtype=float), params)
        target, p = self._target_and_weights()
        cache = {
            "phase": self.phase,
            "success": bool(optimized_ok),
            "status": status,
            "message": message,
            "cost": float(selected_cost),
            "nit": nit,
            "candidate_cost": best_cost,
            "target": target,
            "P_diag": p,
            "predicted_state": X_pred,
            "predicted_control": U,
            **aero,
        }
        self.last_cache = cache
        control = {
            "delta_cmd": float(u0[0]),
            "F": float(u0[1]),
            "sigma": float(np.clip(u0[1] / max(params.F_max, 1e-9), 0.0, 1.0)),
        }
        return control, cache
