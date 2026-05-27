from __future__ import annotations

import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimulationResult
from .system import IDX_DELTA, IDX_OMEGA, IDX_PHI, IDX_VX, IDX_VY, IDX_X, IDX_Y

PALETTE = {
    "bg": "#0d0f1a",
    "panel": "#121629",
    "grid": "#242b45",
    "tick": "#6f7aa8",
    "title": "#d7dff7",
    "label": "#a9b4da",
    "body": "#d9dde7",
    "body_edge": "#f6f7fb",
    "stripe": "#d53c33",
    "window": "#7fc8ff",
    "fin": "#9aa3ba",
    "nozzle": "#6e768a",
    "flame_outer": "#ff5a36",
    "flame_mid": "#ffb703",
    "flame_core": "#fff2a8",
    "trail": "#79a8ff",
    "vertical": "#5ef38c",
    "theta": "#5aa0ff",
    "omega": "#ffb14e",
    "delta": "#cf9fff",
    "sigma": "#ef476f",
    "speed": "#61d095",
    "ground": "#1b3a1f",
    "ground_line": "#46a758",
    "text": "#f2f5ff",
}

def _mission_markers(result: SimulationResult):
    mission = result.config.get("mission", {})

    x_start = float(mission.get("x_start", 0.0))
    x_land, y_land = _final_target(result)
    _, h_target = _targets(result)

    x_mid_raw = mission.get("x_mid", None)
    x_mid = float(x_mid_raw) if x_mid_raw is not None else 0.5 * (x_start + x_land)

    final_label = "LAND" if abs(y_land) < 1e-9 else "FINAL"

    return [
        ("START", x_start, 0.0, "o", PALETTE["omega"]),
        ("TARGET ALT", x_mid, h_target, "D", PALETTE["vertical"]),
        (final_label, x_land, y_land, "X", PALETTE["ground_line"]),
    ]


def _draw_mission_markers(ax, markers):
    if not markers:
        return

    xs = [p[1] for p in markers]
    ys = [p[2] for p in markers]

    ax.plot(
        xs,
        ys,
        "--",
        color=PALETTE["vertical"],
        lw=1.1,
        alpha=0.45,
        zorder=2,
        label="mission points",
    )

    for label, px, py, marker, color in markers:
        ax.scatter(
            [px],
            [py],
            marker=marker,
            s=85,
            color=color,
            edgecolors=PALETTE["body_edge"],
            linewidths=0.8,
            zorder=20,
        )
        ax.text(
            px,
            py + 2.0,
            label,
            color=PALETTE["text"],
            fontsize=7.5,
            ha="center",
            va="bottom",
            bbox=dict(
                boxstyle="round,pad=0.25",
                fc=PALETTE["bg"],
                ec=color,
                alpha=0.82,
            ),
            zorder=21,
        )


def _mission_view_limits(result: SimulationResult, mission_markers):
    state = result.state

    marker_x = [p[1] for p in mission_markers]
    marker_y = [p[2] for p in mission_markers]

    x_min = min(float(state[:, IDX_X].min()), min(marker_x)) - 12.0
    x_max = max(float(state[:, IDX_X].max()), max(marker_x)) + 12.0
    y_min = min(-5.0, float(state[:, IDX_Y].min()) - 5.0, min(marker_y) - 5.0)
    y_max = max(float(state[:, IDX_Y].max()), max(marker_y)) + 15.0

    return (x_min, x_max, y_min, y_max)

def _style_ax(ax):
    ax.set_facecolor(PALETTE["panel"])
    ax.grid(color=PALETTE["grid"], linewidth=0.6, alpha=0.7)
    ax.tick_params(colors=PALETTE["tick"], labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE["grid"])


def rot2d(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s], [-s, c]])


def body_to_world(pts: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    return (rot2d(theta) @ pts.T).T + np.array([x, y])


def _targets(result: SimulationResult) -> tuple[float, float]:
    mission = result.config.get("mission", {})
    return float(mission.get("x_land", mission.get("xland", 0.0))), float(mission.get("h_target", mission.get("htarget", 0.0)))


def _final_target(result: SimulationResult) -> tuple[float, float]:
    mission = result.config.get("mission", {})
    x_land = float(mission.get("x_land", mission.get("xland", 0.0)))
    y_land = float(mission.get("y_land", mission.get("yland", 0.0)))
    return x_land, y_land


def draw_rocket(
    ax,
    x,
    y,
    theta,
    delta,
    throttle,
    trail_x,
    trail_y,
    vw=20.0,
    vh=24.0,
    title="MPC mission",
    mission_markers=None,
    view_limits=None,
):
    ax.clear()
    _style_ax(ax)
    if view_limits is None:
        ax.set_xlim(x - vw, x + vw)
        ax.set_ylim(max(y - vh, -2.0), y + vh)
        ground_x_min, ground_x_max = x - 500, x + 500
    else:
        ax.set_xlim(view_limits[0], view_limits[1])
        ax.set_ylim(view_limits[2], view_limits[3])
        ground_x_min, ground_x_max = view_limits[0], view_limits[1]
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]", color=PALETTE["label"], fontsize=8)
    ax.set_ylabel("y [m]", color=PALETTE["label"], fontsize=8)
    ax.set_title(title, color=PALETTE["title"], fontsize=9, pad=6)
    ax.axhline(0.0, color=PALETTE["ground_line"], lw=1.6, zorder=0)
    ax.fill_between([ground_x_min, ground_x_max], -100, 0.0, color=PALETTE["ground"], alpha=0.6, zorder=0)
    ax.plot(trail_x, trail_y, color=PALETTE["trail"], lw=1.6, alpha=0.85, zorder=1)
    _draw_mission_markers(ax, mission_markers)

    bh, bw = 4.2, 0.9
    hh, hw = bh / 2, bw / 2
    body_pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh * 0.64], [0, hh], [-hw, hh * 0.64]])
    stripe_pts = np.array([[-hw * 0.85, -0.25], [hw * 0.85, -0.25], [hw * 0.85, 0.30], [-hw * 0.85, 0.30]])
    window_pts = np.array([[-0.20, 0.78], [0.20, 0.78], [0.25, 1.15], [0.0, 1.35], [-0.25, 1.15]])
    fin_l = np.array([[-hw, -0.85], [-hw - 0.45, -1.45], [-hw * 0.25, -1.25]])
    fin_r = fin_l.copy(); fin_r[:, 0] *= -1
    nozzle_pts = np.array([[-0.23, -hh], [0.23, -hh], [0.13, -hh - 0.38], [-0.13, -hh - 0.38]])
    for pts, fc, ec, lw, zo in [
        (body_pts, PALETTE["body"], PALETTE["body_edge"], 1.6, 7),
        (stripe_pts, PALETTE["stripe"], PALETTE["stripe"], 1.0, 8),
        (window_pts, PALETTE["window"], PALETTE["body_edge"], 0.9, 8),
        (fin_l, PALETTE["fin"], PALETTE["body_edge"], 1.0, 6),
        (fin_r, PALETTE["fin"], PALETTE["body_edge"], 1.0, 6),
        (nozzle_pts, PALETTE["nozzle"], PALETTE["body_edge"], 1.0, 6),
    ]:
        ax.add_patch(mpatches.Polygon(body_to_world(pts, x, y, theta), closed=True, facecolor=fc, edgecolor=ec, lw=lw, zorder=zo))

    nozzle_c = body_to_world(np.array([[0.0, -hh - 0.34]]), x, y, theta)[0]
    thrust_angle = theta - delta
    fs = max(float(throttle), 0.02)
    for ln, lw_f, fc in [(3.2 * fs, 0.40, PALETTE["flame_outer"]), (2.2 * fs, 0.27, PALETTE["flame_mid"]), (1.1 * fs, 0.14, PALETTE["flame_core"])]:
        flame_pts = np.array([[-lw_f, 0], [lw_f, 0], [0, -ln]])
        ax.add_patch(mpatches.Polygon(body_to_world(flame_pts, nozzle_c[0], nozzle_c[1], thrust_angle), closed=True, facecolor=fc, edgecolor="none", alpha=0.85, zorder=3))


def save_all_figures(result: SimulationResult, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    t = result.t
    state = result.state
    ctl = result.controls
    drv = result.derived
    x_land, h_target = _targets(result)
    _, y_land = _final_target(result)

    # 1. Position and velocity.
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, facecolor=PALETTE["bg"])
    for ax in axes.flat:
        _style_ax(ax)
    axes[0, 0].plot(t, state[:, IDX_X], color=PALETTE["theta"])
    axes[0, 0].axhline(x_land, ls="--", color=PALETTE["vertical"], lw=1, label="x_land")
    axes[0, 0].set_ylabel("x [m]", color=PALETTE["label"])
    axes[0, 0].set_title("Horizontal position", color=PALETTE["title"], fontsize=9)
    axes[0, 1].plot(t, state[:, IDX_Y], color=PALETTE["omega"])
    axes[0, 1].axhline(h_target, ls=":", color=PALETTE["vertical"], lw=1, label="h_target")
    axes[0, 1].axhline(0, ls="--", color=PALETTE["ground_line"], lw=1)
    axes[0, 1].set_ylabel("y [m]", color=PALETTE["label"])
    axes[0, 1].set_title("Altitude", color=PALETTE["title"], fontsize=9)
    axes[1, 0].plot(t, state[:, IDX_VX], color=PALETTE["theta"])
    axes[1, 0].axhline(0, ls="--", color=PALETTE["grid"], lw=0.8)
    axes[1, 0].set_ylabel("vx [m/s]", color=PALETTE["label"])
    axes[1, 0].set_xlabel("Time [s]", color=PALETTE["label"])
    axes[1, 1].plot(t, state[:, IDX_VY], color=PALETTE["omega"])
    axes[1, 1].axhline(0, ls="--", color=PALETTE["grid"], lw=0.8)
    axes[1, 1].set_ylabel("vy [m/s]", color=PALETTE["label"])
    axes[1, 1].set_xlabel("Time [s]", color=PALETTE["label"])
    fig.suptitle("MPC position and velocity", color=PALETTE["title"], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / "position_velocity.png", dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    # 2. Attitude, nozzle and command.
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, facecolor=PALETTE["bg"])
    for ax in axes:
        _style_ax(ax)
    axes[0].plot(t, np.degrees(state[:, IDX_PHI]), color=PALETTE["theta"], label="theta")
    axes[0].axhline(math.degrees(result.params.theta_max), color=PALETTE["grid"], ls=":", lw=1)
    axes[0].axhline(-math.degrees(result.params.theta_max), color=PALETTE["grid"], ls=":", lw=1)
    axes[0].set_ylabel("theta [deg]", color=PALETTE["label"])
    axes[0].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[1].plot(t, np.degrees(state[:, IDX_OMEGA]), color=PALETTE["omega"], label="omega")
    axes[1].set_ylabel("omega [deg/s]", color=PALETTE["label"])
    axes[1].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[2].plot(t, np.degrees(state[:, IDX_DELTA]), color=PALETTE["delta"], label="delta")
    axes[2].plot(t, np.degrees(ctl["delta_cmd"]), ":", color=PALETTE["sigma"], label="delta_cmd")
    axes[2].axhline(math.degrees(result.params.delta_max), color=PALETTE["grid"], ls=":", lw=1)
    axes[2].axhline(-math.degrees(result.params.delta_max), color=PALETTE["grid"], ls=":", lw=1)
    axes[2].set_ylabel("gimbal [deg]", color=PALETTE["label"])
    axes[2].set_xlabel("Time [s]", color=PALETTE["label"])
    axes[2].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    fig.suptitle("MPC attitude and nozzle", color=PALETTE["title"], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / "attitude_and_gimbal.png", dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    # 3. Control and solver diagnostics.
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, facecolor=PALETTE["bg"])
    for ax in axes:
        _style_ax(ax)
    axes[0].plot(t, ctl["sigma"], color=PALETTE["sigma"], label="sigma=F/Fmax")
    axes[0].axhline(result.params.throttle_hover, ls="--", color=PALETTE["vertical"], lw=1, label="hover")
    axes[0].set_ylabel("throttle", color=PALETTE["label"])
    axes[0].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[1].plot(t, ctl["F"] / 1000.0, color=PALETTE["speed"], label="F")
    axes[1].axhline(result.params.F_hover / 1000.0, ls="--", color=PALETTE["vertical"], lw=1)
    axes[1].set_ylabel("F [kN]", color=PALETTE["label"])
    axes[1].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[2].step(t, ctl["phase"], where="post", color=PALETTE["theta"], label="phase: 0 ascent, 1 descent")
    axes[2].plot(t, drv.get("solver_success", np.ones_like(t)), color=PALETTE["vertical"], alpha=0.7, label="solver success")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_xlabel("Time [s]", color=PALETTE["label"])
    axes[2].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    fig.suptitle("MPC controls and phase", color=PALETTE["title"], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / "control_and_phase.png", dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    # 4. Aerodynamics.
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, facecolor=PALETTE["bg"])
    for ax in axes:
        _style_ax(ax)
    axes[0].plot(t, drv["speed"], color=PALETTE["speed"], label="V")
    axes[0].set_ylabel("V [m/s]", color=PALETTE["label"])
    axes[0].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[1].plot(t, drv.get("alpha_deg", np.zeros_like(t)), color=PALETTE["theta"], label="alpha")
    axes[1].set_ylabel("alpha [deg]", color=PALETTE["label"])
    axes[1].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    axes[2].plot(t, drv.get("X_b", np.zeros_like(t)) / 1000.0, color=PALETTE["omega"], label="X_b")
    axes[2].plot(t, drv.get("Y_b", np.zeros_like(t)) / 1000.0, color=PALETTE["delta"], label="Y_b")
    axes[2].set_ylabel("force [kN]", color=PALETTE["label"])
    axes[2].set_xlabel("Time [s]", color=PALETTE["label"])
    axes[2].legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    fig.suptitle("Aerodynamic quantities", color=PALETTE["title"], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / "aerodynamics.png", dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    # 5. Planar trajectory.
    fig, ax = plt.subplots(figsize=(8, 8), facecolor=PALETTE["bg"])
    _style_ax(ax)
    ax.plot(state[:, IDX_X], state[:, IDX_Y], color=PALETTE["trail"], lw=2.0, label="trajectory")
    ax.scatter([state[0, IDX_X]], [state[0, IDX_Y]], s=60, color=PALETTE["omega"], zorder=5, label="start")
    ax.scatter([x_land], [y_land], marker="X", s=90, color=PALETTE["ground_line"], zorder=5, label="final target")
    ax.axhline(h_target, color=PALETTE["vertical"], ls=":", lw=1.2, label="target altitude")
    if abs(y_land) > 1e-9:
        ax.axhline(y_land, color=PALETTE["ground_line"], ls="--", lw=1.0, alpha=0.75, label="final altitude")
    ax.axhline(0, color=PALETTE["ground_line"], lw=1.4)
    x_min = min(float(state[:, IDX_X].min()), x_land) - 10
    x_max = max(float(state[:, IDX_X].max()), x_land) + 10
    ax.fill_between([x_min, x_max], -20, 0, color=PALETTE["ground"], alpha=0.5, zorder=0)
    ax.set_title("Planar MPC trajectory", color=PALETTE["title"], fontsize=10)
    ax.set_xlabel("x [m]", color=PALETTE["label"])
    ax.set_ylabel("y [m]", color=PALETTE["label"])
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8, facecolor=PALETTE["panel"], labelcolor=PALETTE["label"])
    fig.tight_layout()
    fig.savefig(output_dir / "planar_trajectory.png", dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_preview_figure(result: SimulationResult, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    idx = min(len(result.t) - 1, max(1, len(result.t) // 3))
    mission_markers = _mission_markers(result)
    view_limits = _mission_view_limits(result, mission_markers)
    fig = plt.figure(figsize=(7.4, 8.2), facecolor=PALETTE["bg"])
    ax = fig.add_subplot(111)
    draw_rocket(
        ax,
        x=float(result.state[idx, IDX_X]),
        y=float(result.state[idx, IDX_Y]),
        theta=float(result.state[idx, IDX_PHI]),
        delta=float(result.state[idx, IDX_DELTA]),
        throttle=float(result.controls["sigma"][idx]),
        trail_x=result.state[: idx + 1, IDX_X],
        trail_y=result.state[: idx + 1, IDX_Y],
        title="MPC rocket mission",
        mission_markers=mission_markers,
        view_limits=view_limits,
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_animation(result: SimulationResult, output_path: Path) -> None:
    output_path = output_path.with_suffix(".gif")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = result.t
    state = result.state
    ctl = result.controls

    mission_markers = _mission_markers(result)
    view_limits = _mission_view_limits(result, mission_markers)

    sample_dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.1
    target_fps = 10
    stride = max(1, int(round(1.0 / (target_fps * sample_dt))))
    frames = list(range(0, len(t), stride))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)

    import matplotlib
    matplotlib.use("Agg")

    fig = plt.figure(figsize=(7.2, 8.0), facecolor=PALETTE["bg"])
    ax = fig.add_subplot(111)

    def update(frame_num):
        idx = frames[frame_num]
        draw_rocket(
            ax,
            x=float(state[idx, IDX_X]),
            y=float(state[idx, IDX_Y]),
            theta=float(state[idx, IDX_PHI]),
            delta=float(state[idx, IDX_DELTA]),
            throttle=float(ctl["sigma"][idx]),
            trail_x=state[: idx + 1, IDX_X],
            trail_y=state[: idx + 1, IDX_Y],
            title="MPC rocket mission",
            mission_markers=mission_markers,
            view_limits=view_limits,
        )
        phase = "ascent" if ctl["phase"][idx] < 0.5 else "descent"
        info = (
            f"t      = {t[idx]:5.2f} s\n"
            f"phase  = {phase}\n"
            f"x      = {state[idx, IDX_X]:+7.2f} m\n"
            f"y      = {state[idx, IDX_Y]:+7.2f} m\n"
            f"theta  = {math.degrees(state[idx, IDX_PHI]):+6.2f} deg\n"
            f"delta  = {math.degrees(state[idx, IDX_DELTA]):+5.2f} deg\n"
            f"sigma  = {ctl['sigma'][idx]:.3f}"
        )
        ax.text(0.02, 0.98, info, transform=ax.transAxes, va="top", color=PALETTE["text"], fontsize=8.5, fontfamily="monospace", bbox=dict(boxstyle="round,pad=0.35", fc=PALETTE["bg"], ec=PALETTE["grid"], alpha=0.90))
        fig.suptitle("Planar TVC Rocket — MPC", color=PALETTE["text"], fontsize=11)

    try:
        anim = animation.FuncAnimation(fig, update, frames=len(frames), interval=1000 // target_fps)
        anim.save(str(output_path), writer=animation.PillowWriter(fps=target_fps), dpi=80)
    finally:
        plt.close(fig)
