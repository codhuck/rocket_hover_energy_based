from __future__ import annotations

import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimulationResult
from .system import IDX_X, IDX_Y, IDX_VX, IDX_VY, IDX_PHI, IDX_OMEGA, IDX_DELTA

PALETTE = {
    'bg':           '#0d0f1a',
    'panel':        '#121629',
    'grid':         '#242b45',
    'tick':         '#6f7aa8',
    'title':        '#d7dff7',
    'label':        '#a9b4da',
    'body':         '#d9dde7',
    'body_edge':    '#f6f7fb',
    'stripe':       '#d53c33',
    'window':       '#7fc8ff',
    'fin':          '#9aa3ba',
    'nozzle':       '#6e768a',
    'flame_outer':  '#ff5a36',
    'flame_mid':    '#ffb703',
    'flame_core':   '#fff2a8',
    'trail':        '#79a8ff',
    'vertical':     '#5ef38c',
    'crosshair':    '#ffffff',
    'theta':        '#5aa0ff',
    'omega':        '#ffb14e',
    'delta':        '#cf9fff',
    'accel':        '#61d095',
    'text':         '#f2f5ff',
    'ground':       '#1b3a1f',
    'ground_line':  '#46a758',
    'lyap':         '#ffd166',
    'z_phi':        '#5aa0ff',
    'z_omega':      '#ffb14e',
    'z_delta':      '#cf9fff',
    'sigma':        '#ef476f',
    'speed':        '#61d095',
}


def _ctrl_label(result: "SimulationResult") -> str:
    return result.config.get("controller", {}).get("type", "backstepping")


def _style_ax(ax):
    ax.set_facecolor(PALETTE['panel'])
    ax.grid(color=PALETTE['grid'], linewidth=0.6, alpha=0.7)
    ax.tick_params(colors=PALETTE['tick'], labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE['grid'])


def rot2d(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s], [-s, c]])


def body_to_world(pts: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    return (rot2d(theta) @ pts.T).T + np.array([x, y])


def draw_rocket(ax, x, y, theta, delta, throttle, trail_x, trail_y,
                vw=5.0, vh=6.5, title="Backstepping landing"):
    ax.clear()
    ax.set_facecolor(PALETTE['panel'])
    ax.grid(color=PALETTE['grid'], linewidth=0.6, alpha=0.7)
    ax.tick_params(colors=PALETTE['tick'], labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE['grid'])

    ax.set_xlim(x - vw, x + vw)
    ax.set_ylim(max(y - vh, -1.0), y + vh)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]', color=PALETTE['label'], fontsize=8)
    ax.set_ylabel('y [m]', color=PALETTE['label'], fontsize=8)
    ax.set_title(title, color=PALETTE['title'], fontsize=9, pad=6)

    ax.axhline(0.0, color=PALETTE['ground_line'], lw=1.6, zorder=0)
    ax.fill_between([x - 200, x + 200], -100, 0.0,
                    color=PALETTE['ground'], alpha=0.6, zorder=0)
    ax.plot(trail_x, trail_y, color=PALETTE['trail'], lw=1.6, alpha=0.85, zorder=1)

    bh, bw = 1.55, 0.34
    hh, hw = bh / 2, bw / 2
    body_pts = np.array([
        [-hw, -hh], [hw, -hh], [hw, hh * 0.64], [0, hh], [-hw, hh * 0.64]
    ])
    stripe_pts = np.array([
        [-hw * 0.85, -0.10], [hw * 0.85, -0.10],
        [hw * 0.85,   0.16], [-hw * 0.85, 0.16]
    ])
    window_pts = np.array([
        [-0.08, 0.28], [0.08, 0.28], [0.10, 0.48], [0.0, 0.58], [-0.10, 0.48]
    ])
    fin_l = np.array([[-hw, -0.32], [-hw - 0.18, -0.60], [-hw * 0.25, -0.52]])
    fin_r = fin_l.copy(); fin_r[:, 0] *= -1
    nozzle_pts = np.array([
        [-0.09, -hh], [0.09, -hh], [0.05, -hh - 0.16], [-0.05, -hh - 0.16]
    ])
    for pts, fc, ec, lw, zo in [
        (body_pts,   PALETTE['body'],   PALETTE['body_edge'], 1.6, 7),
        (stripe_pts, PALETTE['stripe'], PALETTE['stripe'],    1.0, 8),
        (window_pts, PALETTE['window'], PALETTE['body_edge'], 0.9, 8),
        (fin_l,      PALETTE['fin'],    PALETTE['body_edge'], 1.0, 6),
        (fin_r,      PALETTE['fin'],    PALETTE['body_edge'], 1.0, 6),
        (nozzle_pts, PALETTE['nozzle'], PALETTE['body_edge'], 1.0, 6),
    ]:
        ax.add_patch(mpatches.Polygon(
            body_to_world(pts, x, y, theta),
            closed=True, facecolor=fc, edgecolor=ec, lw=lw, zorder=zo
        ))

    # Thrust flame — nozzle deflects opposite to delta sign:
    # positive delta tilts thrust right of body axis → torque increases phi (nose left).
    # So the flame draws at theta - delta (nozzle swings right when delta > 0).
    nozzle_c = body_to_world(np.array([[0.0, -hh - 0.14]]), x, y, theta)[0]
    thrust_angle = theta - delta
    fs = max(throttle, 0.02)
    for ln, lw_f, fc in [
        (1.55 * fs, 0.17, PALETTE['flame_outer']),
        (1.0  * fs, 0.11, PALETTE['flame_mid']),
        (0.55 * fs, 0.06, PALETTE['flame_core']),
    ]:
        flame_pts = np.array([[-lw_f, 0], [lw_f, 0], [0, -ln]])
        ax.add_patch(mpatches.Polygon(
            body_to_world(flame_pts, nozzle_c[0], nozzle_c[1], thrust_angle),
            closed=True, facecolor=fc, edgecolor='none', alpha=0.85, zorder=3
        ))

    ax.plot([x, x], [y, y + 1.5], color=PALETTE['vertical'],
            lw=1.2, ls='--', alpha=0.9, zorder=4)


def save_all_figures(result: SimulationResult, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    t     = result.t
    state = result.state
    ctl   = result.controls
    drv   = result.derived

    # ── 1. Position and velocity ─────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True,
                              facecolor=PALETTE['bg'])
    for ax in axes.flat:
        _style_ax(ax)

    axes[0, 0].plot(t, state[:, IDX_X], color=PALETTE['theta'])
    axes[0, 0].axhline(result.config["controller"].get("x_d", 0), ls='--',
                        color=PALETTE['vertical'], lw=1)
    axes[0, 0].set_ylabel('x [m]', color=PALETTE['label'])
    axes[0, 0].set_title('Horizontal position', color=PALETTE['title'], fontsize=9)

    axes[0, 1].plot(t, state[:, IDX_Y], color=PALETTE['omega'])
    axes[0, 1].axhline(0, ls='--', color=PALETTE['vertical'], lw=1)
    axes[0, 1].set_ylabel('y [m]', color=PALETTE['label'])
    axes[0, 1].set_title('Altitude', color=PALETTE['title'], fontsize=9)

    axes[1, 0].plot(t, state[:, IDX_VX], color=PALETTE['theta'])
    axes[1, 0].axhline(0, ls='--', color=PALETTE['vertical'], lw=0.8)
    axes[1, 0].set_ylabel('vx [m/s]', color=PALETTE['label'])
    axes[1, 0].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[1, 0].set_title('Horizontal velocity', color=PALETTE['title'], fontsize=9)

    axes[1, 1].plot(t, state[:, IDX_VY], color=PALETTE['omega'])
    axes[1, 1].axhline(0, ls='--', color=PALETTE['vertical'], lw=0.8)
    axes[1, 1].set_ylabel('vy [m/s]', color=PALETTE['label'])
    axes[1, 1].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[1, 1].set_title('Vertical velocity', color=PALETTE['title'], fontsize=9)

    fig.suptitle(f'Position and velocity — {_ctrl_label(result)} landing',
                 color=PALETTE['title'], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / 'position_velocity.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 2. Attitude + nozzle ─────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True,
                              facecolor=PALETTE['bg'])
    for ax in axes:
        _style_ax(ax)

    axes[0].plot(t, np.degrees(state[:, IDX_PHI]),
                 color=PALETTE['theta'], label='phi(t)')
    axes[0].plot(t, np.degrees(ctl['theta_star']), '--',
                 color=PALETTE['vertical'], label='phi*(t)')
    axes[0].set_ylabel('Angle [deg]', color=PALETTE['label'])
    axes[0].set_title('Pitch angle vs desired', color=PALETTE['title'], fontsize=9)
    axes[0].legend(fontsize=8, facecolor=PALETTE['panel'],
                   labelcolor=PALETTE['label'])

    axes[1].plot(t, np.degrees(state[:, IDX_OMEGA]),
                 color=PALETTE['omega'], label='omega(t)')
    axes[1].plot(t, np.degrees(ctl['alpha1']), '--',
                 color=PALETTE['accel'], label='alpha1(t)')
    axes[1].set_ylabel('[deg/s]', color=PALETTE['label'])
    axes[1].set_title('Angular rate vs virtual reference', color=PALETTE['title'], fontsize=9)
    axes[1].legend(fontsize=8, facecolor=PALETTE['panel'],
                   labelcolor=PALETTE['label'])

    axes[2].plot(t, np.degrees(state[:, IDX_DELTA]),
                 color=PALETTE['delta'], label='delta(t)')
    axes[2].plot(t, np.degrees(ctl['alpha2f']), '--',
                 color=PALETTE['lyap'], label='alpha2f(t)')
    axes[2].plot(t, np.degrees(ctl['delta_cmd']), ':',
                 color=PALETTE['sigma'], lw=1.2, label='delta_cmd')
    axes[2].axhline(math.degrees(result.params.delta_max_cmd),
                    color='#ff7aa8', ls=':', lw=1.0)
    axes[2].axhline(-math.degrees(result.params.delta_max_cmd),
                    color='#ff7aa8', ls=':', lw=1.0)
    axes[2].set_ylabel('[deg]', color=PALETTE['label'])
    axes[2].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[2].set_title('Nozzle state vs filter reference',
                      color=PALETTE['title'], fontsize=9)
    axes[2].legend(fontsize=8, facecolor=PALETTE['panel'],
                   labelcolor=PALETTE['label'])

    fig.tight_layout()
    fig.savefig(output_dir / 'attitude_and_gimbal.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 3. Backstepping errors (z-coordinates) ───────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True,
                              facecolor=PALETTE['bg'])
    for ax in axes:
        _style_ax(ax)

    axes[0].plot(t, np.degrees(drv['z_phi']),
                 color=PALETTE['z_phi'], label='z_phi = phi - phi*')
    axes[0].axhline(0, ls='--', color=PALETTE['grid'], lw=0.8)
    axes[0].set_ylabel('[deg]', color=PALETTE['label'])
    axes[0].set_title('Pitch error z_phi', color=PALETTE['title'], fontsize=9)
    axes[0].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    axes[1].plot(t, np.degrees(drv['z_omega']),
                 color=PALETTE['z_omega'], label='z_omega = omega - alpha1')
    axes[1].axhline(0, ls='--', color=PALETTE['grid'], lw=0.8)
    axes[1].set_ylabel('[deg/s]', color=PALETTE['label'])
    axes[1].set_title('Rate error z_omega', color=PALETTE['title'], fontsize=9)
    axes[1].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    axes[2].plot(t, np.degrees(drv['z_delta']),
                 color=PALETTE['z_delta'], label='z_delta = delta - alpha2f')
    axes[2].axhline(0, ls='--', color=PALETTE['grid'], lw=0.8)
    axes[2].set_ylabel('[deg]', color=PALETTE['label'])
    axes[2].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[2].set_title('Actuator error z_delta', color=PALETTE['title'], fontsize=9)
    axes[2].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    fig.suptitle(f'Tracking errors z_phi, z_omega, z_delta — {_ctrl_label(result)}',
                 color=PALETTE['title'], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / 'backstepping_errors.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 4. Lyapunov function + throttle ──────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                              facecolor=PALETTE['bg'])
    for ax in axes:
        _style_ax(ax)

    axes[0].plot(t, drv['lyap_V'], color=PALETTE['lyap'], lw=1.8,
                 label='V(t) — composite Lyapunov function')
    axes[0].set_ylabel('V(t)', color=PALETTE['label'])
    axes[0].set_title('Unified Lyapunov function (should be non-increasing)',
                      color=PALETTE['title'], fontsize=9)
    axes[0].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    axes[1].plot(t, ctl['sigma'], color=PALETTE['sigma'], label='sigma (throttle)')
    axes[1].axhline(result.params.throttle_hover, ls='--',
                    color=PALETTE['vertical'], lw=1, label='hover throttle')
    axes[1].set_ylabel('Throttle', color=PALETTE['label'])
    axes[1].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[1].set_title('Throttle command', color=PALETTE['title'], fontsize=9)
    axes[1].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    fig.tight_layout()
    fig.savefig(output_dir / 'lyapunov_and_throttle.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 5. Coupling terms P and Q ─────────────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                              facecolor=PALETTE['bg'])
    for ax in axes:
        _style_ax(ax)

    axes[0].plot(t, drv['P_coupling'], color=PALETTE['theta'], lw=1.5,
                 label='P = (F/m)(vx·cos(phi*) − vy·sin(phi*))')
    axes[0].axhline(0, ls='--', color=PALETTE['grid'], lw=0.8)
    axes[0].set_ylabel('[m/s²]', color=PALETTE['label'])
    axes[0].set_title('Position-attitude coupling P', color=PALETTE['title'], fontsize=9)
    axes[0].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    axes[1].plot(t, drv['Q_coupling'], color=PALETTE['omega'], lw=1.5,
                 label='Q = (F/m)(vx·cos(phi) − vy·sin(phi))')
    axes[1].axhline(0, ls='--', color=PALETTE['grid'], lw=0.8)
    axes[1].set_ylabel('[m/s²]', color=PALETTE['label'])
    axes[1].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[1].set_title('Nozzle-position coupling Q', color=PALETTE['title'], fontsize=9)
    axes[1].legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])

    fig.suptitle(f'Coupling terms P and Q — {_ctrl_label(result)}',
                 color=PALETTE['title'], fontsize=11)
    fig.tight_layout()
    fig.savefig(output_dir / 'coupling_terms.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)

    # ── 6. Planar trajectory ──────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(7, 9), facecolor=PALETTE['bg'])
    _style_ax(ax)
    ax.plot(state[:, IDX_X], state[:, IDX_Y],
            color=PALETTE['trail'], lw=2.0, label='trajectory')
    ax.scatter([state[0, IDX_X]], [state[0, IDX_Y]],
               s=60, color=PALETTE['omega'], zorder=5, label='start')
    ax.scatter([state[-1, IDX_X]], [state[-1, IDX_Y]],
               s=60, color=PALETTE['theta'], zorder=5, label='end')
    x_d = result.config["controller"].get("x_d", 0)
    ax.scatter([x_d], [0], marker='X', s=80,
               color=PALETTE['ground_line'], zorder=5, label='target')
    ax.axhline(0, color=PALETTE['ground_line'], lw=1.4, zorder=0)
    x_min = min(float(state[:, IDX_X].min()), x_d) - 5
    x_max = max(float(state[:, IDX_X].max()), x_d) + 5
    ax.fill_between([x_min, x_max], -10, 0,
                    color=PALETTE['ground'], alpha=0.5, zorder=0)
    ax.set_title('Planar landing trajectory', color=PALETTE['title'], fontsize=10)
    ax.set_xlabel('x [m]', color=PALETTE['label'])
    ax.set_ylabel('y [m]', color=PALETTE['label'])
    ax.set_aspect('equal', adjustable='box')
    ax.legend(fontsize=8, facecolor=PALETTE['panel'], labelcolor=PALETTE['label'])
    fig.tight_layout()
    fig.savefig(output_dir / 'planar_trajectory.png', dpi=190,
                facecolor=fig.get_facecolor())
    plt.close(fig)


def save_preview_figure(result: SimulationResult, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    idx = min(len(result.t) - 1, len(result.t) // 4)
    fig = plt.figure(figsize=(7.4, 8.2), facecolor=PALETTE['bg'])
    ax = fig.add_subplot(111)
    draw_rocket(
        ax,
        x=float(result.state[idx, IDX_X]),
        y=float(result.state[idx, IDX_Y]),
        theta=float(result.state[idx, IDX_PHI]),
        delta=float(result.state[idx, IDX_DELTA]),
        throttle=float(result.controls['sigma'][idx]),
        trail_x=result.state[:idx + 1, IDX_X],
        trail_y=result.state[:idx + 1, IDX_Y],
        title=f"{_ctrl_label(result)} landing",
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_animation(result: SimulationResult, output_path: Path) -> None:
    output_path = output_path.with_suffix('.gif')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t      = result.t
    state  = result.state
    ctl    = result.controls

    sample_dt  = float(np.mean(np.diff(t))) if len(t) > 1 else 0.05
    target_fps = 10
    stride     = max(1, int(round(1.0 / (target_fps * sample_dt))))
    frames     = list(range(0, len(t), stride))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)

    # Use non-interactive Agg backend to avoid GPU/display driver crashes on Linux.
    import matplotlib
    matplotlib.use('Agg')

    fig = plt.figure(figsize=(7.2, 8.0), facecolor=PALETTE['bg'])
    ax  = fig.add_subplot(111)

    def update(frame_num):
        idx = frames[frame_num]
        ctrl_lbl = _ctrl_label(result)
        draw_rocket(
            ax,
            x=float(state[idx, IDX_X]),
            y=float(state[idx, IDX_Y]),
            theta=float(state[idx, IDX_PHI]),
            delta=float(state[idx, IDX_DELTA]),
            throttle=float(ctl['sigma'][idx]),
            trail_x=state[:idx + 1, IDX_X],
            trail_y=state[:idx + 1, IDX_Y],
            title=f"{ctrl_lbl} landing",
        )
        info = (
            f"t      = {t[idx]:5.2f} s\n"
            f"x      = {state[idx, IDX_X]:+7.2f} m\n"
            f"y      = {state[idx, IDX_Y]:+7.2f} m\n"
            f"phi    = {math.degrees(state[idx, IDX_PHI]):+6.2f} deg\n"
            f"omega  = {math.degrees(state[idx, IDX_OMEGA]):+6.2f} deg/s\n"
            f"delta  = {math.degrees(state[idx, IDX_DELTA]):+5.2f} deg\n"
            f"sigma  = {ctl['sigma'][idx]:.3f}"
        )
        ax.text(0.02, 0.98, info, transform=ax.transAxes, va='top',
                color=PALETTE['text'], fontsize=8.5, fontfamily='monospace',
                bbox=dict(boxstyle='round,pad=0.35', fc=PALETTE['bg'],
                          ec=PALETTE['grid'], alpha=0.90))
        fig.suptitle(f'Planar TVC Rocket — {ctrl_lbl}',
                     color=PALETTE['text'], fontsize=11)

    try:
        anim = animation.FuncAnimation(
            fig, update, frames=len(frames), interval=1000 // target_fps
        )
        anim.save(str(output_path), writer=animation.PillowWriter(fps=target_fps),
                  dpi=80)
    finally:
        plt.close(fig)
