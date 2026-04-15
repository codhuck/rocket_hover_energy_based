from __future__ import annotations

from pathlib import Path
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import shutil
import subprocess
import tempfile

from .simulation import SimulationResult


PALETTE = {
    'bg': '#0d0f1a',
    'panel': '#121629',
    'grid': '#242b45',
    'tick': '#6f7aa8',
    'title': '#d7dff7',
    'label': '#a9b4da',
    'body': '#d9dde7',
    'body_edge': '#f6f7fb',
    'stripe': '#d53c33',
    'window': '#7fc8ff',
    'fin': '#9aa3ba',
    'nozzle': '#6e768a',
    'flame_outer': '#ff5a36',
    'flame_mid': '#ffb703',
    'flame_core': '#fff2a8',
    'trail': '#79a8ff',
    'vertical': '#5ef38c',
    'crosshair': '#ffffff',
    'phi': '#5aa0ff',
    'omega': '#ffb14e',
    'delta': '#cf9fff',
    'accel': '#61d095',
    'text': '#f2f5ff',
    'ground': '#1b3a1f',
    'ground_line': '#46a758',
}


def rot2d(phi: float) -> np.ndarray:
    c = math.cos(phi)
    s = math.sin(phi)
    return np.array([[c, -s], [s, c]])


def body_to_world(points: np.ndarray, x: float, y: float, phi: float) -> np.ndarray:
    return (rot2d(phi) @ points.T).T + np.array([x, y])


def draw_rocket(ax, x: float, y: float, phi: float, delta: float, alpha: float, trail_x, trail_y, viewport_hw: float = 4.5, viewport_hh: float = 5.8):
    ax.clear()
    ax.set_facecolor(PALETTE['panel'])
    ax.grid(color=PALETTE['grid'], linewidth=0.6, alpha=0.7)
    ax.tick_params(colors=PALETTE['tick'], labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE['grid'])

    ax.set_xlim(x - viewport_hw, x + viewport_hw)
    ax.set_ylim(y - viewport_hh, y + viewport_hh)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]', color=PALETTE['label'], fontsize=8)
    ax.set_ylabel('y [m]', color=PALETTE['label'], fontsize=8)
    ax.set_title('Rocket-centred camera', color=PALETTE['title'], fontsize=9, pad=6)

    ax.axhline(0.0, color=PALETTE['ground_line'], lw=1.4, zorder=0)
    ax.fill_between([x - 100, x + 100], -100, 0.0, color=PALETTE['ground'], alpha=0.6, zorder=0)

    ax.plot(trail_x, trail_y, color=PALETTE['trail'], lw=1.6, alpha=0.85, zorder=1, label='trajectory')

    body_h = 1.55
    body_w = 0.34
    half_h = body_h / 2.0
    half_w = body_w / 2.0

    body_pts = np.array([
        [-half_w, -half_h],
        [ half_w, -half_h],
        [ half_w,  half_h * 0.64],
        [ 0.0,     half_h],
        [-half_w,  half_h * 0.64],
    ])
    stripe_pts = np.array([
        [-half_w * 0.85, -0.10],
        [ half_w * 0.85, -0.10],
        [ half_w * 0.85,  0.16],
        [-half_w * 0.85,  0.16],
    ])
    window_pts = np.array([
        [-0.08, 0.28],
        [ 0.08, 0.28],
        [ 0.10, 0.48],
        [ 0.00, 0.58],
        [-0.10, 0.48],
    ])
    fin_left = np.array([
        [-half_w, -0.32],
        [-half_w - 0.18, -0.60],
        [-half_w * 0.25, -0.52],
    ])
    fin_right = fin_left.copy()
    fin_right[:, 0] *= -1.0
    nozzle_pts = np.array([
        [-0.09, -half_h],
        [ 0.09, -half_h],
        [ 0.05, -half_h - 0.16],
        [-0.05, -half_h - 0.16],
    ])

    for pts, fc, ec, lw, zo in [
        (body_pts, PALETTE['body'], PALETTE['body_edge'], 1.6, 7),
        (stripe_pts, PALETTE['stripe'], PALETTE['stripe'], 1.0, 8),
        (window_pts, PALETTE['window'], PALETTE['body_edge'], 0.9, 8),
        (fin_left, PALETTE['fin'], PALETTE['body_edge'], 1.0, 6),
        (fin_right, PALETTE['fin'], PALETTE['body_edge'], 1.0, 6),
        (nozzle_pts, PALETTE['nozzle'], PALETTE['body_edge'], 1.0, 6),
    ]:
        world = body_to_world(pts, x, y, phi)
        ax.add_patch(mpatches.Polygon(world, closed=True, facecolor=fc, edgecolor=ec, lw=lw, zorder=zo))

    ax.plot([x - viewport_hw, x + viewport_hw], [y, y], color=PALETTE['crosshair'], lw=0.35, alpha=0.15, zorder=1)
    ax.plot([x, x], [y - viewport_hh, y + viewport_hh], color=PALETTE['crosshair'], lw=0.35, alpha=0.15, zorder=1)

    body_axis = np.array([[0.0, 0.0], [0.0, 1.2]])
    body_axis_world = body_to_world(body_axis, x, y, phi)
    ax.plot(body_axis_world[:, 0], body_axis_world[:, 1], color=PALETTE['body_edge'], lw=1.0, alpha=0.7, zorder=9)
    ax.plot([x, x], [y, y + 1.2], color=PALETTE['vertical'], lw=1.2, ls='--', alpha=0.9, zorder=4, label='upright reference')

    nozzle_center = body_to_world(np.array([[0.0, -half_h - 0.14]]), x, y, phi)[0]
    thrust_angle = phi + delta
    flame_scale = max(alpha, 0.02)
    lengths = [1.55 * flame_scale, 1.0 * flame_scale, 0.55 * flame_scale]
    widths = [0.17, 0.11, 0.06]
    colors = [PALETTE['flame_outer'], PALETTE['flame_mid'], PALETTE['flame_core']]
    for length, width, color in zip(lengths, widths, colors):
        flame_local = np.array([
            [-width, 0.0],
            [ width, 0.0],
            [ 0.0, -length],
        ])
        flame_world = body_to_world(flame_local, nozzle_center[0], nozzle_center[1], thrust_angle)
        ax.add_patch(mpatches.Polygon(flame_world, closed=True, facecolor=color, edgecolor='none', alpha=0.85, zorder=3))

    arc_r = 0.42
    arc_theta = np.linspace(phi + math.pi, thrust_angle + math.pi, 25)
    ax.plot(
        nozzle_center[0] + arc_r * np.sin(arc_theta),
        nozzle_center[1] - arc_r * np.cos(arc_theta),
        color='#ff7aa8', lw=1.3, alpha=0.9, zorder=9,
    )

    ax.legend(loc='lower right', fontsize=7, facecolor=PALETTE['panel'], edgecolor=PALETTE['grid'], labelcolor=PALETTE['label'])


def _decorate_timeseries_axes(ax):
    ax.set_facecolor(PALETTE['panel'])
    ax.grid(color=PALETTE['grid'], linewidth=0.6, alpha=0.7)
    ax.tick_params(colors=PALETTE['tick'], labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor(PALETTE['grid'])


def save_all_figures(result: SimulationResult, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    t = result.t
    state = result.state
    controls = result.controls
    derived = result.derived

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True, facecolor=PALETTE['bg'])
    for ax in axes:
        _decorate_timeseries_axes(ax)
    axes[0].plot(t, state[:, 0], label='x(t)', color=PALETTE['phi'])
    axes[0].plot(t, state[:, 1], label='y(t)', color=PALETTE['omega'])
    axes[0].set_ylabel('Position [m]', color=PALETTE['label'])
    axes[0].set_title('Translational response under attitude-only control', color=PALETTE['title'], fontsize=10)
    axes[0].legend(fontsize=8)

    axes[1].plot(t, np.degrees(state[:, 2]), label='phi(t)', color=PALETTE['phi'])
    axes[1].plot(t, np.degrees(controls['phi_target']), '--', label='phi_target', color=PALETTE['vertical'])
    axes[1].plot(t, np.degrees(controls['delta']), label='delta(t)', color=PALETTE['delta'])
    axes[1].set_ylabel('Angle [deg]', color=PALETTE['label'])
    axes[1].set_title('Attitude regulation and gimbal command', color=PALETTE['title'], fontsize=10)
    axes[1].legend(fontsize=8)

    axes[2].plot(t, np.degrees(state[:, 5]), label='omega(t)', color=PALETTE['omega'])
    axes[2].plot(t, derived['speed'], label='speed(t)', color=PALETTE['accel'])
    axes[2].plot(t, controls['alpha'], label='alpha(t)', color=PALETTE['delta'])
    axes[2].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[2].set_ylabel('Magnitude', color=PALETTE['label'])
    axes[2].set_title('Angular-rate decay and constant hover throttle', color=PALETTE['title'], fontsize=10)
    axes[2].legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(output_dir / 'state_trajectories.png', dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 5.2), facecolor=PALETTE['bg'])
    _decorate_timeseries_axes(ax)
    ax.plot(t, np.degrees(state[:, 2]), label='phi(t)', color=PALETTE['phi'])
    ax.plot(t, np.degrees(controls['phi_target']), '--', label='phi_target', color=PALETTE['vertical'])
    ax.plot(t, np.degrees(controls['delta']), label='delta(t)', color=PALETTE['delta'])
    ax.axhline(np.degrees(result.params.delta_max), color='#ff7aa8', ls=':', lw=1.0, label='±delta_max')
    ax.axhline(-np.degrees(result.params.delta_max), color='#ff7aa8', ls=':', lw=1.0)
    ax.set_title('Attitude and gimbal command', color=PALETTE['title'], fontsize=10)
    ax.set_xlabel('Time [s]', color=PALETTE['label'])
    ax.set_ylabel('Angle [deg]', color=PALETTE['label'])
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / 'attitude_and_gimbal.png', dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 5.2), facecolor=PALETTE['bg'])
    _decorate_timeseries_axes(ax)
    ax.plot(t, controls['alpha'], label='Throttle alpha', color=PALETTE['delta'])
    ax.plot(t, derived['speed'], label='Speed [m/s]', color=PALETTE['omega'])
    ax.plot(t, np.degrees(np.abs(controls['e_phi'])), label='|phi error| [deg]', color=PALETTE['accel'])
    ax.set_title('Control effort and attitude convergence', color=PALETTE['title'], fontsize=10)
    ax.set_xlabel('Time [s]', color=PALETTE['label'])
    ax.set_ylabel('Magnitude', color=PALETTE['label'])
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / 'control_and_error.png', dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.8, 8.4), facecolor=PALETTE['bg'])
    _decorate_timeseries_axes(ax)
    ax.plot(state[:, 0], state[:, 1], color=PALETTE['trail'], lw=2.0, label='trajectory')
    ax.scatter([state[0, 0]], [state[0, 1]], s=60, color=PALETTE['omega'], label='start')
    ax.scatter([state[-1, 0]], [state[-1, 1]], s=60, color=PALETTE['phi'], label='end')
    ax.set_title('Planar drift while pitch is being stabilized', color=PALETTE['title'], fontsize=10)
    ax.set_xlabel('x [m]', color=PALETTE['label'])
    ax.set_ylabel('y [m]', color=PALETTE['label'])
    ax.set_aspect('equal', adjustable='box')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / 'planar_trajectory.png', dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_comparison(result_pd: SimulationResult, result_cross: SimulationResult, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    t_pd = result_pd.t
    t_cross = result_cross.t

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True, facecolor=PALETTE['bg'])
    for ax in axes:
        _decorate_timeseries_axes(ax)

    axes[0].plot(t_pd, np.degrees(result_pd.state[:, 2]), color=PALETTE['phi'], lw=1.8, label='PD')
    axes[0].plot(t_cross, np.degrees(result_cross.state[:, 2]), color=PALETTE['vertical'], lw=1.8, ls='--', label='Cross-term')
    axes[0].set_ylabel('phi [deg]', color=PALETTE['label'])
    axes[0].set_title('Attitude angle comparison', color=PALETTE['title'], fontsize=10)
    axes[0].legend(fontsize=8)

    axes[1].plot(t_pd, np.degrees(result_pd.state[:, 5]), color=PALETTE['omega'], lw=1.8, label='PD')
    axes[1].plot(t_cross, np.degrees(result_cross.state[:, 5]), color=PALETTE['accel'], lw=1.8, ls='--', label='Cross-term')
    axes[1].set_ylabel('omega [deg/s]', color=PALETTE['label'])
    axes[1].set_title('Angular rate comparison', color=PALETTE['title'], fontsize=10)
    axes[1].legend(fontsize=8)

    axes[2].plot(t_pd, np.degrees(result_pd.controls['delta']), color=PALETTE['delta'], lw=1.8, label='PD')
    axes[2].plot(t_cross, np.degrees(result_cross.controls['delta']), color=PALETTE['trail'], lw=1.8, ls='--', label='Cross-term')
    axes[2].set_xlabel('Time [s]', color=PALETTE['label'])
    axes[2].set_ylabel('delta [deg]', color=PALETTE['label'])
    axes[2].set_title('Gimbal command comparison', color=PALETTE['title'], fontsize=10)
    axes[2].legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(output_dir / 'comparison.png', dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_preview_figure(result: SimulationResult, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    preview_idx = min(len(result.t) - 1, len(result.t) // 3)
    fig = plt.figure(figsize=(7.4, 8.2), facecolor=PALETTE['bg'])
    ax = fig.add_subplot(111)
    draw_rocket(
        ax,
        x=float(result.state[preview_idx, 0]),
        y=float(result.state[preview_idx, 1]),
        phi=float(result.state[preview_idx, 2]),
        delta=float(result.controls['delta'][preview_idx]),
        alpha=float(result.controls['alpha'][preview_idx]),
        trail_x=result.state[: preview_idx + 1, 0],
        trail_y=result.state[: preview_idx + 1, 1],
    )
    ax.set_title('Visualisation preview: attitude-only rocket stabilization', color=PALETTE['title'], fontsize=10)
    fig.tight_layout()
    fig.savefig(output_path, dpi=190, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_animation(result: SimulationResult, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = result.t
    state = result.state
    controls = result.controls
    derived = result.derived

    sample_dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.05
    target_fps = 20
    stride = max(1, int(round(1.0 / (target_fps * sample_dt))))
    fps = target_fps

    fig = plt.figure(figsize=(7.2, 8.0), facecolor=PALETTE['bg'])
    ax = fig.add_subplot(111)
    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir_path = Path(tmpdir)
            frame_idx = 0
            for idx in range(0, len(t), stride):
                draw_rocket(
                    ax,
                    x=float(state[idx, 0]),
                    y=float(state[idx, 1]),
                    phi=float(state[idx, 2]),
                    delta=float(controls['delta'][idx]),
                    alpha=float(controls['alpha'][idx]),
                    trail_x=state[: idx + 1, 0],
                    trail_y=state[: idx + 1, 1],
                )
                info = (
                    f"t = {t[idx]:5.2f} s\n"
                    f"phi = {np.degrees(state[idx, 2]):+6.2f} deg\n"
                    f"omega = {np.degrees(state[idx, 5]):+6.2f} deg/s\n"
                    f"delta = {np.degrees(controls['delta'][idx]):+5.2f} deg\n"
                    f"alpha = {controls['alpha'][idx]:0.3f}\n"
                    f"speed = {derived['speed'][idx]:.3f} m/s"
                )
                ax.text(
                    0.02, 0.98, info, transform=ax.transAxes, va='top',
                    color=PALETTE['text'], fontsize=8.5, fontfamily='monospace',
                    bbox=dict(boxstyle='round,pad=0.35', fc=PALETTE['bg'], ec=PALETTE['grid'], alpha=0.90),
                )
                fig.suptitle('Planar TVC Rocket — Attitude Stabilization', color=PALETTE['text'], fontsize=11)
                fig.savefig(tmpdir_path / f"frame_{frame_idx:05d}.png", dpi=80, facecolor=fig.get_facecolor())
                frame_idx += 1

            if (len(t) - 1) % stride != 0:
                idx = len(t) - 1
                draw_rocket(
                    ax,
                    x=float(state[idx, 0]),
                    y=float(state[idx, 1]),
                    phi=float(state[idx, 2]),
                    delta=float(controls['delta'][idx]),
                    alpha=float(controls['alpha'][idx]),
                    trail_x=state[: idx + 1, 0],
                    trail_y=state[: idx + 1, 1],
                )
                info = (
                    f"t = {t[idx]:5.2f} s\n"
                    f"phi = {np.degrees(state[idx, 2]):+6.2f} deg\n"
                    f"omega = {np.degrees(state[idx, 5]):+6.2f} deg/s\n"
                    f"delta = {np.degrees(controls['delta'][idx]):+5.2f} deg\n"
                    f"alpha = {controls['alpha'][idx]:0.3f}\n"
                    f"speed = {derived['speed'][idx]:.3f} m/s"
                )
                ax.text(
                    0.02, 0.98, info, transform=ax.transAxes, va='top',
                    color=PALETTE['text'], fontsize=8.5, fontfamily='monospace',
                    bbox=dict(boxstyle='round,pad=0.35', fc=PALETTE['bg'], ec=PALETTE['grid'], alpha=0.90),
                )
                fig.suptitle('Planar TVC Rocket — Attitude Stabilization', color=PALETTE['text'], fontsize=11)
                fig.savefig(tmpdir_path / f"frame_{frame_idx:05d}.png", dpi=80, facecolor=fig.get_facecolor())

            ffmpeg_exe = shutil.which('ffmpeg')
            if ffmpeg_exe is None:
                import imageio_ffmpeg
                ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()

            cmd = [
                ffmpeg_exe, '-y', '-loglevel', 'error', '-framerate', str(fps),
                '-i', str(tmpdir_path / 'frame_%05d.png'),
                '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', str(output_path),
            ]
            subprocess.run(cmd, check=True)
    finally:
        plt.close(fig)