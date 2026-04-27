from __future__ import annotations

import argparse
import csv
import json
from copy import deepcopy
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml

from .simulation import simulate


# Default sweep grid: attitude gain k_theta and adaptation gain gamma.
K_THETA_VALUES = np.arange(6.0, 26.0, 2.0)
GAMMA_VALUES = np.array([0.5, 1.0, 2.0, 5.0, 10.0, 20.0])

# All metrics written to CSV. The first one is used as the default
# z-axis for the 3D surface plot.
METRIC_KEYS = [
    "settling_time_theta_error_band_s",
    "final_c_hat_error",
    "max_abs_theta_deg",
    "max_abs_delta_deg",
    "final_theta_deg",
    "final_omega_deg_s",
    "max_speed",
]


def load_config(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_case(base_cfg: dict[str, Any], k_theta: float, gamma: float) -> dict[str, float | None]:
    cfg = deepcopy(base_cfg)
    cfg["controller"]["k_theta"] = float(k_theta)
    cfg["controller"]["gamma"] = float(gamma)

    result = simulate(cfg)

    row: dict[str, float | None] = {
        "k_theta": float(k_theta),
        "gamma": float(gamma),
    }
    for metric in METRIC_KEYS:
        value = result.summary.get(metric)
        row[metric] = None if value is None else float(value)
    return row


def generate_sweep_rows(
    base_cfg: dict[str, Any],
    k_theta_values: np.ndarray = K_THETA_VALUES,
    gamma_values: np.ndarray = GAMMA_VALUES,
) -> list[dict[str, float | None]]:
    rows: list[dict[str, float | None]] = []
    for gamma in gamma_values:
        for k_theta in k_theta_values:
            rows.append(run_case(base_cfg, float(k_theta), float(gamma)))
    return rows


def save_rows_csv(rows: list[dict[str, float | None]], output_path: Path) -> None:
    fieldnames = ["k_theta", "gamma", *METRIC_KEYS]
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_rows_json(rows: list[dict[str, float | None]], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2)


def build_surface_arrays(
    rows: list[dict[str, float | None]],
    metric: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    k_theta_values = np.array(sorted({float(row["k_theta"]) for row in rows}), dtype=float)
    gamma_values = np.array(sorted({float(row["gamma"]) for row in rows}), dtype=float)
    x_grid, y_grid = np.meshgrid(k_theta_values, gamma_values)
    z_grid = np.full_like(x_grid, np.nan, dtype=float)

    gamma_index = {value: idx for idx, value in enumerate(gamma_values)}
    theta_index = {value: idx for idx, value in enumerate(k_theta_values)}

    for row in rows:
        i = gamma_index[float(row["gamma"])]
        j = theta_index[float(row["k_theta"])]
        value = row.get(metric)
        z_grid[i, j] = np.nan if value is None else float(value)

    return x_grid, y_grid, z_grid


def save_surface_plot(
    rows: list[dict[str, float | None]],
    output_path: Path,
    metric: str,
) -> None:
    x_grid, y_grid, z_grid = build_surface_arrays(rows, metric)

    fig = plt.figure(figsize=(9, 6.8))
    ax = fig.add_subplot(111, projection="3d")
    surface = ax.plot_surface(
        x_grid, y_grid, z_grid, cmap="viridis", edgecolor="none", antialiased=True
    )
    ax.set_xlabel("k_theta")
    ax.set_ylabel("gamma")
    ax.set_zlabel(metric)
    ax.set_title(f"{metric} surface over (k_theta, gamma)")
    ax.view_init(elev=28, azim=-132)
    fig.colorbar(surface, ax=ax, shrink=0.7, pad=0.1, label=metric)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def save_sweep_outputs(
    rows: list[dict[str, float | None]],
    output_dir: Path,
    metric: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    save_rows_csv(rows, output_dir / "sweep_grid.csv")
    save_rows_json(rows, output_dir / "sweep_grid.json")
    save_surface_plot(rows, output_dir / f"surface_{metric}.png", metric)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a regular-grid sweep over k_theta and gamma for adaptive control."
    )
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-root", type=Path, default=Path("outputs/sweep"))
    parser.add_argument(
        "--metric",
        type=str,
        default=METRIC_KEYS[0],
        choices=METRIC_KEYS,
        help="Metric used for the 3D surface plot (all metrics still go to CSV).",
    )
    args = parser.parse_args()

    base_cfg = load_config(args.config)
    rows = generate_sweep_rows(base_cfg)
    save_sweep_outputs(rows, args.output_root, args.metric)

    print(json.dumps(
        {
            "cases": len(rows),
            "output_root": str(args.output_root),
            "metric": args.metric,
            "csv": str(args.output_root / "sweep_grid.csv"),
            "json": str(args.output_root / "sweep_grid.json"),
            "plot": str(args.output_root / f"surface_{args.metric}.png"),
        },
        indent=2,
    ))


if __name__ == "__main__":
    main()
