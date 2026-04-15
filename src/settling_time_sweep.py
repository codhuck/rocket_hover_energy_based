from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml

from .simulation import simulate


K_PHI_VALUES = np.arange(6.0, 26.0, 2.0)
K_OMEGA_VALUES = np.arange(2.0, 8.0, 1.0)


def load_config(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_case(base_cfg: dict[str, Any], k_phi: float, k_omega: float) -> dict[str, float | None]:
    cfg = deepcopy(base_cfg)
    cfg["controller"]["k_phi"] = float(k_phi)
    cfg["controller"]["k_omega"] = float(k_omega)

    result = simulate(cfg)
    return {
        "k_phi": float(k_phi),
        "k_omega": float(k_omega),
        "phi_error_band_rad": float(result.summary["phi_error_band_rad"]),
        "settling_time_phi_error_band_s": result.summary["settling_time_phi_error_band_s"],
    }


def generate_sweep_rows(
    base_cfg: dict[str, Any],
    k_phi_values: np.ndarray = K_PHI_VALUES,
    k_omega_values: np.ndarray = K_OMEGA_VALUES,
) -> list[dict[str, float | None]]:
    rows: list[dict[str, float | None]] = []
    for k_omega in k_omega_values:
        for k_phi in k_phi_values:
            rows.append(run_case(base_cfg, float(k_phi), float(k_omega)))
    return rows


def save_rows_csv(rows: list[dict[str, float | None]], output_path: Path) -> None:
    fieldnames = [
        "k_phi",
        "k_omega",
        "phi_error_band_rad",
        "settling_time_phi_error_band_s",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_rows_json(rows: list[dict[str, float | None]], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2)


def build_surface_arrays(
    rows: list[dict[str, float | None]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    k_phi_values = np.array(sorted({float(row["k_phi"]) for row in rows}), dtype=float)
    k_omega_values = np.array(sorted({float(row["k_omega"]) for row in rows}), dtype=float)
    x_grid, y_grid = np.meshgrid(k_phi_values, k_omega_values)
    z_grid = np.full_like(x_grid, np.nan, dtype=float)

    omega_index = {value: idx for idx, value in enumerate(k_omega_values)}
    phi_index = {value: idx for idx, value in enumerate(k_phi_values)}

    for row in rows:
        i = omega_index[float(row["k_omega"])]
        j = phi_index[float(row["k_phi"])]
        settling_time = row["settling_time_phi_error_band_s"]
        z_grid[i, j] = np.nan if settling_time is None else float(settling_time)

    return x_grid, y_grid, z_grid


def save_surface_plot(rows: list[dict[str, float | None]], output_path: Path) -> None:
    x_grid, y_grid, z_grid = build_surface_arrays(rows)

    fig = plt.figure(figsize=(9, 6.8))
    ax = fig.add_subplot(111, projection="3d")
    surface = ax.plot_surface(x_grid, y_grid, z_grid, cmap="viridis", edgecolor="none", antialiased=True)
    ax.set_xlabel("k_phi")
    ax.set_ylabel("k_omega")
    ax.set_zlabel("Settling time [s]")
    ax.set_title("Settling Time Surface over k_phi and k_omega")
    ax.view_init(elev=28, azim=-132)
    fig.colorbar(surface, ax=ax, shrink=0.7, pad=0.1, label="Settling time [s]")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def save_sweep_outputs(rows: list[dict[str, float | None]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    save_rows_csv(rows, output_dir / "settling_time_grid.csv")
    save_rows_json(rows, output_dir / "settling_time_grid.json")
    save_surface_plot(rows, output_dir / "settling_time_surface.png")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a regular-grid settling time sweep over k_phi and k_omega.")
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-root", type=Path, default=Path("outputs/settling_time_sweep"))
    args = parser.parse_args()

    base_cfg = load_config(args.config)
    rows = generate_sweep_rows(base_cfg)
    save_sweep_outputs(rows, args.output_root)

    print(json.dumps(
        {
            "cases": len(rows),
            "output_root": str(args.output_root),
            "csv": str(args.output_root / "settling_time_grid.csv"),
            "json": str(args.output_root / "settling_time_grid.json"),
            "plot": str(args.output_root / "settling_time_surface.png"),
        },
        indent=2,
    ))


if __name__ == "__main__":
    main()
