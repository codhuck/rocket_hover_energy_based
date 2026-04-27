from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .simulation import simulate, simulate_comparison, comparison_diff
from .visualization import (
    save_all_figures,
    save_animation,
    save_preview_figure,
    plot_comparison,
)


def _load_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _run_single(cfg: dict, output_root: Path) -> None:
    figures_dir = output_root / "figures"
    animations_dir = output_root / "animations"
    summary_path = figures_dir / "summary.json"

    result = simulate(cfg)
    save_all_figures(result, figures_dir)
    save_preview_figure(result, figures_dir / "rocket_visualization_preview.png")
    save_animation(result, animations_dir / "rocket_attitude_realtime.gif")

    figures_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(result.summary, indent=2), encoding="utf-8")
    print(json.dumps(result.summary, indent=2))


def _run_comparison(cfg: dict, output_root: Path) -> None:
    figures_dir = output_root / "figures"
    animations_dir = output_root / "animations"
    summary_path = figures_dir / "summary.json"

    result_baseline, result_adaptive = simulate_comparison(cfg)

    # Per-controller subdirectories with full per-run figures.
    save_all_figures(result_baseline, figures_dir / "baseline")
    save_all_figures(result_adaptive, figures_dir / "adaptive")
    save_preview_figure(
        result_adaptive,
        figures_dir / "rocket_visualization_preview.png",
    )

    # Side-by-side comparison plot.
    plot_comparison(
        result_baseline,
        result_adaptive,
        figures_dir,
        label_baseline="Project 1 (Lyapunov)",
        label_adaptive="Project 2 (Adaptive CE)",
    )

    # Animation of the adaptive run (the more interesting one to watch).
    save_animation(
        result_adaptive,
        animations_dir / "rocket_attitude_adaptive.gif",
    )

    summary_payload = {
        "baseline": result_baseline.summary,
        "adaptive": result_adaptive.summary,
        "diff": comparison_diff(result_baseline, result_adaptive),
    }
    figures_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(summary_payload, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary_payload, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run planar TVC rocket simulation (single controller or comparison)."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/default.yaml"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("."),
    )
    parser.add_argument(
        "--mode",
        choices=["single", "comparison"],
        default="single",
        help="single: run one controller (cfg['controller']). "
             "comparison: run baseline and adaptive in cfg['comparison'].",
    )
    args = parser.parse_args()

    cfg = _load_config(args.config)

    if args.mode == "single":
        _run_single(cfg, args.output_root)
    else:
        _run_comparison(cfg, args.output_root)


if __name__ == "__main__":
    main()
