from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .simulation import simulate
from .visualization import save_all_figures, save_animation, save_preview_figure


def _load_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _run_single(cfg: dict, output_root: Path, animate: bool = False) -> None:
    figures_dir    = output_root / "figures"
    animations_dir = output_root / "animations"
    summary_path   = figures_dir / "summary.json"

    result = simulate(cfg)
    save_all_figures(result, figures_dir)
    save_preview_figure(result, figures_dir / "rocket_visualization_preview.png")

    if animate:
        ctrl_type = cfg["controller"].get("type", "backstepping")
        save_animation(result, animations_dir / f"{ctrl_type}_landing.gif")

    figures_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(result.summary, indent=2), encoding="utf-8")
    print(json.dumps(result.summary, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run planar TVC rocket backstepping landing simulation."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/backstepping.yaml"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/backstepping"),
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        default=False,
        help="Also render a GIF animation (slow — off by default).",
    )
    args = parser.parse_args()

    cfg = _load_config(args.config)
    _run_single(cfg, args.output_root, animate=args.animate)


if __name__ == "__main__":
    main()
