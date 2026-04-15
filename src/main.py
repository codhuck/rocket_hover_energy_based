from __future__ import annotations

import argparse
import json
from pathlib import Path
import yaml

from .simulation import simulate
from .visualization import save_all_figures, save_animation, save_preview_figure


def main() -> None:
    parser = argparse.ArgumentParser(description='Run planar TVC rocket simulation.')
    parser.add_argument('--config', type=Path, default=Path('configs/default.yaml'))
    parser.add_argument('--output-root', type=Path, default=Path('.'))
    args = parser.parse_args()

    with args.config.open('r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    result = simulate(cfg)
    figures_dir = args.output_root / 'figures'
    animations_dir = args.output_root / 'animations'

    save_all_figures(result, figures_dir)
    save_preview_figure(result, figures_dir / 'rocket_visualization_preview.png')

    summary_path = figures_dir / 'summary.json'
    summary_path.write_text(json.dumps(result.summary, indent=2), encoding='utf-8')

    save_animation(result, animations_dir / 'rocket_attitude_realtime.mp4')

    print(json.dumps(result.summary, indent=2))


if __name__ == '__main__':
    main()