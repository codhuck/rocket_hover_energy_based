from __future__ import annotations

import argparse
import json
from pathlib import Path
import yaml

from .simulation import simulate_both
from .visualization import save_all_figures, save_animation, save_preview_figure, plot_comparison


def main() -> None:
    parser = argparse.ArgumentParser(description='Run planar TVC rocket simulation.')
    parser.add_argument('--config', type=Path, default=Path('configs/default.yaml'))
    parser.add_argument('--output-root', type=Path, default=Path('.'))
    args = parser.parse_args()

    with args.config.open('r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    result_pd, result_cross = simulate_both(cfg)
    figures_dir = args.output_root / 'figures'
    animations_dir = args.output_root / 'animations'

    save_all_figures(result_pd, figures_dir)
    save_preview_figure(result_pd, figures_dir / 'rocket_visualization_preview.png')
    plot_comparison(result_pd, result_cross, figures_dir)

    summary_path = figures_dir / 'summary.json'
    summary_payload = {
        'pd': result_pd.summary,
        'cross_term': result_cross.summary,
        'comparison': {
            'final_phi_deg_diff': float(result_cross.summary['final_phi_deg'] - result_pd.summary['final_phi_deg']),
            'final_omega_deg_s_diff': float(result_cross.summary['final_omega_deg_s'] - result_pd.summary['final_omega_deg_s']),
            'max_abs_phi_deg_diff': float(result_cross.summary['max_abs_phi_deg'] - result_pd.summary['max_abs_phi_deg']),
            'max_abs_delta_deg_diff': float(result_cross.summary['max_abs_delta_deg'] - result_pd.summary['max_abs_delta_deg']),
            'max_speed_diff': float(result_cross.summary['max_speed'] - result_pd.summary['max_speed']),
        },
    }
    summary_path.write_text(json.dumps(summary_payload, indent=2), encoding='utf-8')

    save_animation(result_pd, animations_dir / 'rocket_attitude_realtime.mp4')

    print(json.dumps(result_pd.summary, indent=2))


if __name__ == '__main__':
    main()