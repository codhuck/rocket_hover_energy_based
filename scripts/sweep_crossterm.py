#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.controller import CrossTermLyapunovController
from src.simulation import _simulate_with_controller


def frange(start: float, stop: float, step: float) -> list[float]:
    values = []
    x = start
    while x <= stop + 1e-12:
        values.append(float(round(x, 10)))
        x += step
    return values


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--k-phi", type=float, default=25.0)
    parser.add_argument("--k-omega", type=float, default=10.0)
    parser.add_argument("--k-c-start", type=float, default=0.0)
    parser.add_argument("--k-c-stop", type=float, default=5.0)
    parser.add_argument("--k-c-step", type=float, default=1.0)
    parser.add_argument("--c-start", type=float, default=0.2)
    parser.add_argument("--c-stop", type=float, default=2.2)
    parser.add_argument("--c-step", type=float, default=0.4)
    parser.add_argument("--phi-threshold", type=float, default=0.1)
    parser.add_argument("--output", type=Path, default=Path("figures/crossterm_sweep_phi01_compact.json"))
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as f:
        base_cfg = yaml.safe_load(f)

    base_cfg["controller_crossterm"]["k_phi"] = float(args.k_phi)
    base_cfg["controller_crossterm"]["k_omega"] = float(args.k_omega)

    k_c_values = frange(args.k_c_start, args.k_c_stop, args.k_c_step)
    c_values = frange(args.c_start, args.c_stop, args.c_step)

    rows = []
    for k_c in k_c_values:
        for c in c_values:
            if c * c >= args.k_phi:
                continue

            cfg = copy.deepcopy(base_cfg)
            cfg["controller_crossterm"]["k_c"] = float(k_c)
            cfg["controller_crossterm"]["c"] = float(c)

            controller = CrossTermLyapunovController.from_config(cfg)
            result = _simulate_with_controller(cfg, controller)

            t = result.t
            phi_abs = np.abs(result.state[:, 2])
            below = phi_abs < float(args.phi_threshold)

            first_idx = np.where(below)[0]
            t_first = float(t[first_idx[0]]) if first_idx.size else None

            entries = np.where((~below[:-1]) & (below[1:]))[0] + 1
            entry_times = [float(t[i]) for i in entries]

            settle_idx = None
            for i in range(len(t)):
                if np.all(below[i:]):
                    settle_idx = i
                    break
            t_settle = float(t[settle_idx]) if settle_idx is not None else None

            rows.append(
                {
                    "k_c": float(k_c),
                    "c": float(c),
                    "t_first_below_threshold": t_first,
                    "t_settle_persistent": t_settle,
                    "n_entries": int(len(entry_times)),
                    "entry_times": entry_times,
                    "max_abs_delta_deg": float(np.max(np.abs(np.degrees(result.controls["delta"])))),
                    "max_speed": float(np.max(result.derived["speed"])),
                    "final_phi_deg": float(np.degrees(result.state[-1, 2])),
                }
            )

    ranked = sorted(
        rows,
        key=lambda r: (
            float("inf") if r["t_settle_persistent"] is None else r["t_settle_persistent"],
            float("inf") if r["t_first_below_threshold"] is None else r["t_first_below_threshold"],
            r["max_speed"],
        ),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(ranked, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"saved: {args.output}")
    print(f"cases: {len(ranked)}")
    if ranked:
        print("best:", json.dumps(ranked[0], ensure_ascii=False))


if __name__ == "__main__":
    main()
