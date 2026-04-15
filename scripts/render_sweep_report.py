#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def to_float_or_nan(value):
    if value is None:
        return float("nan")
    return float(value)


def get_value(row, *keys, default=None):
    for key in keys:
        if key in row:
            return row[key]
    return default


def load_rows(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError("Input JSON must be a list of sweep rows.")
    return data


def write_csv(rows, output_csv: Path):
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "k_c",
        "c",
        "t_first_below_threshold_s",
        "t_settle_persistent_s",
        "n_entries",
        "entry_times_s",
        "max_abs_delta_deg",
        "max_speed_mps",
        "final_phi_deg",
    ]
    with output_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(
                {
                    "k_c": r["k_c"],
                    "c": r["c"],
                    "t_first_below_threshold_s": get_value(r, "t_first_below_threshold", "t_first"),
                    "t_settle_persistent_s": get_value(r, "t_settle_persistent", "t_settle"),
                    "n_entries": r.get("n_entries"),
                    "entry_times_s": "; ".join(str(x) for x in r.get("entry_times", [])),
                    "max_abs_delta_deg": get_value(r, "max_abs_delta_deg", "max_delta_deg"),
                    "max_speed_mps": r.get("max_speed"),
                    "final_phi_deg": r.get("final_phi_deg"),
                }
            )


def write_markdown(rows, output_md: Path, threshold: float):
    output_md.parent.mkdir(parents=True, exist_ok=True)
    converged = [r for r in rows if get_value(r, "t_first_below_threshold", "t_first") is not None]
    non_converged = [r for r in rows if get_value(r, "t_first_below_threshold", "t_first") is None]
    top = sorted(
        converged,
        key=lambda r: (
            get_value(r, "t_settle_persistent", "t_settle"),
            get_value(r, "t_first_below_threshold", "t_first"),
            r["max_speed"],
        ),
    )[:10]

    lines = []
    lines.append("# Cross-Term Sweep Report")
    lines.append("")
    lines.append(f"- Input rows: **{len(rows)}**")
    lines.append(f"- Convergence threshold: **|phi| < {threshold} rad**")
    lines.append(f"- Converged cases: **{len(converged)}**")
    lines.append(f"- Non-converged cases: **{len(non_converged)}**")
    lines.append("")
    lines.append("## Top 10 by Persistent Settling Time")
    lines.append("")
    lines.append("| Rank | k_c | c | First Entry [s] | Settling [s] | Entries | Max \\|delta\\| [deg] | Max Speed [m/s] |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|")
    for i, r in enumerate(top, 1):
        lines.append(
            f"| {i} | {r['k_c']:.3g} | {r['c']:.3g} | {get_value(r, 't_first_below_threshold', 't_first'):.4f} | "
            f"{get_value(r, 't_settle_persistent', 't_settle'):.4f} | {r['n_entries']} | {get_value(r, 'max_abs_delta_deg', 'max_delta_deg'):.4f} | {r['max_speed']:.4f} |"
        )
    lines.append("")
    lines.append("## Non-Converged Cases")
    lines.append("")
    if non_converged:
        lines.append("| k_c | c | Max \\|delta\\| [deg] | Max Speed [m/s] | Final phi [deg] |")
        lines.append("|---:|---:|---:|---:|---:|")
        for r in sorted(non_converged, key=lambda x: (x["k_c"], x["c"])):
            lines.append(
                f"| {r['k_c']:.3g} | {r['c']:.3g} | {get_value(r, 'max_abs_delta_deg', 'max_delta_deg'):.4f} | {r['max_speed']:.4f} | {r['final_phi_deg']:.4f} |"
            )
    else:
        lines.append("All cases converged.")
    lines.append("")
    output_md.write_text("\n".join(lines), encoding="utf-8")


def build_grid(rows, key):
    k_values = sorted({float(r["k_c"]) for r in rows})
    c_values = sorted({float(r["c"]) for r in rows})
    grid = np.full((len(k_values), len(c_values)), np.nan, dtype=float)
    k_index = {v: i for i, v in enumerate(k_values)}
    c_index = {v: i for i, v in enumerate(c_values)}
    for r in rows:
        i = k_index[float(r["k_c"])]
        j = c_index[float(r["c"])]
        value = get_value(r, key, "t_settle" if key == "t_settle_persistent" else None, "max_delta_deg" if key == "max_abs_delta_deg" else None)
        grid[i, j] = to_float_or_nan(value)
    return np.array(k_values), np.array(c_values), grid


def plot_heatmap(rows, key, title, colorbar_label, output_path: Path, cmap="viridis"):
    k_values, c_values, grid = build_grid(rows, key)
    fig, ax = plt.subplots(figsize=(8, 5.5))
    masked = np.ma.masked_invalid(grid)
    im = ax.imshow(masked, origin="lower", aspect="auto", cmap=cmap)
    ax.set_title(title)
    ax.set_xlabel("c")
    ax.set_ylabel("k_c")
    ax.set_xticks(np.arange(len(c_values)))
    ax.set_xticklabels([f"{v:.2g}" for v in c_values])
    ax.set_yticks(np.arange(len(k_values)))
    ax.set_yticklabels([f"{v:.2g}" for v in k_values])
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label(colorbar_label)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("figures/crossterm_sweep_phi01_compact.json"),
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=Path("figures/crossterm_sweep_report"),
    )
    parser.add_argument("--threshold", type=float, default=0.1)
    args = parser.parse_args()

    rows = load_rows(args.input)
    rows = sorted(rows, key=lambda r: (float(r["k_c"]), float(r["c"])))

    csv_path = args.output_prefix.with_suffix(".csv")
    md_path = args.output_prefix.with_suffix(".md")
    write_csv(rows, csv_path)
    write_markdown(rows, md_path, args.threshold)

    plot_heatmap(
        rows,
        key="t_settle_persistent",
        title="Persistent Settling Time Heatmap",
        colorbar_label="time [s]",
        output_path=args.output_prefix.parent / f"{args.output_prefix.name}_heatmap_t_settle.png",
        cmap="viridis",
    )
    plot_heatmap(
        rows,
        key="max_abs_delta_deg",
        title="Peak Gimbal Angle Heatmap",
        colorbar_label="|delta| max [deg]",
        output_path=args.output_prefix.parent / f"{args.output_prefix.name}_heatmap_delta.png",
        cmap="magma",
    )
    plot_heatmap(
        rows,
        key="max_speed",
        title="Peak Speed Heatmap",
        colorbar_label="speed max [m/s]",
        output_path=args.output_prefix.parent / f"{args.output_prefix.name}_heatmap_speed.png",
        cmap="plasma",
    )

    print(f"table_csv: {csv_path}")
    print(f"table_md: {md_path}")
    print(f"heatmap_t_settle: {args.output_prefix.parent / f'{args.output_prefix.name}_heatmap_t_settle.png'}")
    print(f"heatmap_delta: {args.output_prefix.parent / f'{args.output_prefix.name}_heatmap_delta.png'}")
    print(f"heatmap_speed: {args.output_prefix.parent / f'{args.output_prefix.name}_heatmap_speed.png'}")


if __name__ == "__main__":
    main()
