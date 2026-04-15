#!/usr/bin/env bash
set -euo pipefail

OUTPUT_ROOT="${1:-outputs}"

CONFIGS=(
  "default"
  "tuning_soft"
  "tuning_balanced"
  "tuning_fast"
  "tuning_springy"
)

for name in "${CONFIGS[@]}"; do
  config_path="configs/${name}.yaml"
  output_path="${OUTPUT_ROOT}/${name}"

  echo "Running ${config_path} -> ${output_path}"
  python -m src.main --config "${config_path}" --output-root "${output_path}"
done
