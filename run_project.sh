#!/usr/bin/env bash
set -euo pipefail

python -m src.main --config configs/backstepping.yaml --output-root outputs/backstepping --animate
python -m src.main --config configs/pid_cascade.yaml --output-root outputs/pid_cascade --animate
