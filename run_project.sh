#!/usr/bin/env bash
set -euo pipefail
python -m src.main --config configs/mpc.yaml --output-root outputs/mpc "$@"
