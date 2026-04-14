#!/usr/bin/env bash
set -euo pipefail

python -m src.main --config configs/default.yaml --output-root .
