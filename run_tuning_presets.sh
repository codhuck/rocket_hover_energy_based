#!/usr/bin/env bash
set -euo pipefail

OUTPUT_ROOT="${1:-outputs}"
FIGURES_DIR="figures"

export MPLCONFIGDIR="${OUTPUT_ROOT}/.mplconfig"
export XDG_CACHE_HOME="${OUTPUT_ROOT}/.cache"

CONFIGS=(
  "default"
  "tuning_soft"
  "tuning_balanced"
  "tuning_fast"
  "tuning_springy"
)

mkdir -p "${FIGURES_DIR}"
mkdir -p "${MPLCONFIGDIR}" "${XDG_CACHE_HOME}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg is required to generate GIF files." >&2
  exit 1
fi

for name in "${CONFIGS[@]}"; do
  config_path="configs/${name}.yaml"
  output_path="${OUTPUT_ROOT}/${name}"
  video_path="${output_path}/animations/rocket_attitude_realtime.mp4"
  palette_path="${output_path}/animations/${name}_palette.png"
  gif_path="${FIGURES_DIR}/${name}.gif"

  echo "Running ${config_path} -> ${output_path}"
  python -m src.main --config "${config_path}" --output-root "${output_path}"

  echo "Creating ${gif_path}"
  ffmpeg -y -loglevel error -i "${video_path}" -vf "fps=15,scale=720:-1:flags=lanczos,palettegen" "${palette_path}"
  ffmpeg -y -loglevel error -i "${video_path}" -i "${palette_path}" -lavfi "fps=15,scale=720:-1:flags=lanczos[x];[x][1:v]paletteuse" "${gif_path}"
  rm -f "${palette_path}"
done
