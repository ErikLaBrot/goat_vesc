#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: scripts/run_vesc_probe.sh [device_path] [baud]

Environment:
  VESC_DEVICE   Default device path when no CLI device_path is supplied
  VESC_BAUD     Default baud when a device path is supplied and no CLI baud is supplied

This script launches the already-built build/default/vesc_probe example.
EOF
  exit 0
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$repo_root/build/default/vesc_probe"

if [[ ! -x "$binary" ]]; then
  echo "Missing executable: $binary" >&2
  echo "Build it first with: cmake --build --preset default --target vesc_probe" >&2
  exit 1
fi

device="${1:-${VESC_DEVICE:-}}"
baud="${2:-${VESC_BAUD:-115200}}"

cmd=("$binary")
if [[ -n "$device" ]]; then
  cmd+=("$device" "$baud")
elif [[ "${VESC_BAUD:-}" != "" && "${VESC_BAUD:-115200}" != "115200" ]]; then
  echo "A custom baud requires an explicit device path for vesc_probe" >&2
  exit 2
fi

printf 'Running:'
printf ' %q' "${cmd[@]}"
printf '\n'
"${cmd[@]}"
