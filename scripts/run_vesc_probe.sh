#!/usr/bin/env bash
# Probe example runner.
#
# Purpose:
#   Launch the already-built `vesc_probe` example with optional device and baud
#   overrides for operator bring-up checks.
#
# Inputs:
#   Optional `device_path` and `baud` positional arguments or the `VESC_DEVICE`
#   and `VESC_BAUD` environment variables.
#
# Outputs:
#   Runs the `build/default/vesc_probe` binary and prints its operator-facing
#   output to the terminal.
#
# Usage:
#   scripts/run_vesc_probe.sh
#   scripts/run_vesc_probe.sh /dev/ttyACM0 115200
#
# Notes:
#   The target binary must already exist. This wrapper does not build it.
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
