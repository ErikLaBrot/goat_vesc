#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: scripts/run_vesc_hardware_smoke.sh [extra example args...]

Environment:
  VESC_DEVICE
  VESC_BAUD
  VESC_IMU_POLL_MS
  VESC_MOTOR_POLL_MS
  VESC_PHASE_MS
  VESC_STATUS_MS
  VESC_DUTY
  VESC_CURRENT
  VESC_RPM
  VESC_BRAKE_CURRENT
  VESC_SERVO_CENTER
  VESC_SERVO_AMPLITUDE
  VESC_ARM_ACTUATORS=1

This script launches the already-built build/default/vesc_hardware_smoke example.
It only passes --arm-actuators when VESC_ARM_ACTUATORS=1 or when you supply the
flag explicitly on the command line.
EOF
  exit 0
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$repo_root/build/default/vesc_hardware_smoke"

if [[ ! -x "$binary" ]]; then
  echo "Missing executable: $binary" >&2
  echo "Build it first with: cmake --build --preset default --target vesc_hardware_smoke" >&2
  exit 1
fi

cmd=("$binary")

if [[ -n "${VESC_DEVICE:-}" ]]; then
  cmd+=("--device" "$VESC_DEVICE")
fi
if [[ -n "${VESC_BAUD:-}" ]]; then
  cmd+=("--baud" "$VESC_BAUD")
fi
if [[ -n "${VESC_IMU_POLL_MS:-}" ]]; then
  cmd+=("--imu-poll-ms" "$VESC_IMU_POLL_MS")
fi
if [[ -n "${VESC_MOTOR_POLL_MS:-}" ]]; then
  cmd+=("--motor-poll-ms" "$VESC_MOTOR_POLL_MS")
fi
if [[ -n "${VESC_PHASE_MS:-}" ]]; then
  cmd+=("--phase-ms" "$VESC_PHASE_MS")
fi
if [[ -n "${VESC_STATUS_MS:-}" ]]; then
  cmd+=("--status-ms" "$VESC_STATUS_MS")
fi
if [[ -n "${VESC_DUTY:-}" ]]; then
  cmd+=("--duty" "$VESC_DUTY")
fi
if [[ -n "${VESC_CURRENT:-}" ]]; then
  cmd+=("--current" "$VESC_CURRENT")
fi
if [[ -n "${VESC_RPM:-}" ]]; then
  cmd+=("--rpm" "$VESC_RPM")
fi
if [[ -n "${VESC_BRAKE_CURRENT:-}" ]]; then
  cmd+=("--brake-current" "$VESC_BRAKE_CURRENT")
fi
if [[ -n "${VESC_SERVO_CENTER:-}" ]]; then
  cmd+=("--servo-center" "$VESC_SERVO_CENTER")
fi
if [[ -n "${VESC_SERVO_AMPLITUDE:-}" ]]; then
  cmd+=("--servo-amplitude" "$VESC_SERVO_AMPLITUDE")
fi
if [[ "${VESC_ARM_ACTUATORS:-0}" == "1" ]]; then
  cmd+=("--arm-actuators")
fi

cmd+=("$@")

printf 'Running:'
printf ' %q' "${cmd[@]}"
printf '\n'
"${cmd[@]}"
