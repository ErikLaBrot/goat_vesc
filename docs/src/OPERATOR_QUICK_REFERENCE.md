# goat_vesc Operator Quick Reference

`goat_vesc` is a GOAT-focused VESC transport library. It gives higher-level
applications a thread-safe way to connect to a controller, read the important
telemetry, and send the core drive and steering commands without handling
packet framing or serial arbitration themselves.

## Discovery And Lifecycle

- `VescClient::find_devices()`
  Returns visible `/dev/ttyACM*` candidates.
- `connect()`
  Opens the configured transport and starts the background I/O thread.
- `disconnect()`
  Stops the background thread and closes the transport.
- `is_connected()`
  Reports whether the client still considers the transport active.

## Telemetry Access

- `latest_imu()`
  Returns the latest cached IMU sample, if any.
- `latest_motor_state()`
  Returns the latest cached motor telemetry sample, if any.
- `subscribe_imu(...)`
  Registers a callback for fresh IMU samples.
- `subscribe_motor_state(...)`
  Registers a callback for fresh motor-state samples.

## Motor Telemetry

`VescMotorState` currently exposes:

- `stamp_ns`
- `rpm`
- `current_motor`
- `current_in`
- `duty_cycle`
- `vin`
- `temp_motor`
- `temp_fet`
- `tachometer`
- `tachometer_abs`
- `fault_code`

For power-oriented monitoring, the most useful fields are usually:

- `vin`
  Battery/input voltage
- `current_in`
  Input current from the battery side
- `vin * current_in`
  Derived electrical input power in watts

## IMU Telemetry

`VescIMUData` currently exposes:

- `stamp_ns`
- `roll`, `pitch`, `yaw`
- `acc_x`, `acc_y`, `acc_z`
- `gyro_x`, `gyro_y`, `gyro_z`
- `mag_x`, `mag_y`, `mag_z`
- `quat_w`, `quat_x`, `quat_y`, `quat_z`

## Commands

The high-level client currently supports:

- `set_rpm(...)`
- `set_duty(...)`
- `set_current(...)`
- `set_current_brake(...)`
- `set_servo_pos(...)`

These are the bridge-facing control outputs the library is designed around.

## Runtime Config And Diagnostics

The client currently exposes:

- `set_imu_poll_interval(...)`
- `set_motor_poll_interval(...)`
- `config_snapshot()`
- `request_fw_version(...)`

## `VescConfig` Options

The runtime config object currently includes:

- `device_path`
  Serial device path. If empty, the client auto-detects the first
  `/dev/ttyACM*` device.
- `baud`
  Serial baud rate.
- `motor_poll_interval`
  Motor-state polling cadence.
- `imu_poll_interval`
  IMU polling cadence.
- `poll_response_timeout`
  Deadline for a sent poll or query to receive a reply.
- `query_guard_window`
  Guard band that keeps one-shot queries from cutting too close to due polls.
- `command_watchdog_timeout`
  Optional host-side stale-command timeout.
- `command_watchdog_action`
  Safe-stop action for the stale-command watchdog.
- `max_brake_current`
  Maximum allowed active brake-current magnitude.
- `command_watchdog_brake_current`
  Requested watchdog brake-current before clamping.
- `wall_time_ns`
  Optional timestamp source for decoded samples.
- `open_serial_fn`
  Optional custom transport opener used mainly for tests or alternate backends.

## What Is Configurable Today

Config is programmatic today. The library does not include a built-in pre-launch
file loader for YAML, TOML, JSON, or similar formats.

The expected pattern today is:

1. Higher-level GOAT code decides the desired launch/config values.
2. That code populates a `VescConfig`.
3. The configured `VescClient` is constructed from that object.

## Real Hardware Smoke Tools

Focused manual examples already included:

- `vesc_probe`
  Connect, query firmware, and verify IMU plus motor telemetry.
- `vesc_duty_sweep`
  Manual duty-cycle sweep against real hardware.
- `vesc_servo_sweep`
  Manual servo sweep against real hardware.

New integrated manual smoke tool:

- `vesc_hardware_smoke`
  Runs telemetry polling, live subscriptions, concurrent command streaming, and
  a firmware query under load in one operator-facing hardware smoke pass.

Operator runner scripts:

- `scripts/run_vesc_probe.sh`
- `scripts/run_vesc_hardware_smoke.sh`

These scripts only launch already-built example binaries with transparent,
operator-visible arguments.
