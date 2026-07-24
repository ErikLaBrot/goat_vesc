# goat_vesc Operator Quick Reference

`goat_vesc` is a GOAT-focused VESC transport library. It gives higher-level
applications a thread-safe way to connect to a controller, read the important
telemetry, and send the core drive and steering commands without handling
packet framing or serial arbitration themselves.

## Discovery And Lifecycle

- `VescClient::find_devices()`: Return visible `/dev/ttyACM*` candidates.
- `connect()`: Open the configured transport and start the background I/O
  thread.
- `disconnect()`: Stop the background thread and close the transport.
- `is_connected()`: Report whether the client still considers the transport
  active.

## Telemetry Access

- `latest_imu()`: Return the latest cached IMU sample, if any.
- `latest_motor_state()`: Return the latest cached motor telemetry sample, if
  any.
- `subscribe_imu(...)`: Register a callback for fresh IMU samples.
- `subscribe_motor_state(...)`: Register a callback for fresh motor-state
  samples.

Callbacks run on the I/O thread. Exceptions are contained, but callbacks should
remain short because they delay transport work; blocking requests return no
result from that thread. Do not destroy the client from its own callback. A
callback already copied for dispatch may run once after its subscription resets.

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

The high-level client supports:

- `set_rpm(...)`
- `set_duty(...)`
- `set_current(...)`
- `set_current_brake(...)`
- `set_servo_pos(...)`

These are the bridge-facing control outputs the library is designed around.

## Runtime Configuration And Diagnostics

Runtime adjustment and diagnostic entry points include:

- `set_imu_poll_interval(...)`
- `set_motor_poll_interval(...)`
- `config_snapshot()`
- `request_fw_version(...)`

## Controller Configuration And LispBM

- `request_motor_config(...)` and `request_app_config(...)` return opaque,
  firmware-native images.
- `write_motor_config(...)` persists a motor image.
- `write_app_config(...)` explicitly selects volatile or persistent storage.
- `request_lisp_code(...)`, `erase_lisp_code(...)`, and
  `write_lisp_code(...)` manage stored LispBM source/import bytes.
- `set_lisp_running(...)` explicitly starts or stops LispBM. Upload leaves it
  stopped.
- `set_app_output_disabled(...)` queues bounded firmware application-output
  suppression. The command has no acknowledgement.
- `run_foc_calibration(...)` performs direct-controller firmware-7.00 FOC
  detection and returns the raw firmware result code.

Motor and app writes compare the image's embedded schema signature with the
active controller before sending. A management timeout stops the connection;
reconnect and reread state before retrying. Persistent writes change controller
flash and require the same hardware authorization as other real-device changes.
FOC calibration also persists motor configuration and moves the motor. It
requires separate actuator authorization and an unloaded, secured motor; it
never scans CAN.

## `VescConfig` Options

The runtime config object includes:

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
  Deadline for a sent periodic poll to receive a reply. Blocking operations use
  the timeout supplied to that operation.
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

## Configuration Model

Host runtime config is programmatic today. The library does not include a file
loader or interpret VESC configuration fields. Controller images are
firmware-specific binary data intended for a version-aware consuming
application.

The expected pattern today is:

1. Higher-level GOAT code decides the desired launch/config values.
2. That code populates a `VescConfig`.
3. The configured `VescClient` is constructed from that object.

## Manual Tools

Built examples:

- `vesc_probe`
  Connect, query firmware, and verify IMU plus motor telemetry.
- `vesc_hardware_smoke`
  Runs telemetry polling, live subscriptions, explicitly armed command phases,
  and a firmware query under load in one operator-facing hardware smoke pass.

Build the requested target first, then invoke its binary from `build/default/`.
Actuating validation requires an explicit target and arming flag:

```bash
build/default/vesc_hardware_smoke --device /dev/ttyACM0 --arm-actuators
```

Final neutral commands are best effort. VESC-side timeout and hardware limits
remain the failure backstop.
