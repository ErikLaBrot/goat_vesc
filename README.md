# goat_vesc

`goat_vesc` is the VESC transport library for the GOAT autonomous racer
platform. It is intended for the GOAT vehicle stack running on a Jetson Orin
Nano / ARM Linux system and provides a thread-safe transport owner, typed
protocol helpers, periodic telemetry polling, cached latest-value access, and
callback-based subscriptions for fresh samples.

## Requirements

| Area | Requirement | Notes |
|---|---|---|
| Build system | CMake 3.16 or newer | See [`CMakeLists.txt`](CMakeLists.txt). |
| Compiler | C++17 compiler | |
| Target platform | Jetson Orin Nano running ARM Linux | |
| Hardware / transport | GOAT racer VESC hardware reachable over the expected serial device path | |
| Optional docs | Doxygen | Only needed if you want to build the generated API documentation. |

## Quickstart

Configure, build, and test with the default preset:

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default
```

This will build the ['examples/'](examples/). These are demonstrations of the library basics, and how to implement the library at an application layer.

## API At A Glance

| Area | Primary entry points | Purpose |
|---|---|---|
| Discovery and lifecycle | `VescClient::find_devices()`, `connect()`, `disconnect()`, `is_connected()` | Find a controller and manage the transport thread. |
| Telemetry access | `latest_imu()`, `latest_motor_state()`, `subscribe_imu(...)`, `subscribe_motor_state(...)` | Read cached samples or receive fresh decoded telemetry. |
| Runtime config | `set_imu_poll_interval(...)`, `set_motor_poll_interval(...)`, `config_snapshot()` | Adjust polling cadence and inspect active bridge-facing settings. |
| Control output | `set_rpm(...)`, `set_duty(...)`, `set_current(...)`, `set_current_brake(...)`, `set_servo_pos(...)` | Send motor and servo commands through the serialized transport path. |
| Diagnostics | `request_fw_version(...)` | Perform a blocking firmware version query. |

Lower-level packet and protocol helpers are also installed for applications that
need direct access to framing or typed request/response parsing.

## API Docs

Doxygen-generated API docs can be built with:

```bash
cmake --build --preset default --target docs
```

Generated HTML lands under `docs/html/`.

For implementation-side context, see
[`docs/src/ARCHITECTURE.md`](docs/src/ARCHITECTURE.md).

## Release / Workflow Notes

Bridge readiness requirements, release tracking context, and milestone-oriented
workflow notes live in
[`docs/src/BRIDGE_READINESS_CHECKLIST.md`](docs/src/BRIDGE_READINESS_CHECKLIST.md).
