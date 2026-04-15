# goat_vesc

`goat_vesc` is the GOAT racer VESC transport library. It provides a
thread-safe, C++17 interface for connecting to a controller, exchanging typed
protocol messages, polling core telemetry, and sending drive or steering
commands through one transport owner.

## Purpose

This library is intended for higher-level GOAT applications that need:

- a reusable CMake package instead of application-local serial code
- one place to own transport lifecycle and serialized command writes
- typed helpers for the bridge-facing VESC message set
- cached telemetry reads plus callback-based delivery of fresh samples

The library does not include a ROS interface. `goat_vesc_ros` is the ROS-facing
adapter layer that consumes this installed package.

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

This builds the library, tests, and the operator-facing examples under
[`examples/`](examples/), including the integrated
`vesc_hardware_smoke` manual validation tool.

## Installed Surface

`goat_vesc` installs as a reusable CMake package. A clean install emits:

- public headers under `include/goat_vesc/`
- the `goat_vesc` library artifact
- CMake package metadata under `lib/cmake/goat_vesc/`
- `share/goat_vesc/package.xml`

One simple local install flow is:

```bash
cmake -S . -B build/install-export \
  -DGOAT_VESC_BUILD_TESTS=OFF \
  -DGOAT_VESC_BUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/goat_vesc"
cmake --build build/install-export
cmake --install build/install-export
```

Downstream consumers can then point CMake at that install prefix with
`CMAKE_PREFIX_PATH` or `goat_vesc_DIR` and use the exported target:

```cmake
find_package(goat_vesc CONFIG REQUIRED)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app PRIVATE goat_vesc::goat_vesc)
```

`goat_vesc_ros` is expected to consume the library through this installed
package interface rather than via direct source-tree coupling.

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

## Manual Tools

The repository ships focused real-hardware examples and thin runner scripts for
manual validation:

- `vesc_probe`
  Connect, query firmware, and confirm IMU plus motor-state telemetry.
- `vesc_duty_sweep`
  Manual duty-cycle sweep against real hardware.
- `vesc_servo_sweep`
  Manual servo sweep around a chosen center position.
- `vesc_hardware_smoke`
  Integrated smoke pass for telemetry, subscriptions, firmware queries, and
  concurrent command streaming.
- `scripts/run_vesc_probe.sh`
  Launch the built `vesc_probe` binary with transparent operator arguments.
- `scripts/run_vesc_hardware_smoke.sh`
  Launch the built `vesc_hardware_smoke` binary with environment-driven
  defaults.
- `scripts/run_quality_gate.sh`
  Run formatting, static analysis, build, and test checks for the repository.

## Documentation

Doxygen-generated API docs can be built with:

```bash
cmake --build --preset default --target docs
```

Generated HTML lands under `docs/html/`.

- [`docs/src/ARCHITECTURE.md`](docs/src/ARCHITECTURE.md)
  Implementation-side architecture notes for transport ownership, scheduling,
  and callback behavior.
- [`docs/src/OPERATOR_QUICK_REFERENCE.md`](docs/src/OPERATOR_QUICK_REFERENCE.md)
  Operator-facing summary of the public library surface, runtime configuration,
  and manual tools.
- [`docs/src/BRIDGE_READINESS_CHECKLIST.md`](docs/src/BRIDGE_READINESS_CHECKLIST.md)
  Requirement and evidence checklist for the bridge-v1 scope.
