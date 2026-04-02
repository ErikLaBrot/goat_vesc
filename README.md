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

This will build the ['examples/'](examples/). These include focused manual
examples plus a full real-hardware smoke tool for validating telemetry,
subscriptions, and live command streaming against a real controller.

## Install And Export

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

## API Docs

Doxygen-generated API docs can be built with:

```bash
cmake --build --preset default --target docs
```

Generated HTML lands under `docs/html/`.

For implementation-side context, see
[`docs/src/ARCHITECTURE.md`](docs/src/ARCHITECTURE.md).

For a short operator-facing overview of commands, telemetry, config options, and
manual smoke tools, see
[`docs/src/OPERATOR_QUICK_REFERENCE.md`](docs/src/OPERATOR_QUICK_REFERENCE.md).

## Release / Workflow Notes

Bridge readiness requirements, release tracking context, and milestone-oriented
workflow notes live in
[`docs/src/BRIDGE_READINESS_CHECKLIST.md`](docs/src/BRIDGE_READINESS_CHECKLIST.md).
