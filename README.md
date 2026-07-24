# goat_motor_controller

`goat_motor_controller` is the GOAT racer motor-controller transport library.
It provides a thread-safe, C++17 interface for connecting to a controller,
exchanging typed protocol messages, polling core telemetry, and sending drive
or steering commands through one transport owner.

## Purpose

This library is intended for higher-level GOAT applications that need:

- a reusable CMake package instead of application-local serial code
- one place to own transport lifecycle and serialized command writes
- typed helpers for the supported controller message set
- cached telemetry reads plus callback-based delivery of fresh samples
- firmware-native motor, application, and LispBM backup/restore primitives
- guarded firmware-7.00 local FOC calibration transport

The library does not include a ROS interface.

## Requirements

| Area | Requirement | Notes |
|---|---|---|
| Build system | CMake 3.21 or newer | See [`CMakeLists.txt`](CMakeLists.txt). |
| Compiler | C++17 compiler | |
| Target platform | Jetson Orin Nano running ARM Linux | |
| Hardware / transport | Compatible motor-controller hardware reachable over the expected serial device path | |
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
`controller_hardware_smoke` manual validation tool.

## Installed Surface

`goat_motor_controller` installs as a reusable CMake package. A clean install emits:

- public headers under `include/goat_motor_controller/`
- the `goat_motor_controller` library artifact
- CMake package metadata under `lib/cmake/goat_motor_controller/`
- `share/goat_motor_controller/package.xml`

One simple local install flow is:

```bash
cmake -S . -B build/install-export \
  -DBUILD_TESTING=OFF \
  -DGOAT_MOTOR_CONTROLLER_BUILD_EXAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/install/goat_motor_controller"
cmake --build build/install-export
cmake --install build/install-export
```

Downstream consumers can then point CMake at that install prefix with
`CMAKE_PREFIX_PATH` or `goat_motor_controller_DIR` and use the exported target:

```cmake
find_package(goat_motor_controller CONFIG REQUIRED)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app PRIVATE goat_motor_controller::goat_motor_controller)
```

## Compatibility

The library independently implements the required serial protocol subset for
controllers running firmware from the
[`vedderb/bldc`](https://github.com/vedderb/bldc) lineage. That upstream
firmware is the authority for wire-format compatibility; it is not linked into
this project.

## Documentation

Doxygen-generated API docs can be built with:

```bash
cmake --build --preset default --target docs
```

Generated HTML lands under `docs/html/`.

- [`docs/ai/README.md`](docs/ai/README.md)
  Context and change guidance for coding agents working in this repository.
- [`docs/src/ARCHITECTURE.md`](docs/src/ARCHITECTURE.md)
  Implementation-side architecture notes for transport ownership, scheduling,
  and callback behavior.
- [`docs/src/OPERATOR_QUICK_REFERENCE.md`](docs/src/OPERATOR_QUICK_REFERENCE.md)
  Operator-facing summary of the public library surface, runtime configuration,
  and manual tools.
- [`docs/src/BRIDGE_READINESS_CHECKLIST.md`](docs/src/BRIDGE_READINESS_CHECKLIST.md)
  Requirement and evidence checklist for the bridge-v1 scope.

## License

Copyright (c) 2026 Erik LaBrot. Released under the [MIT License](LICENSE).
