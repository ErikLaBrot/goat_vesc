# goat_vesc

`goat_vesc` is a small C++ client library for talking to a Vedder VESC over the serial protocol.

The library is intended to act like a device-driver layer for a larger server, ROS node, or ZMQ bridge:

- one class owns the serial port
- periodic IMU and motor-state polling is scheduled internally
- decoded samples are timestamped in the library
- callers can either read the latest cached value or subscribe to fresh samples
- infrequent query traffic is serialized so it does not permanently break periodic polling

## Design Summary

`VescClient` owns:

- the serial file descriptor
- one background I/O thread
- the periodic poll schedule
- request arbitration
- sample timestamping
- latest-value caches

Public callers use a much smaller API:

- `connect()` / `disconnect()`
- `latest_imu()` / `latest_motor_state()`
- `subscribe_imu(...)` / `subscribe_motor_state(...)`
- motor control commands like `set_current(...)`
- one-shot blocking queries like `request_fw_version(...)`

## Safety and Control Behavior

### Do control commands collide?

At the library level, no byte-level collision should happen.

All writes are serialized through a single transport thread. If multiple threads call:

- `set_rpm(...)`
- `set_duty(...)`
- `set_current(...)`
- `set_current_brake(...)`

those requests are converted into packets and pushed into the command queue. The I/O thread drains that queue in FIFO order before issuing periodic poll traffic.

That means:

- command bytes are not interleaved on the serial line
- a poll request will not cut through the middle of a control packet
- multiple callers can enqueue commands safely

What is still possible is semantic override:

- if one thread sends `set_duty(0.2)` and another immediately sends `set_duty(0.6)`, both will be sent, and the later command wins because it arrives later
- if callers mix incompatible control modes, such as alternating RPM and duty commands, the VESC will receive those changes in order

So the transport is serialized, but the application still needs one clear control authority.

### What happens if the serial connection drops?

If the client detects a transport failure:

- `running_` is cleared
- the I/O loop exits
- outstanding blocking queries complete with `std::nullopt`
- future command submissions return `false`
- a command API returns `true` only while the command is still deliverable when the call returns
- no more polling or command traffic is sent

What the library does **not** currently do:

- it does not repeatedly refresh a safe-stop command after the watchdog fires
- it cannot send a final stop command after the serial link is already gone

### Is there timeout safety for remote control dropout?

Yes, if you enable the watchdog in `VescConfig`.

The client now supports an opt-in stale-command watchdog for bridge-style control loops:

- set `command_watchdog_timeout` to a positive duration
- choose `command_watchdog_action`
- if using `BrakeCurrent`, set `command_watchdog_brake_current` to a positive amp value

When enabled, each fresh control command re-arms the watchdog. If no further control command arrives before the timeout, the library sends one safe-stop command:

- `ControlWatchdogAction::Coast` sends `COMM_SET_CURRENT` with `0.0 A`
- `ControlWatchdogAction::BrakeCurrent` sends `COMM_SET_CURRENT_BRAKE` with the configured brake current

The watchdog is intentionally one-shot. After it fires, a new control command must arrive to arm it again.

If you leave the watchdog disabled, the previous behavior still applies: if your remote operator stops sending commands but the serial link and process are still alive, the last command can remain active until something else changes it. Whether the motor actually keeps running depends on the VESC firmware/app configuration.

For a remote-control use case, you should treat this as a required safety feature and not as optional polish.

Recommended safety layers:

1. Configure the VESC-side timeout/watchdog so stale control commands decay to zero or brake.
2. Enable the library watchdog when you want the host transport owner to enforce a one-shot safe-stop on stale input.
3. Add an application-level command heartbeat in your server/ROS node.
4. If the heartbeat expires, explicitly command a safe output such as zero current or brake current.

The important constraint is this:

- once the link is already lost, the library cannot rely on sending one last stop command

so the final authority for stale-command safety should still live on the VESC side or in a watchdog mechanism that is continuously refreshed while control is healthy.

## API Overview

Main type: [`goat_vesc::VescClient`](include/goat_vesc/vesc_client.hpp)

Configuration: [`goat_vesc::VescConfig`](include/goat_vesc/types.hpp)

Bridge introspection snapshot: [`goat_vesc::VescClientConfigSnapshot`](include/goat_vesc/types.hpp)

Watchdog configuration fields:

- `std::chrono::milliseconds command_watchdog_timeout`
- `ControlWatchdogAction command_watchdog_action`
- `float command_watchdog_brake_current`

Primary methods:

- `bool connect()`
- `void disconnect()`
- `bool is_connected() const`
- `VescClientConfigSnapshot config_snapshot() const`
- `std::optional<VescIMUData> latest_imu() const`
- `std::optional<VescMotorState> latest_motor_state() const`
- `SubscriptionHandle subscribe_imu(ImuCallback cb)`
- `SubscriptionHandle subscribe_motor_state(MotorStateCallback cb)`
- `bool set_rpm(std::int32_t rpm)`
- `bool set_duty(float duty)`
- `bool set_current(float amps)`
- `bool set_current_brake(float amps)`
- `std::optional<FwVersion> request_fw_version(std::chrono::milliseconds timeout)`

## Timing Model

The library uses:

- `steady_clock` for internal scheduling
- a wall-clock callback for sample timestamps

This means:

- polling cadence is based on monotonic scheduling
- published/cached timestamps can still line up with ROS/system time

Decoded IMU and motor-state samples are stamped when the full response packet is received and decoded by the I/O thread.

`config_snapshot()` exposes the current bridge-facing scheduling and watchdog configuration, including live poll intervals after any `set_*_poll_interval(...)` updates. This gives bridge/operator code a cheap way to report the active cadence and safety settings without reaching into transport internals.

## Typical Usage

```cpp
#include "goat_vesc/vesc_client.hpp"

using namespace goat_vesc;
using namespace std::chrono_literals;

int main() {
    VescConfig config;
    config.device_path = "/dev/ttyACM0";
    config.imu_poll_interval = 10ms;
    config.motor_poll_interval = 50ms;
    config.command_watchdog_timeout = 100ms;
    config.command_watchdog_action = ControlWatchdogAction::BrakeCurrent;
    config.command_watchdog_brake_current = 3.0f;

    VescClient client(config);
    if (!client.connect()) {
        return 1;
    }

    const auto snapshot = client.config_snapshot();
    if (snapshot.command_watchdog_timeout > std::chrono::milliseconds{0}) {
        // publish configured safety behavior to diagnostics
    }

    auto imu_sub = client.subscribe_imu([](const VescIMUData& imu) {
        // publish to ROS here
        (void)imu;
    });

    auto motor_sub = client.subscribe_motor_state([](const VescMotorState& state) {
        // publish telemetry here
        (void)state;
    });

    if (!client.set_current(3.0f)) {
        client.disconnect();
        return 1;
    }

    if (auto fw = client.request_fw_version(200ms)) {
        // use fw->major / fw->minor
    }

    client.disconnect();
    return 0;
}
```

## Recommended ROS Pattern

For a ROS node:

- let `VescClient` own polling cadence
- use subscriptions to publish fresh IMU and motor-state messages
- use `latest_*()` for services, diagnostics, or lazy reads
- keep one clear control authority that issues motor commands
- enable the client watchdog or an application/VESC-side watchdog for stale remote control input

This keeps protocol timing in one place and avoids ROS-side request traffic from perturbing IMU timing.

## Build and Test

```bash
cmake -S . -B build/goat_vesc
cmake --build build/goat_vesc
ctest --test-dir build/goat_vesc --output-on-failure
```

Static analysis gate:

```bash
cmake -S . -B build/goat_vesc-analysis \
  -DGOAT_VESC_BUILD_TESTS=ON \
  -DGOAT_VESC_BUILD_TOOLS=ON \
  -DGOAT_VESC_ENABLE_STRICT_MODE=ON
cmake --build build/goat_vesc-analysis
ctest --test-dir build/goat_vesc-analysis --output-on-failure
```

Sanitizer gate:

```bash
cmake -S . -B build/goat_vesc-sanitized \
  -DGOAT_VESC_BUILD_TESTS=ON \
  -DGOAT_VESC_BUILD_TOOLS=ON \
  -DGOAT_VESC_ENABLE_WERROR=ON \
  -DGOAT_VESC_ENABLE_ASAN=ON \
  -DGOAT_VESC_ENABLE_LSAN=ON \
  -DGOAT_VESC_ENABLE_UBSAN=ON
cmake --build build/goat_vesc-sanitized
ctest --test-dir build/goat_vesc-sanitized --output-on-failure
```

Thread sanitizer gate:

```bash
cmake -S . -B build/goat_vesc-tsan \
  -DGOAT_VESC_BUILD_TESTS=ON \
  -DGOAT_VESC_BUILD_TOOLS=ON \
  -DGOAT_VESC_ENABLE_WERROR=ON \
  -DGOAT_VESC_ENABLE_TSAN=ON
cmake --build build/goat_vesc-tsan
ctest --test-dir build/goat_vesc-tsan --output-on-failure
```

Clang-only thread sanitizer gate:

```bash
cmake -S . -B build/goat_vesc-tsan-clang \
  -DCMAKE_CXX_COMPILER=clang++-18 \
  -DGOAT_VESC_BUILD_TESTS=ON \
  -DGOAT_VESC_BUILD_TOOLS=ON \
  -DGOAT_VESC_ENABLE_TSAN=ON
cmake --build build/goat_vesc-tsan-clang
ctest --test-dir build/goat_vesc-tsan-clang --output-on-failure
```

`ThreadSanitizer` should run as its own build. Do not combine it with
`AddressSanitizer`, `LeakSanitizer`, or `UndefinedBehaviorSanitizer`.

If the default compiler is GCC and TSAN aborts with `unexpected memory mapping`,
prefer the Clang-only TSAN lane above. It changes compilers for that build only;
the rest of the project can keep using GCC.

Convenience presets:

```bash
cmake --preset strict-analysis
cmake --build --preset strict-analysis
ctest --preset strict-analysis

cmake --preset asan-ubsan-lsan
cmake --build --preset asan-ubsan-lsan
ctest --preset asan-ubsan-lsan

cmake --preset tsan
cmake --build --preset tsan
ctest --preset tsan

cmake --preset tsan-clang
cmake --build --preset tsan-clang
ctest --preset tsan-clang
```

Repo quality gate:

```bash
./scripts/run_quality_gate.sh
```

Optional TSAN lane:

```bash
./scripts/run_quality_gate.sh --with-tsan
```

Optional Clang-only TSAN lane:

```bash
./scripts/run_quality_gate.sh --with-tsan-clang
```

`GOAT_VESC_ENABLE_STRICT_MODE` enables the repo's stricter local gate:

- compiler warnings-as-errors
- `clang-tidy` with warnings promoted to errors
- `cppcheck` in exhaustive mode without suppressing `useStlAlgorithm`
- format and whitespace checks through the quality-gate script

## Backlog Workflow

Development work is tracked as short-lived issue branches, with release scope
tracked by the GitHub milestone:

- do not pre-create branches
- default workflow is `1 issue = 1 branch = 1 PR`
- derive branch names from the issue using `issue-<number>-<short-slug>`
- only create a branch when you are ready to start that issue
- merge and delete the branch before starting the next substantial branch
- use the current v1.0 milestone to indicate release membership
- use labels such as `bug`, `feature`, `test`, `docs`, and `chore` to classify
  work item type instead of encoding type in the branch name

The current release-definition and requirement coverage reference lives in
[`BRIDGE_READINESS_CHECKLIST.md`](BRIDGE_READINESS_CHECKLIST.md). Once work
items are opened on GitHub, milestone-assigned issues are the active backlog.

## Bridge Readiness

Bridge-v1 release requirements and requirement coverage are tracked locally in
[`BRIDGE_READINESS_CHECKLIST.md`](BRIDGE_READINESS_CHECKLIST.md), while the
GitHub milestone remains authoritative for active release tracking.

## Example

A minimal example program is provided in [`tools/vesc_example.cpp`](tools/vesc_example.cpp).
