# Bridge Readiness Checklist

This document defines what "bridge-ready" means for `goat_vesc` on the GOAT
autonomous racer platform for bridge v1. It is intentionally lightweight: the
goal is a practical release gate, not a formal specification.

Use this checklist for requirement coverage. Test coverage should be judged by
how well it proves these GOAT bridge-v1 requirements, not just by line
coverage.

GitHub milestone and issue state are authoritative for active work tracking and
approval. This document is the local requirement-coverage mirror for GOAT
bridge-v1 behavior.

## How To Read This

- `Priority`
  - `must have`: required for bridge v1
  - `should have later`: useful, but not a v1 release blocker
  - `open decision`: required area, but semantics are not locked yet
- `Evidence`
  - `test`
  - `manual validation`
  - `code review`
  - `not yet covered`
- `Status`
  - `covered`: acceptable evidence exists now
  - `partial`: some evidence exists, but not enough for release confidence
  - `open`: intentionally unresolved
  - `missing`: no acceptable evidence path exists yet

## Status View

| Bucket | Count |
|---|---:|
| Must-have requirements | 17 |
| Must-have requirements fully covered | 17 |
| Must-have requirements partially covered | 0 |
| Must-have requirements missing | 0 |
| Open product decisions | 0 |

All current bridge-v1 `must have` requirements are covered in the shipped code
and tests, and the related blocking bug issues are closed. Remaining release
work is checklist and milestone bookkeeping rather than known missing
bridge-v1 behavior.

## Protocol Support Matrix

| Message | Comm ID | Direction | Library surface | Bridge-v1 status | Requirement / evidence |
|---|---:|---|---|---|---|
| `COMM_FW_VERSION` | 0 | request/response | `request_fw_version(...)`, `parse_fw_version(...)` | supported for diagnostics | `BRIDGE-CFG-003`, covered by test |
| `COMM_JUMP_TO_BOOTLOADER` | 1 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_ERASE_NEW_APP` | 2 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_WRITE_NEW_APP_DATA` | 3 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_GET_VALUES` | 4 | request/response | `build_get_values_request(...)`, `parse_get_values(...)` | supported | `BRIDGE-TEL-002`, covered by test |
| `COMM_SET_DUTY` | 5 | command | `set_duty(...)`, `build_set_duty_command(...)` | supported | `BRIDGE-CTRL-001`, covered by test |
| `COMM_SET_CURRENT` | 6 | command | `set_current(...)`, `build_set_current_command(...)` | supported | `BRIDGE-CTRL-005`, covered by test |
| `COMM_SET_CURRENT_BRAKE` | 7 | command | `set_current_brake(...)`, `build_set_current_brake_command(...)` | supported as bounded active braking | `BRIDGE-CTRL-004`, test + doc |
| `COMM_SET_RPM` | 8 | command | `set_rpm(...)`, `build_set_rpm_command(...)` | supported | `BRIDGE-CTRL-002`, covered by test |
| `COMM_SET_POS` | 9 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_HANDBRAKE` | 10 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_DETECT` | 11 | command | enum only | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_SERVO_POS` | 12 | command | `set_servo_pos(...)`, `build_set_servo_pos_command(...)` | supported | `BRIDGE-CTRL-003`, covered by test |
| `COMM_GET_IMU_DATA` | 65 | request/response | `build_get_imu_data_request(...)`, `parse_get_imu_data(...)` | supported | `BRIDGE-TEL-001`, covered by test |

## Motor Control

| ID | Requirement | Rationale | Priority | Status | Evidence | Suggested GitHub issue title |
|---|---|---|---|---|---|---|
| BRIDGE-CTRL-001 | The library shall support duty-cycle command output for the main motor. | Bridge control needs direct open-loop throttle behavior. | must have | covered | test | Add test coverage for duty, brake current, and servo commands |
| BRIDGE-CTRL-002 | The library shall support RPM command output for the main motor. | Bridge control needs closed-loop speed control. | must have | covered | test | Add parser/framing edge-case test coverage; Add lifecycle/concurrency coverage for connect/disconnect and polling updates |
| BRIDGE-CTRL-003 | The library shall support servo position output for steering or auxiliary actuation. | Bridge v1 requires servo control from the same transport owner. | must have | covered | test | Add test coverage for duty, brake current, and servo commands |
| BRIDGE-CTRL-004 | The library shall support braking for bridge control through `COMM_SET_CURRENT_BRAKE`, with client-side clamping to a configured hardware-safe brake-current limit. Coasting remains a separate zero-command behavior. | Bridge control needs an explicit active-brake path without allowing callers to exceed the vehicle's configured brake-current bound. | must have | covered | test + doc | Bridge-safe stale-command watchdog applies configured brake current as a one-shot safe-stop |
| BRIDGE-CTRL-005 | The library shall support current command output for the main motor. | Bridge control needs current-mode actuation for closed-loop torque behavior. | must have | covered | test | Add test coverage for duty, brake current, and servo commands |

## Telemetry

| ID | Requirement | Rationale | Priority | Status | Evidence | Suggested GitHub issue title |
|---|---|---|---|---|---|---|
| BRIDGE-TEL-001 | The library shall expose full IMU field support through `VescIMUData` and the IMU polling path. | The bridge needs a single typed IMU payload shape. | must have | covered | test | Add parser/framing edge-case test coverage; Add lifecycle/concurrency coverage for connect/disconnect and polling updates |
| BRIDGE-TEL-002 | The library shall expose drive telemetry from `GetValues`, including `vin`, `current_in`, `current_motor`, `duty_cycle`, `rpm`, `temp_motor`, `temp_fet`, and `fault_code`. | Bridge power and health reporting depends on these fields. | must have | covered | test | Add parser/framing edge-case test coverage; Add lifecycle/concurrency coverage for connect/disconnect and polling updates |
| BRIDGE-TEL-003 | The library shall provide latest-value access for bridge telemetry reads. | The bridge needs cheap pull-based access in addition to callbacks. | must have | covered | test | Add lifecycle/concurrency coverage for connect/disconnect and polling updates |
| BRIDGE-TEL-004 | The library shall provide callback/subscription delivery for fresh IMU and motor-state samples. | The bridge needs push-based publishing without polling from user code. | must have | covered | test | Add lifecycle/concurrency coverage for connect/disconnect and polling updates; Fix `SubscriptionHandle` lifetime and ownership safety |

## Runtime / Config Behavior

| ID | Requirement | Rationale | Priority | Status | Evidence | Suggested GitHub issue title |
|---|---|---|---|---|---|---|
| BRIDGE-CFG-001 | The library shall allow the bridge to configure IMU and motor polling rates. | Polling cadence is controlled on the host side, not by the VESC. | must have | covered | test | Add lifecycle/concurrency coverage for connect/disconnect and polling updates |
| BRIDGE-CFG-002 | The bridge-facing stack shall expose current polling/config behavior for introspection. | Operators should be able to tell what cadence/config the bridge is using. | must have | covered | test + doc | Expose current polling/config behavior for bridge introspection |
| BRIDGE-CFG-003 | The library shall keep firmware-version query support available for bridge diagnostics. | Firmware identification is useful for compatibility and field debugging. | should have later | covered | test | Add parser/framing edge-case test coverage; Add lifecycle/concurrency coverage for connect/disconnect and polling updates |

## Reliability / Safety Gates

| ID | Requirement | Rationale | Priority | Status | Evidence | Suggested GitHub issue title |
|---|---|---|---|---|---|---|
| BRIDGE-REL-001 | The library shall cleanly shut down after async transport failure without leaving a joinable thread or leaked transport state. | Transport faults must not terminate the process or leave broken client state behind. | must have | covered | test | Ensure disconnect fully cleans up after async transport failure |
| BRIDGE-REL-002 | Subscription lifetime handling shall remain safe if subscription handles outlive client shutdown. | Bridge code should not trigger use-after-free by normal teardown ordering. | must have | covered | test | Fix `SubscriptionHandle` lifetime and ownership safety |
| BRIDGE-REL-003 | Disconnect shall not hang if the transport thread is blocked waiting for write readiness. | The bridge must be able to stop promptly even under bad transport conditions. | must have | covered | test | Make disconnect unblock writes and avoid shutdown hangs |
| BRIDGE-REL-004 | Poll timeouts shall recover without permanently stalling IMU or motor telemetry. | Temporary missed replies should not kill telemetry flow. | must have | covered | test | Harden IMU/motor poll timeout and recovery semantics |
| BRIDGE-REL-005 | Blocking query deadlines shall be honored, and stale late replies shall not satisfy a newer request. | Bridge diagnostics must not return misleading results after timing faults. | must have | covered | test | Enforce per-query deadlines and reject stale late replies |
| BRIDGE-REL-006 | Command submission results shall truthfully reflect whether a command can still be delivered during disconnect races. | Bridge control logic needs accurate command-send outcomes. | must have | covered | test | Prevent commands from reporting success when dropped during disconnect |
| BRIDGE-REL-007 | Sanitizer-enabled builds should run across the library, examples, and tests. | Sanitizers are a practical reliability gate for transport and lifetime bugs. | should have later | covered | build + test | none |

## Coverage Policy

Requirement coverage for bridge v1 means:

- every `must have` requirement has at least one evidence path
- every bridge-v1 public API path used by the bridge has a success-path test
- every reliability requirement has a failure-path or regression test
- every bug fixed on the bridge-v1 path gets a regression test tied to the
  relevant requirement or bug issue

"More or less 100% coverage" for this repo should mean near-complete coverage of
bridge-v1 requirements and their public behavior. It does not require literal
100% line coverage for unrelated or deferred code paths.

## Out Of Scope For This Draft

- Full-library requirements beyond bridge v1
- Additional `GetValues` fields beyond the agreed bridge-v1 telemetry set
- Deep runtime diagnostics beyond config introspection
