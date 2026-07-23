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
  - `should have later`: useful, but not a release blocker for bridge v1
  - `open decision`: important area whose final semantics are not locked yet
- `Evidence`
  - `test`: automated evidence in the repository
  - `manual validation`: operator-facing evidence on real hardware
  - `code review`: reasoning-based evidence without dedicated execution proof
  - `not yet covered`: no accepted evidence path captured yet
- `Status`
  - `covered`: acceptable evidence exists now
  - `partial`: some evidence exists, but not enough for release confidence
  - `open`: intentionally unresolved
  - `missing`: no acceptable evidence path exists yet

## Protocol Support Matrix

| Message | Comm ID | Direction | Library surface | Bridge-v1 status | Requirement / evidence |
|---|---:|---|---|---|---|
| `COMM_FW_VERSION` | 0 | request/response | `request_fw_version(...)`, `parse_fw_version(...)` | supported for diagnostics | `BRIDGE-CFG-003`, covered by test |
| `COMM_JUMP_TO_BOOTLOADER` | 1 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_ERASE_NEW_APP` | 2 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_WRITE_NEW_APP_DATA` | 3 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_GET_VALUES` | 4 | request/response | `build_get_values_request(...)`, `parse_get_values(...)` | supported | `BRIDGE-TEL-002`, covered by test |
| `COMM_SET_DUTY` | 5 | command | `set_duty(...)`, `build_set_duty_command(...)` | supported | `BRIDGE-CTRL-001`, covered by test |
| `COMM_SET_CURRENT` | 6 | command | `set_current(...)`, `build_set_current_command(...)` | supported | `BRIDGE-CTRL-005`, covered by test |
| `COMM_SET_CURRENT_BRAKE` | 7 | command | `set_current_brake(...)`, `build_set_current_brake_command(...)` | supported as bounded active braking | `BRIDGE-CTRL-004`, test + doc |
| `COMM_SET_RPM` | 8 | command | `set_rpm(...)`, `build_set_rpm_command(...)` | supported | `BRIDGE-CTRL-002`, covered by test |
| `COMM_SET_POS` | 9 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_HANDBRAKE` | 10 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_DETECT` | 11 | command | none | deferred / unsupported | not in bridge-v1 scope |
| `COMM_SET_SERVO_POS` | 12 | command | `set_servo_pos(...)`, `build_set_servo_pos_command(...)` | supported | `BRIDGE-CTRL-003`, covered by test |
| `COMM_SET_MCCONF` | 13 | request/ack | `write_motor_config(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_GET_MCCONF` | 14 | request/response | `request_motor_config(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_SET_APPCONF` | 16 | request/ack | `write_app_config(..., Persistent, ...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_GET_APPCONF` | 17 | request/response | `request_app_config(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_GET_IMU_DATA` | 65 | request/response | `build_get_imu_data_request(...)`, `parse_get_imu_data(...)` | supported | `BRIDGE-TEL-001`, covered by test |
| `COMM_LISP_READ_CODE` | 130 | request/response | `request_lisp_code(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_LISP_WRITE_CODE` | 131 | request/ack | `write_lisp_code(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_LISP_ERASE_CODE` | 132 | request/ack | `erase_lisp_code(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_LISP_SET_RUNNING` | 133 | request/ack | `set_lisp_running(...)` | supported management surface | covered by test; outside bridge-v1 |
| `COMM_SET_APPCONF_NO_STORE` | 149 | request/ack | `write_app_config(..., Volatile, ...)` | supported management surface | covered by test; outside bridge-v1 |

## Motor Control

| ID | Requirement | Rationale | Priority | Status | Evidence |
|---|---|---|---|---|---|
| BRIDGE-CTRL-001 | The library shall support duty-cycle command output for the main motor. | Bridge control needs direct open-loop throttle behavior. | must have | covered | test |
| BRIDGE-CTRL-002 | The library shall support RPM command output for the main motor. | Bridge control needs closed-loop speed control. | must have | covered | test |
| BRIDGE-CTRL-003 | The library shall support servo position output for steering or auxiliary actuation. | Bridge v1 requires servo control from the same transport owner. | must have | covered | test |
| BRIDGE-CTRL-004 | The library shall support braking for bridge control through `COMM_SET_CURRENT_BRAKE`, with client-side clamping to a configured hardware-safe brake-current limit. Coasting remains a separate zero-command behavior. | Bridge control needs an explicit active-brake path without allowing callers to exceed the vehicle's configured brake-current bound. | must have | covered | test + doc |
| BRIDGE-CTRL-005 | The library shall support current command output for the main motor. | Bridge control needs current-mode actuation for closed-loop torque behavior. | must have | covered | test |

## Telemetry

| ID | Requirement | Rationale | Priority | Status | Evidence |
|---|---|---|---|---|---|
| BRIDGE-TEL-001 | The library shall expose full IMU field support through `VescIMUData` and the IMU polling path. | The bridge needs a single typed IMU payload shape. | must have | covered | test |
| BRIDGE-TEL-002 | The library shall expose drive telemetry from `GetValues`, including `vin`, `current_in`, `current_motor`, `duty_cycle`, `rpm`, `temp_motor`, `temp_fet`, and `fault_code`. | Bridge power and health reporting depends on these fields. | must have | covered | test |
| BRIDGE-TEL-003 | The library shall provide latest-value access for bridge telemetry reads. | The bridge needs cheap pull-based access in addition to callbacks. | must have | covered | test |
| BRIDGE-TEL-004 | The library shall provide callback/subscription delivery for fresh IMU and motor-state samples. | The bridge needs push-based publishing without polling from user code. | must have | covered | test |

## Runtime / Config Behavior

| ID | Requirement | Rationale | Priority | Status | Evidence |
|---|---|---|---|---|---|
| BRIDGE-CFG-001 | The library shall allow the bridge to configure IMU and motor polling rates. | Polling cadence is controlled on the host side, not by the VESC. | must have | covered | test |
| BRIDGE-CFG-002 | The bridge-facing stack shall expose current polling/config behavior for introspection. | Operators should be able to tell what cadence/config the bridge is using. | must have | covered | test + doc |
| BRIDGE-CFG-003 | The library shall keep firmware-version query support available for bridge diagnostics. | Firmware identification is useful for compatibility and field debugging. | should have later | covered | test |

## Reliability / Safety Gates

| ID | Requirement | Rationale | Priority | Status | Evidence |
|---|---|---|---|---|---|
| BRIDGE-REL-001 | Concurrent lifecycle calls and async transport failure shall cleanly stop without leaving a joinable thread or leaked transport state. | Transport and lifecycle races must not terminate the process or leave broken client state behind. | must have | covered | test |
| BRIDGE-REL-002 | Subscription lifetime and callback failure handling shall remain safe through shutdown and reentrant stop requests. | Bridge callbacks must not terminate or deadlock the transport thread. | must have | covered | test |
| BRIDGE-REL-003 | Disconnect shall not hang if the transport thread is blocked waiting for write readiness. | The bridge must be able to stop promptly even under bad transport conditions. | must have | covered | test |
| BRIDGE-REL-004 | Poll timeouts shall recover without permanently stalling IMU or motor telemetry. | Temporary missed replies should not kill telemetry flow. | must have | covered | test |
| BRIDGE-REL-005 | Blocking query deadlines shall be honored and later firmware queries shall recover after a timeout. | Firmware diagnostics must not become permanently unavailable after a missed reply. | must have | covered | test |
| BRIDGE-REL-006 | Command and query submission shall remain truthful during disconnect races; pending control commands shall retain only the latest value per command ID and be discarded when the watchdog fires. | Bridge control needs bounded stale work and accurate submission outcomes. | must have | covered | test |
| BRIDGE-REL-007 | Sanitizer-enabled builds should run across the library, examples, and tests. | Sanitizers are a practical reliability gate for transport and lifetime bugs. | should have later | covered | build + test |

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
