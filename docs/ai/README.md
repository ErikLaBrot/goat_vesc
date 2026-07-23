# goat_vesc AI Context

This is the entry point for coding agents changing `goat_vesc`. It supplies
repository context and routes readers to the existing API and operator
documentation rather than repeating it.

## Project Boundary

`goat_vesc` is a C++17 library that owns one VESC serial transport, translates
between typed values and VESC packets, schedules telemetry and diagnostic
requests, and exposes cached or callback-based telemetry. It installs as a
reusable CMake package for higher-level GOAT applications.

This repository does not own:

- the ROS adapter or application-level control policy
- VESC firmware, calibration, or controller-side timeout configuration
- selection or coordination of multiple processes competing for one controller

Start with the [project README](../../README.md) for the supported public
surface and install contract.

## Protocol Authority

`goat_vesc` independently implements a supported subset of the VESC serial
`COMM_*` protocol defined by the upstream
[`vedderb/bldc`](https://github.com/vedderb/bldc) firmware. It does not build or
link against that firmware.

The target controller hardware is constrained to this firmware lineage, so the
upstream source is authoritative for command IDs, payload layouts, scaling, and
configuration serialization. Verify new protocol support against the exact
firmware version deployed on the controller; this repository does not currently
pin one.

## Layers And Responsibilities

| Layer | Responsibility | Location |
|---|---|---|
| Public contract | Consumer-facing configuration, data types, lifecycle, control, telemetry, and query APIs | [`include/goat_vesc/`](../../include/goat_vesc/) |
| Packet framing | Integral field serialization, frame boundaries, CRC validation, and byte-stream resynchronization | [`packet_builder.cpp`](../../src/packet_builder.cpp), [`packet_parser.cpp`](../../src/packet_parser.cpp) |
| Message protocol | Command IDs, field masks, scaling, and typed request/response encoding | [`protocol_ids.hpp`](../../include/goat_vesc/protocol_ids.hpp), [`protocol.cpp`](../../src/protocol.cpp) |
| Transport and scheduling | Serial lifecycle, work arbitration, timeouts, reply matching, caches, callbacks, and shutdown | [`vesc_client.cpp`](../../src/vesc_client.cpp) |
| Automated evidence | Exact wire-format tests and fake-transport lifecycle, concurrency, and failure tests | [`test_protocol.cpp`](../../tests/test_protocol.cpp), [`test_client.cpp`](../../tests/test_client.cpp) |
| Manual validation | Operator-visible probe, sweep, and hardware smoke workflows | [`examples/`](../../examples/), [`scripts/`](../../scripts/) |

## Operation Lifecycles

| Operation | Lifecycle |
|---|---|
| Control command | A public method encodes a packet under the protocol lock and submits it to the command queue. Control commands have scheduler priority, and the I/O thread writes them without tracking a reply. Accepted control commands refresh the optional watchdog. |
| One-shot query | Each request carries an absolute deadline from submission; the request queue holds the packet, expected reply ID, and completion callbacks. Only one reply-bearing request is in flight. A matching reply completes it, while queued or in-flight requests can expire. The current firmware query also tracks and discards a late reply after timeout. |
| Periodic telemetry | IMU and motor-state poll channels become due independently. The scheduler sends a poll when no reply-bearing request is in flight, then the I/O thread decodes and timestamps the reply, updates the latest-value cache, and publishes callbacks outside internal locks. IMU wins a tie between due channels. |

All transport traffic converges on the same I/O thread and packet parser.

See [Architecture Notes](../src/ARCHITECTURE.md) for the detailed scheduling,
watchdog, cache, and callback model.

## Invariants To Preserve

- One active process owns a controller transport; inside `VescClient`, one I/O
  thread performs all serial reads and writes.
- At most one reply-bearing request is in flight, so replies can be matched by
  expected packet ID.
- Control commands take priority over periodic polls and diagnostic queries.
- User callbacks run outside cache and registry locks and should remain
  lightweight.
- Subscription handles may outlive client shutdown without accessing destroyed
  client state.
- The optional host watchdog is one-shot, disabled by default, and cannot stop
  hardware after transport loss. VESC-side timeout configuration remains the
  backstop.
- Repository automation must not touch real VESC hardware. Hardware access and
  actuator commands require explicit authorization.

## Sources Of Truth

- Public contract: [`include/goat_vesc/`](../../include/goat_vesc/)
- Runtime behavior: [`src/`](../../src/)
- Executable evidence: [`tests/`](../../tests/)
- Bridge-v1 requirements and status:
  [Bridge Readiness Checklist](../src/BRIDGE_READINESS_CHECKLIST.md)
- Operator behavior and hardware tools:
  [Operator Quick Reference](../src/OPERATOR_QUICK_REFERENCE.md)

When prose and behavior disagree, verify the public header, implementation, and
tests together, then update the stale documentation in the same change.

For change locations and validation, continue to the
[Change Guide](CHANGE_GUIDE.md).
