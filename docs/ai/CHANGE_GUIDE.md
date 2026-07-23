# goat_vesc Change Guide

Read the [AI context index](README.md) first. Keep changes at the shared layer
that owns the behavior, then update its executable evidence and affected
documentation.

## Change Map

| Change | Primary locations | Evidence and context |
|---|---|---|
| Packet framing, CRC, resynchronization | [`protocol.cpp`](../../src/protocol.cpp), [`packet_parser.cpp`](../../src/packet_parser.cpp) | [`test_protocol.cpp`](../../tests/test_protocol.cpp) |
| Message IDs, masks, typed payloads | [`protocol_ids.hpp`](../../include/goat_vesc/protocol_ids.hpp), [`types.hpp`](../../include/goat_vesc/types.hpp), [`protocol.cpp`](../../src/protocol.cpp) | [`test_protocol.cpp`](../../tests/test_protocol.cpp), [readiness checklist](../src/BRIDGE_READINESS_CHECKLIST.md) |
| Transport lifecycle, scheduling, polling, queries | [`vesc_client.hpp`](../../include/goat_vesc/vesc_client.hpp), [`vesc_client.cpp`](../../src/vesc_client.cpp) | [`test_client.cpp`](../../tests/test_client.cpp), [architecture notes](../src/ARCHITECTURE.md) |
| Watchdog, braking, callbacks, shutdown safety | [`types.hpp`](../../include/goat_vesc/types.hpp), [`vesc_client.cpp`](../../src/vesc_client.cpp) | [`test_client.cpp`](../../tests/test_client.cpp), [architecture notes](../src/ARCHITECTURE.md) |
| Build, install, and exported package | [`CMakeLists.txt`](../../CMakeLists.txt), [`cmake/`](../../cmake/), [`package.xml`](../../package.xml) | [README install flow](../../README.md#installed-surface) |
| Operator or hardware workflow | [`examples/`](../../examples/) | [operator quick reference](../src/OPERATOR_QUICK_REFERENCE.md); manual validation only |

Before changing shared behavior, find every caller and related test. For
example:

```bash
rg "symbol_name" include src examples tests
```

Protocol changes normally require updating the public declaration, encoder or
decoder, exact-byte or rejection tests, and the support matrix. Transport or
safety changes normally require a focused `test_client` case covering failure
or teardown behavior.

## Automated Validation

Run from the repository root:

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default
git diff --check
```

If Doxygen is installed when CMake configures the project, also run:

```bash
cmake --build --preset default --target docs
```

The default automated tests use local fake transports. They do not require a
controller.

## Hardware Boundary

Do not run examples against a VESC unless the user explicitly
authorizes hardware access and identifies the target. Treat actuator arming as
a separate consequential action requiring explicit authorization. Start with
the non-actuating probe when that is sufficient; use the smoke tool's actuator
mode only when the requested validation requires motion.
