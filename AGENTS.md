# goat_vesc Agent Guidance

Read the [AI context index](docs/ai/README.md) before changing this repository.

- Preserve one active transport owner and keep serial writes on its I/O thread.
- Trace all callers before changing shared protocol, scheduling, or safety code.
- Run the configured build and automated tests described in the
  [change guide](docs/ai/CHANGE_GUIDE.md).
- Do not connect to VESC hardware or run hardware tools without explicit
  authorization. Actuator commands require separate explicit authorization.
