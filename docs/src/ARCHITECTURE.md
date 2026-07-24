# goat_motor_controller Architecture Notes

This document is the implementation-side companion to the public API docs for
the GOAT racer motor-controller transport layer. The public headers describe what the
library exposes. This page explains the core subsystems and the invariants that
shape the implementation.

## Transport Ownership

`goat_motor_controller::ControllerClient` owns the serial transport and keeps all reads and writes
on a single background thread. Public methods can be called from multiple
threads, but they do not write directly to the file descriptor. Instead they:

- build packets with stateless protocol helpers
- hand queued work to the scheduler under `scheduler_mutex_`
- wake the transport thread through `wake_pipe_`

This keeps byte-stream ownership centralized and avoids interleaving writes from
multiple callers.

## Scheduling and Query Arbitration

The I/O loop handles three categories of work:

1. fire-and-forget control commands
2. periodic IMU and motor-state polls
3. blocking reply-bearing diagnostic and management requests

The scheduler enforces one in-flight reply-bearing request at a time so replies
can be matched by expected packet ID without a more complicated correlator.
Control commands have priority over polls and queries. The queue retains only
the newest pending command for each command ID, and the I/O loop writes at most
one queued command per pass so reply timeouts and polls still progress. Polls
are scheduled from their own channels, and IMU polling wins ties over
motor-state polling.

One-shot requests are delayed or timed out rather than allowed to permanently
disturb periodic polling. Firmware version is stable during an owned connection,
so its diagnostic query can safely recover after a late reply. Configuration and
LispBM replies describe or acknowledge mutable state; a sent management request
that times out stops the connection before newer stateful work can be submitted.

## Configuration And LispBM Management

Motor and application configurations remain firmware-native byte images. Their
first four bytes are the firmware-generated schema signature. A write first
reads the active image and rejects a different signature, then sends the image
and waits for the firmware acknowledgement. Motor writes are persistent;
application writes explicitly choose volatile or persistent storage.

One management mutex spans each complete public operation so another caller
cannot interleave a preflight read with its write or split a LispBM transfer.
LispBM reads assemble bounded chunks. Writes add the firmware length, CRC, and
current zero flags, erase existing code, then validate every chunk
acknowledgement and offset. Upload does not start LispBM; execution changes use
the separate explicit operation.

## FOC Calibration

`run_foc_calibration(...)` is a firmware-7.00 transport primitive, not an
operator wizard. It holds the management mutex, queues bounded application
output suppression, and sends `COMM_DETECT_APPLY_ALL_FOC` with CAN detection
disabled. The firmware performs the blocking measurement and persists the
resulting motor configuration. Its packet contract is pinned to
[`vedderb/bldc` `9ff7e2e`](https://github.com/vedderb/bldc/blob/9ff7e2ef1d3a507e588eeca3fa05516d642f0e35/comm/commands.c).

The firmware can emit unsolicited motor and application configuration payloads
before the final signed result. The scheduler ignores those payloads while
waiting for the calibration reply ID. A well-formed negative result is returned
to the caller; a timeout stops the connection. When still connected, the client
queues an explicit zero-duration command to reenable application output.

## Protocol Layering

The wire-format code is split into two public pieces:

- `PacketParser` consumes raw bytes and emits validated payload frames
- `ControllerProtocol` serializes and frames typed requests, then parses typed responses

`ControllerClient` depends on these pieces but does not own the byte-layout details of
individual messages. That separation keeps transport logic independent from
packet layout and message semantics. The firmware framing does not escape length bytes,
so a corrupted length can consume later frames within one declared candidate
before a subsequent frame restores parser synchronization.

## Watchdog and Safety Model

The optional command watchdog is a host-side safety mechanism for GOAT
bridge-style control loops. Each accepted control command can arm or refresh a
deadline. When the deadline expires, the library sends one safe-stop command
chosen by configuration:

- coast by commanding zero current
- active brake by commanding bounded brake current

The watchdog is intentionally one-shot. It does not replace firmware-side timeout
configuration, and it cannot send a final command after the transport has
already failed. When it fires, it discards pending stale control commands before
writing the configured safe-stop command.

## Caches and Callbacks

Decoded telemetry samples are timestamped in the I/O thread, copied into
latest-value caches, and then published to subscriber callbacks outside the
cache lock. This gives callers two access patterns:

- pull-based reads through `latest_imu()` and `latest_motor_state()`
- push-based updates through subscriptions

Callbacks are copied out of the registry before invocation so user code does not
run while the registry mutex is held. They run on the I/O thread: exceptions are
ignored per subscriber, blocking queries fail immediately, and callback-initiated
disconnect requests stop the loop without attempting to join the current
thread. Slow callbacks delay all transport work, a copied callback may run once
after its subscription resets, and the client must not be destroyed from its own
callback.

## Runtime Configuration Boundary

`ControllerConfig` provides the runtime boundary between higher-level applications and
the transport layer. The caller decides:

- device path and baud
- IMU and motor polling cadence
- periodic-poll reply timeout and query guard window
- optional watchdog behavior and brake-current limits
- optional timestamp and transport hooks for tests or alternate backends

Once the client is constructed, the transport thread owns the operational state
behind those settings. Host runtime-setting mutation remains limited to poll
interval updates. Controller configuration and LispBM management are separate
blocking operations over the same transport owner.
