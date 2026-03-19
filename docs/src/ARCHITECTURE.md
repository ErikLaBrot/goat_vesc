# goat_vesc Architecture Notes

This document is the implementation-side companion to the public API docs for
the GOAT racer VESC transport layer. The public headers describe what the
library exposes. This page explains the core subsystems and the invariants that
shape the implementation.

## Transport Ownership

`goat_vesc::VescClient` owns the serial transport and keeps all reads and writes
on a single background thread. Public methods can be called from multiple
threads, but they do not write directly to the file descriptor. Instead they:

- build packets under `protocol_mutex_`
- hand queued work to the scheduler under `scheduler_mutex_`
- wake the transport thread through `wake_pipe_`

This keeps byte-stream ownership centralized and avoids interleaving writes from
multiple callers.

## Scheduling and Query Arbitration

The I/O loop handles three categories of work:

1. fire-and-forget control commands
2. periodic IMU and motor-state polls
3. blocking reply-bearing queries such as firmware version requests

The scheduler enforces one in-flight reply-bearing request at a time so replies
can be matched by expected packet ID without a more complicated correlator.
Control commands have priority over polls and queries. Polls are scheduled from
their own channels, and IMU polling wins ties over motor-state polling.

One-shot queries are delayed or timed out rather than allowed to permanently
disturb periodic polling. Late replies from timed-out blocking queries are
counted and dropped so a stale response cannot satisfy a newer request.

## Protocol Layering

The wire-format code is split into three public pieces:

- `VescPacketBuilder` serializes typed integer fields into payload bytes
- `VescPacketParser` consumes raw bytes and emits validated payload frames
- `VescProtocol` builds typed requests and parses typed responses

`VescClient` depends on these pieces but does not own the byte-layout details of
individual messages. That separation keeps transport logic independent from
packet layout and message semantics.

## Watchdog and Safety Model

The optional command watchdog is a host-side safety mechanism for GOAT
bridge-style control loops. Each accepted control command can arm or refresh a
deadline. When the deadline expires, the library sends one safe-stop command
chosen by configuration:

- coast by commanding zero current
- active brake by commanding bounded brake current

The watchdog is intentionally one-shot. It does not replace VESC-side timeout
configuration, and it cannot send a final command after the transport has
already failed.

## Caches and Callbacks

Decoded telemetry samples are timestamped in the I/O thread, copied into
latest-value caches, and then published to subscriber callbacks outside the
cache lock. This gives callers two access patterns:

- pull-based reads through `latest_imu()` and `latest_motor_state()`
- push-based updates through subscriptions

Callbacks are copied out of the registry before invocation so user code does not
run while the registry mutex is held.
