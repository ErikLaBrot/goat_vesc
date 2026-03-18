#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/types.hpp"
#include "goat_vesc/vesc_protocol.hpp"

namespace goat_vesc {

/**
 * Thread-safe VESC client with a single transport owner thread.
 *
 * Design goals:
 * - Hide packet framing/parsing behind a small typed API.
 * - Keep all serial reads and writes on one background thread.
 * - Preserve deterministic polling for periodic data like IMU and motor state.
 * - Allow infrequent one-shot queries without permanently disturbing poll cadence.
 *
 * Scheduling model:
 * - Fire-and-forget control commands are highest priority.
 * - Periodic IMU polling is checked before motor-state polling.
 * - Only one reply-bearing request is allowed in flight at a time.
 *
 * Safety note:
 * - This class detects transport loss and stops queueing/sending commands, but it
 *   does not implement a motor-control watchdog that actively commands zero output
 *   when upstream control traffic stops. Use VESC-side timeout settings or add an
 *   application heartbeat/watchdog if loss-of-command must force a stop.
 */
class VescClient {
public:
  using ImuCallback = std::function<void(const VescIMUData&)>;
  using MotorStateCallback = std::function<void(const VescMotorState&)>;

  /**
   * RAII token for a subscription created by subscribe_imu() or
   * subscribe_motor_state(). Destroying or resetting the handle unregisters the
   * callback.
   */
  class SubscriptionHandle {
  public:
    SubscriptionHandle() = default;
    SubscriptionHandle(const SubscriptionHandle&) = delete;
    SubscriptionHandle& operator=(const SubscriptionHandle&) = delete;

    SubscriptionHandle(SubscriptionHandle&& other) noexcept;
    SubscriptionHandle& operator=(SubscriptionHandle&& other) noexcept;

    ~SubscriptionHandle();

    void reset();
    explicit operator bool() const {
      return unsubscribe_ != nullptr;
    }

  private:
    friend class VescClient;
    explicit SubscriptionHandle(std::function<void()> unsubscribe);

    std::function<void()> unsubscribe_;
  };

  /**
   * Creates a client with the given transport, timing, and timestamp settings.
   *
   * The client is inactive until connect() is called.
   */
  explicit VescClient(VescConfig config);
  ~VescClient();

  VescClient(const VescClient&) = delete;
  VescClient& operator=(const VescClient&) = delete;

  /** Returns currently visible `/dev/ttyACM*` candidates. */
  static std::vector<std::string> find_devices();

  /**
   * Opens the configured transport and starts the background I/O thread.
   *
   * If `config.device_path` is empty and no custom `open_serial_fn` is supplied,
   * the first `/dev/ttyACM*` device is used.
   */
  bool connect();

  /**
   * Stops the background thread and closes the transport.
   *
   * Outstanding blocking queries are completed with `std::nullopt`.
   */
  void disconnect();

  /** True while the client believes the transport thread is active. */
  bool is_connected() const;

  /** Updates the periodic motor-state poll interval. `0 ms` disables polling. */
  void set_motor_poll_interval(std::chrono::milliseconds interval);
  /** Updates the periodic IMU poll interval. `0 ms` disables polling. */
  void set_imu_poll_interval(std::chrono::milliseconds interval);

  /** Returns the latest cached motor-state sample, if any. */
  std::optional<VescMotorState> latest_motor_state() const;
  /** Returns the latest cached IMU sample, if any. */
  std::optional<VescIMUData> latest_imu() const;

  /**
   * Registers a callback invoked whenever a fresh IMU sample is decoded.
   *
   * The callback runs outside internal locks. Keep it lightweight.
   */
  SubscriptionHandle subscribe_imu(ImuCallback callback);
  /**
   * Registers a callback invoked whenever a fresh motor-state sample is decoded.
   *
   * The callback runs outside internal locks. Keep it lightweight.
   */
  SubscriptionHandle subscribe_motor_state(MotorStateCallback callback);

  /**
   * Enqueues a `COMM_SET_RPM` command for transmission.
   *
   * Returns `true` only if the command remains queued or otherwise deliverable
   * when the call returns.
   */
  bool set_rpm(std::int32_t rpm);
  /** Enqueues a `COMM_SET_DUTY` command for transmission. */
  bool set_duty(float duty);
  /** Enqueues a `COMM_SET_CURRENT` command for transmission. */
  bool set_current(float amps);
  /** Enqueues a `COMM_SET_CURRENT_BRAKE` command for transmission. */
  bool set_current_brake(float amps);
  /** Enqueues a `COMM_SET_SERVO_POS` command for transmission. */
  bool set_servo_pos(float position);

  /**
   * Performs a blocking firmware-version query.
   *
   * Returns `std::nullopt` on timeout or disconnect.
   */
  std::optional<FwVersion> request_fw_version(std::chrono::milliseconds timeout);

private:
  using Payload = VescPacketParser::Payload;
  using SteadyClock = std::chrono::steady_clock;

  struct QueryState {
    std::mutex mutex;
    std::condition_variable cv;
    bool completed{false};
    std::optional<FwVersion> result;
  };

  struct PollChannel {
    enum class Kind { Imu, MotorState };

    Kind kind;
    std::atomic<std::int64_t> interval_ms{0};
    SteadyClock::time_point next_due{};
    SteadyClock::time_point last_sample{};
  };

  struct ScheduledRequest {
    enum class Kind { PollImu, PollMotorState, FwVersion };

    Kind kind;
    std::uint8_t expected_id{0};
    std::vector<std::uint8_t> packet;
    SteadyClock::time_point deadline{};
    std::function<void(const Payload&)> on_success;
    std::function<void()> on_timeout;
  };

  VescConfig config_;
  int fd_{-1};
  int wake_pipe_[2]{-1, -1};

  VescProtocol io_protocol_;
  VescProtocol cmd_protocol_;
  VescPacketParser parser_;

  std::thread io_thread_;
  std::atomic<bool> running_{false};

  std::mutex protocol_mutex_;
  std::mutex scheduler_mutex_;
  std::deque<std::vector<std::uint8_t>> command_queue_;
  std::deque<ScheduledRequest> request_queue_;
  std::optional<ScheduledRequest> in_flight_request_;
  std::unordered_map<std::uint8_t, std::size_t> stale_query_reply_counts_;

  mutable std::mutex cache_mutex_;
  std::optional<VescMotorState> motor_state_cache_;
  std::optional<VescIMUData> imu_data_cache_;

  std::mutex callback_mutex_;
  std::size_t next_subscription_id_{1};
  std::unordered_map<std::size_t, ImuCallback> imu_callbacks_;
  std::unordered_map<std::size_t, MotorStateCallback> motor_callbacks_;

  PollChannel imu_channel_{PollChannel::Kind::Imu};
  PollChannel motor_channel_{PollChannel::Kind::MotorState};

  static std::uint64_t default_wall_time_ns();
  std::uint64_t wall_time_ns() const;

  void io_loop();
  void dispatch_payload(const Payload& payload, std::uint64_t stamp_ns);
  void handle_request_timeout();
  void schedule_query(ScheduledRequest request);

  bool enqueue_command(std::vector<std::uint8_t> packet);
  void wake_io_thread() const;
  bool write_packet(const std::vector<std::uint8_t>& pkt);
  bool wait_until_writable();
  void publish_imu(const VescIMUData& data);
  void publish_motor_state(const VescMotorState& state);
  void clear_pending_work();
  void unsubscribe_imu(std::size_t id);
  void unsubscribe_motor_state(std::size_t id);

  PollChannel* select_due_poll_channel(const SteadyClock::time_point& now);
  std::optional<ScheduledRequest> make_due_poll_request(PollChannel& channel,
                                                        const SteadyClock::time_point& now);
  std::optional<ScheduledRequest> dequeue_ready_query(const SteadyClock::time_point& now);

#ifdef GOAT_VESC_TESTING
  friend struct VescClientTestAccess;
#endif
};

} // namespace goat_vesc
