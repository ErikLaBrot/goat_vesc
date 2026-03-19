/**
 * @file vesc_client.hpp
 * @brief Thread-safe high-level client for VESC serial communication.
 */

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
 * @brief Thread-safe VESC client with a single transport-owner thread.
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
 * - This class detects transport loss and stops queueing/sending commands.
 * - `VescConfig::command_watchdog_*` can also enable an opt-in stale-command
 *   watchdog that sends one safe-stop command when control input stops
 *   refreshing.
 * - The watchdog is disabled by default, so callers that need dropout safety
 *   should still configure VESC-side timeouts as a backstop.
 */
class VescClient {
public:
  /** @brief Callback invoked when a fresh IMU sample is decoded. */
  using ImuCallback = std::function<void(const VescIMUData&)>;
  /** @brief Callback invoked when a fresh motor-state sample is decoded. */
  using MotorStateCallback = std::function<void(const VescMotorState&)>;

  /**
   * @brief RAII token for a subscription created by subscribe_imu() or
   * subscribe_motor_state(). Destroying or resetting the handle unregisters the
   * callback.
   */
  class SubscriptionHandle {
  public:
    /** @brief Creates an empty handle that is not bound to a subscription. */
    SubscriptionHandle() = default;
    SubscriptionHandle(const SubscriptionHandle&) = delete;
    SubscriptionHandle& operator=(const SubscriptionHandle&) = delete;

    /**
     * @brief Transfers ownership of a subscription registration.
     * @param other Handle to move from.
     */
    SubscriptionHandle(SubscriptionHandle&& other) noexcept;
    /**
     * @brief Transfers ownership of a subscription registration.
     * @param other Handle to move from.
     * @return `*this`.
     */
    SubscriptionHandle& operator=(SubscriptionHandle&& other) noexcept;

    /** @brief Unregisters the subscription if the handle still owns one. */
    ~SubscriptionHandle();

    /** @brief Unregisters the subscription and makes the handle empty. */
    void reset();
    /** @brief Returns `true` when the handle still owns a live subscription. */
    explicit operator bool() const {
      return unsubscribe_ != nullptr;
    }

  private:
    friend class VescClient;
    explicit SubscriptionHandle(std::function<void()> unsubscribe);

    std::function<void()> unsubscribe_;
  };

  /**
   * @brief Creates a client with the given transport, timing, and timestamp settings.
   *
   * The client is inactive until connect() is called.
   *
   * @param config Runtime configuration copied into the client.
   */
  explicit VescClient(VescConfig config);
  /** @brief Disconnects the transport if still connected. */
  ~VescClient();

  VescClient(const VescClient&) = delete;
  VescClient& operator=(const VescClient&) = delete;

  /**
   * @brief Returns currently visible `/dev/ttyACM*` candidates.
   * @return Candidate device paths discovered through globbing.
   */
  static std::vector<std::string> find_devices();

  /**
   * @brief Opens the configured transport and starts the background I/O thread.
   *
   * If `config.device_path` is empty and no custom `open_serial_fn` is supplied,
   * the first `/dev/ttyACM*` device is used.
   *
   * @return `true` when the transport thread is running after the call.
   */
  bool connect();

  /**
   * @brief Stops the background thread and closes the transport.
   *
   * Outstanding blocking queries are completed with `std::nullopt`.
   */
  void disconnect();

  /**
   * @brief True while the client believes the transport thread is active.
   * @return `true` when the transport thread is currently running.
   */
  bool is_connected() const;

  /**
   * @brief Updates the periodic motor-state polling cadence.
   * @param interval New cadence. `0 ms` disables motor polling.
   */
  void set_motor_poll_interval(std::chrono::milliseconds interval);
  /**
   * @brief Updates the periodic IMU polling cadence.
   * @param interval New cadence. `0 ms` disables IMU polling.
   */
  void set_imu_poll_interval(std::chrono::milliseconds interval);
  /**
   * @brief Returns the current bridge-facing polling and watchdog configuration.
   * @return Snapshot including runtime-updated poll intervals.
   */
  VescClientConfigSnapshot config_snapshot() const;

  /**
   * @brief Returns the latest cached motor-state sample, if any.
   * @return Most recent motor-state sample seen by the client.
   */
  std::optional<VescMotorState> latest_motor_state() const;
  /**
   * @brief Returns the latest cached IMU sample, if any.
   * @return Most recent IMU sample seen by the client.
   */
  std::optional<VescIMUData> latest_imu() const;

  /**
   * @brief Registers a callback invoked whenever a fresh IMU sample is decoded.
   *
   * The callback runs outside internal locks. Keep it lightweight.
   *
   * @param callback Function to invoke when a new IMU sample arrives.
   * @return RAII handle that unregisters the callback on destruction.
   */
  SubscriptionHandle subscribe_imu(ImuCallback callback);
  /**
   * @brief Registers a callback invoked whenever a fresh motor-state sample is decoded.
   *
   * The callback runs outside internal locks. Keep it lightweight.
   *
   * @param callback Function to invoke when a new motor-state sample arrives.
   * @return RAII handle that unregisters the callback on destruction.
   */
  SubscriptionHandle subscribe_motor_state(MotorStateCallback callback);

  /**
   * @brief Enqueues a `COMM_SET_RPM` command for transmission.
   *
   * Returns `true` only if the command remains queued or otherwise deliverable
   * when the call returns.
   *
   * @param rpm Target RPM command.
   * @return `true` when the command remains deliverable after submission.
   */
  bool set_rpm(std::int32_t rpm);
  /**
   * @brief Enqueues a `COMM_SET_DUTY` command for transmission.
   * @param duty Duty-cycle request, typically in the `[-1.0, 1.0]` range.
   * @return `true` when the command remains deliverable after submission.
   */
  bool set_duty(float duty);
  /**
   * @brief Enqueues a `COMM_SET_CURRENT` command for transmission.
   * @param amps Target current command in amps.
   * @return `true` when the command remains deliverable after submission.
   */
  bool set_current(float amps);
  /**
   * @brief Enqueues a bounded `COMM_SET_CURRENT_BRAKE` command for transmission.
   *
   * `amps` is a positive brake-current magnitude. Returns `false` if `amps` is
   * non-positive, active braking is disabled by configuration, or the command
   * cannot remain deliverable.
   *
   * @param amps Requested brake-current magnitude in amps.
   * @return `true` when the command remains deliverable after submission.
   */
  bool set_current_brake(float amps);
  /**
   * @brief Enqueues a `COMM_SET_SERVO_POS` command for transmission.
   * @param position Servo position in controller-specific normalized units.
   * @return `true` when the command remains deliverable after submission.
   */
  bool set_servo_pos(float position);

  /**
   * @brief Performs a blocking firmware-version query.
   *
   * Returns `std::nullopt` on timeout or disconnect.
   *
   * @param timeout Maximum time to wait for a firmware reply.
   * @return Parsed firmware version on success, otherwise `std::nullopt`.
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

  struct CallbackRegistry {
    std::mutex mutex;
    std::size_t next_subscription_id{1};
    std::unordered_map<std::size_t, ImuCallback> imu_callbacks;
    std::unordered_map<std::size_t, MotorStateCallback> motor_callbacks;
  };

  struct ControlWatchdogState {
    SteadyClock::time_point deadline{};
    bool armed{false};
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
  ControlWatchdogState control_watchdog_;

  mutable std::mutex cache_mutex_;
  std::optional<VescMotorState> motor_state_cache_;
  std::optional<VescIMUData> imu_data_cache_;

  std::shared_ptr<CallbackRegistry> callback_registry_;

  PollChannel imu_channel_{PollChannel::Kind::Imu};
  PollChannel motor_channel_{PollChannel::Kind::MotorState};

  static std::uint64_t default_wall_time_ns();
  std::uint64_t wall_time_ns() const;

  void io_loop();
  void dispatch_payload(const Payload& payload, std::uint64_t stamp_ns);
  void handle_request_timeout();
  void schedule_query(ScheduledRequest request);

  bool enqueue_control_command(std::vector<std::uint8_t> packet);
  bool control_watchdog_enabled() const;
  std::optional<std::vector<std::uint8_t>>
  dequeue_due_watchdog_command(const SteadyClock::time_point& now);
  void wake_io_thread() const;
  bool write_packet(const std::vector<std::uint8_t>& pkt);
  bool wait_until_writable();
  void publish_imu(const VescIMUData& data);
  void publish_motor_state(const VescMotorState& state);
  void clear_pending_work();
  void cleanup_transport_state();

  PollChannel* select_due_poll_channel(const SteadyClock::time_point& now);
  std::optional<ScheduledRequest> make_due_poll_request(PollChannel& channel,
                                                        const SteadyClock::time_point& now);
  std::optional<ScheduledRequest> dequeue_ready_query(const SteadyClock::time_point& now);

#ifdef GOAT_VESC_TESTING
  friend struct VescClientTestAccess;
#endif
};

} // namespace goat_vesc
