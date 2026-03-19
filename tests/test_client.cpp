#include "test_support.hpp"

#include "goat_vesc/packet_parser.hpp"
#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/vesc_client.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace goat_vesc {

struct VescClientTestAccess {
  static auto scheduler_mutex(VescClient* client) -> std::mutex& {
    return client->scheduler_mutex_;
  }

  static auto running(VescClient* client) -> std::atomic<bool>& {
    return client->running_;
  }

  static auto command_queue(VescClient* client) -> std::deque<std::vector<std::uint8_t>>& {
    return client->command_queue_;
  }

  static auto stale_reply_count(VescClient& client, std::uint8_t id) -> std::size_t {
    std::lock_guard lock(client.scheduler_mutex_);
    const auto it = client.stale_query_reply_counts_.find(id);
    return it == client.stale_query_reply_counts_.end() ? 0U : it->second;
  }

  static auto imu_subscription_count(const VescClient& client) -> std::size_t {
    const auto callbacks = client.callback_registry_;
    std::lock_guard lock(callbacks->mutex);
    return callbacks->imu_callbacks.size();
  }

  static auto motor_subscription_count(const VescClient& client) -> std::size_t {
    const auto callbacks = client.callback_registry_;
    std::lock_guard lock(callbacks->mutex);
    return callbacks->motor_callbacks.size();
  }

  static auto io_thread_joinable(VescClient& client) -> bool {
    return client.io_thread_.joinable();
  }

  static auto fd(const VescClient& client) -> int {
    return client.fd_;
  }

  static auto wake_read_fd(const VescClient& client) -> int {
    return client.wake_pipe_[0];
  }

  static auto wake_write_fd(const VescClient& client) -> int {
    return client.wake_pipe_[1];
  }
};

} // namespace goat_vesc

namespace {

using namespace goat_vesc;
using namespace std::chrono_literals;
using namespace test_support;

std::vector<std::uint8_t> make_fw_response() {
  return frame_payload({
      static_cast<std::uint8_t>(VescPacketCommID::FwVersion),
      6,
      5,
  });
}

std::vector<std::uint8_t> make_imu_response(float seed) {
  std::vector<std::uint8_t> payload;
  append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetImuData));
  append_u16(payload, DefaultVescImuMask);
  for (int i = 0; i < 16; ++i) {
    append_f32(payload, seed + static_cast<float>(i) * 0.25f);
  }
  return frame_payload(payload);
}

std::vector<std::uint8_t> make_values_response(std::int32_t rpm) {
  std::vector<std::uint8_t> payload;
  append_u8(payload, static_cast<std::uint8_t>(VescPacketCommID::GetValues));
  append_i16(payload, 325);
  append_i16(payload, 301);
  append_i32(payload, 1234);
  append_i32(payload, 890);
  append_i32(payload, 0);
  append_i32(payload, 0);
  append_i16(payload, 455);
  append_i32(payload, rpm);
  append_i16(payload, 523);
  append_i32(payload, 0);
  append_i32(payload, 0);
  append_i32(payload, 0);
  append_i32(payload, 0);
  append_i32(payload, 77);
  append_i32(payload, 88);
  append_u8(payload, 0);
  return frame_payload(payload);
}

std::int16_t read_i16(const std::vector<std::uint8_t>& payload, std::size_t offset) {
  assert(offset + 1 < payload.size());
  const auto hi = static_cast<std::uint16_t>(payload[offset]);
  const auto lo = static_cast<std::uint16_t>(payload[offset + 1]);
  return static_cast<std::int16_t>((hi << 8) | lo);
}

std::int32_t read_i32(const std::vector<std::uint8_t>& payload, std::size_t offset) {
  assert(offset + 3 < payload.size());
  const auto b0 = static_cast<std::uint32_t>(payload[offset]);
  const auto b1 = static_cast<std::uint32_t>(payload[offset + 1]);
  const auto b2 = static_cast<std::uint32_t>(payload[offset + 2]);
  const auto b3 = static_cast<std::uint32_t>(payload[offset + 3]);
  return static_cast<std::int32_t>((b0 << 24) | (b1 << 16) | (b2 << 8) | b3);
}

void write_all(int fd, const std::vector<std::uint8_t>& bytes) {
  const std::uint8_t* cursor = bytes.data();
  std::size_t remaining = bytes.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd, cursor, remaining);
    if (n < 0) {
      throw std::runtime_error("write failed");
    }
    cursor += n;
    remaining -= static_cast<std::size_t>(n);
  }
}

bool is_writable(int fd) {
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(fd, &wfds);

  struct timeval timeout {};
  const int rc = ::select(fd + 1, nullptr, &wfds, nullptr, &timeout);
  return rc > 0 && FD_ISSET(fd, &wfds);
}

bool try_write_all(int fd, const std::vector<std::uint8_t>& bytes) {
  const std::uint8_t* cursor = bytes.data();
  std::size_t remaining = bytes.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd, cursor, remaining);
    if (n < 0) {
      return false;
    }
    cursor += n;
    remaining -= static_cast<std::size_t>(n);
  }
  return true;
}

template <typename Predicate>
void wait_until(Predicate predicate, std::chrono::milliseconds timeout, const char* message) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(message);
    }
    std::this_thread::sleep_for(5ms);
  }
}

struct QuiescenceWait {
  std::chrono::milliseconds timeout;
  std::chrono::milliseconds stable_for;
};

template <typename Loader>
auto wait_for_quiescence(Loader load, QuiescenceWait wait, const char* message) -> int {
  const auto deadline = std::chrono::steady_clock::now() + wait.timeout;
  int last_value = load();
  auto stable_since = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    const int current = load();
    if (current != last_value) {
      last_value = current;
      stable_since = std::chrono::steady_clock::now();
    } else if (std::chrono::steady_clock::now() - stable_since >= wait.stable_for) {
      return current;
    }
    std::this_thread::sleep_for(5ms);
  }

  throw std::runtime_error(message);
}

struct ScopedFd {
  explicit ScopedFd(int fd = -1) : fd_(fd) {}

  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;

  ScopedFd(ScopedFd&& other) noexcept : fd_(other.release()) {}

  auto operator=(ScopedFd&& other) noexcept -> ScopedFd& {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  ~ScopedFd() {
    reset();
  }

  [[nodiscard]] auto get() const -> int {
    return fd_;
  }

  [[nodiscard]] auto release() -> int {
    const int released = fd_;
    fd_ = -1;
    return released;
  }

  void reset(int fd = -1) {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }

private:
  int fd_;
};

struct WriteFailingFakeTransport {
  WriteFailingFakeTransport() {
    int pipe_fds[2]{-1, -1};
    if (::pipe(pipe_fds) != 0) {
      throw std::runtime_error("pipe failed");
    }

    client_read_fd_.reset(pipe_fds[0]);
    write_fd_.reset(pipe_fds[1]);
  }

  bool open_client_fd(int& fd_out) {
    if (client_read_fd_.get() < 0) {
      return false;
    }

    fd_out = client_read_fd_.release();
    return true;
  }

private:
  ScopedFd client_read_fd_;
  ScopedFd write_fd_;
};

struct BlockedWriteFakeVesc {
  BlockedWriteFakeVesc() {
    int fds[2]{-1, -1};
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
      throw std::runtime_error("socketpair failed");
    }

    ScopedFd server_fd(fds[0]);
    ScopedFd client_fd(fds[1]);

    const int send_buffer_bytes = 1024;
    if (::setsockopt(client_fd.get(), SOL_SOCKET, SO_SNDBUF, &send_buffer_bytes,
                     sizeof(send_buffer_bytes)) != 0) {
      throw std::runtime_error("setsockopt(SO_SNDBUF) failed");
    }

    socklen_t actual_send_buffer_size_len = sizeof(actual_send_buffer_bytes_);
    if (::getsockopt(client_fd.get(), SOL_SOCKET, SO_SNDBUF, &actual_send_buffer_bytes_,
                     &actual_send_buffer_size_len) != 0) {
      throw std::runtime_error("getsockopt(SO_SNDBUF) failed");
    }

    ScopedFd observer_fd(::dup(client_fd.get()));
    if (observer_fd.get() < 0) {
      throw std::runtime_error("dup failed");
    }

    server_fd_ = server_fd.release();
    client_fd_ = client_fd.release();
    observer_fd_ = observer_fd.release();
  }

  BlockedWriteFakeVesc(const BlockedWriteFakeVesc&) = delete;
  BlockedWriteFakeVesc& operator=(const BlockedWriteFakeVesc&) = delete;
  BlockedWriteFakeVesc(BlockedWriteFakeVesc&&) = delete;
  BlockedWriteFakeVesc& operator=(BlockedWriteFakeVesc&&) = delete;

  ~BlockedWriteFakeVesc() {
    if (observer_fd_ >= 0) {
      ::close(observer_fd_);
    }
    if (server_fd_ >= 0) {
      ::close(server_fd_);
    }
    if (client_fd_ >= 0) {
      ::close(client_fd_);
    }
  }

  bool open_client_fd(int& fd_out) {
    if (client_fd_ < 0) {
      return false;
    }

    fd_out = client_fd_;
    client_fd_ = -1;
    return true;
  }

  void wait_until_blocked(std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!is_writable(observer_fd_)) {
        return;
      }
      std::this_thread::sleep_for(5ms);
    }

    throw std::runtime_error("client transport never reached blocked-write state; actual "
                             "SO_SNDBUF=" +
                             std::to_string(actual_send_buffer_bytes_));
  }

private:
  int actual_send_buffer_bytes_{0};
  int server_fd_{-1};
  int client_fd_{-1};
  int observer_fd_{-1};
};

struct FakeVesc {
  struct Behavior {
    int dropped_imu_replies{0};
    int dropped_value_replies{0};
    bool respond_to_fw{true};
    std::vector<std::chrono::milliseconds> fw_response_delays;

    static auto drop_imu_replies(int count) -> Behavior {
      return Behavior{count, 0, true};
    }

    static auto without_fw_response() -> Behavior {
      return Behavior{0, 0, false};
    }

    static auto delayed_fw_responses(std::vector<std::chrono::milliseconds> delays) -> Behavior {
      Behavior behavior;
      behavior.fw_response_delays = std::move(delays);
      return behavior;
    }
  };

  FakeVesc() : FakeVesc(Behavior{}) {}

  explicit FakeVesc(Behavior behavior)
      : dropped_imu_replies_(std::max(behavior.dropped_imu_replies, 0)),
        dropped_value_replies_(std::max(behavior.dropped_value_replies, 0)),
        respond_fw_(behavior.respond_to_fw),
        fw_response_delays_(std::move(behavior.fw_response_delays)) {
    int fds[2]{-1, -1};
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
      throw std::runtime_error("socketpair failed");
    }
    server_fd_ = fds[0];
    client_fd_ = fds[1];
    const int flags = ::fcntl(server_fd_, F_GETFL, 0);
    (void)::fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
    worker_ = std::thread([this] { run(); });
  }

  ~FakeVesc() {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    std::vector<std::thread> response_threads;
    {
      std::lock_guard lock(response_threads_mutex_);
      response_threads.swap(response_threads_);
    }
    for (auto& response_thread : response_threads) {
      if (response_thread.joinable()) {
        response_thread.join();
      }
    }
    if (server_fd_ >= 0) {
      ::close(server_fd_);
    }
    if (client_fd_ >= 0) {
      ::close(client_fd_);
    }
  }

  bool open_client_fd(int& fd_out) {
    if (client_fd_ < 0) {
      return false;
    }
    fd_out = client_fd_;
    client_fd_ = -1;
    return true;
  }

  std::atomic<int> imu_requests{0};
  std::atomic<int> value_requests{0};
  std::atomic<int> fw_requests{0};
  std::atomic<int> rpm_commands{0};
  std::atomic<int> current_commands{0};
  std::atomic<int> duty_commands{0};
  std::atomic<int> brake_current_commands{0};
  std::atomic<int> servo_commands{0};
  std::atomic<std::int32_t> last_current_raw{0};
  std::atomic<std::int32_t> last_duty_raw{0};
  std::atomic<std::int32_t> last_brake_current_raw{0};
  std::atomic<std::int16_t> last_servo_raw{0};

private:
  void run() {
    VescPacketParser parser;
    float imu_seed = 1.0f;
    std::int32_t rpm_value = 1400;

    while (running_.load()) {
      std::uint8_t buffer[512];
      const ssize_t n = ::read(server_fd_, buffer, sizeof(buffer));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          if (auto payload = parser.feed_byte(buffer[static_cast<std::size_t>(i)])) {
            const auto id = payload->front();
            if (id == static_cast<std::uint8_t>(VescPacketCommID::GetImuData)) {
              const int count = ++imu_requests;
              if (count <= dropped_imu_replies_) {
                continue;
              }
              write_all(server_fd_, make_imu_response(imu_seed));
              imu_seed += 1.0f;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::GetValues)) {
              const int count = ++value_requests;
              if (count <= dropped_value_replies_) {
                continue;
              }
              write_all(server_fd_, make_values_response(rpm_value));
              rpm_value += 25;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::FwVersion)) {
              const int request_index = fw_requests.fetch_add(1);
              if (!respond_fw_) {
                continue;
              }
              const auto request_slot = static_cast<std::size_t>(request_index);
              const auto delay = request_slot < fw_response_delays_.size()
                                     ? fw_response_delays_[request_slot]
                                     : std::chrono::milliseconds::zero();
              if (delay <= std::chrono::milliseconds::zero()) {
                write_all(server_fd_, make_fw_response());
              } else {
                schedule_response(delay, make_fw_response());
              }
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetRpm) &&
                       payload->size() == 5) {
              ++rpm_commands;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetCurrent) &&
                       payload->size() == 5) {
              last_current_raw.store(read_i32(*payload, 1));
              ++current_commands;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetDuty) &&
                       payload->size() == 5) {
              last_duty_raw.store(read_i32(*payload, 1));
              ++duty_commands;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetCurrentBrake) &&
                       payload->size() == 5) {
              last_brake_current_raw.store(read_i32(*payload, 1));
              ++brake_current_commands;
            } else if (id == static_cast<std::uint8_t>(VescPacketCommID::SetServoPos) &&
                       payload->size() == 3) {
              last_servo_raw.store(read_i16(*payload, 1));
              ++servo_commands;
            }
          }
        }
        continue;
      }

      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        std::this_thread::sleep_for(2ms);
        continue;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  int server_fd_{-1};
  int client_fd_{-1};
  std::atomic<bool> running_{true};
  int dropped_imu_replies_{0};
  int dropped_value_replies_{0};
  bool respond_fw_{true};
  std::vector<std::chrono::milliseconds> fw_response_delays_;
  std::mutex response_threads_mutex_;
  std::vector<std::thread> response_threads_;
  std::thread worker_;

  void schedule_response(std::chrono::milliseconds delay, std::vector<std::uint8_t> packet) {
    std::lock_guard lock(response_threads_mutex_);
    response_threads_.emplace_back([this, delay, packet = std::move(packet)]() mutable {
      std::this_thread::sleep_for(delay);
      if (!running_.load()) {
        return;
      }
      (void)try_write_all(server_fd_, packet);
    });
  }
};

void test_client_polling_and_subscriptions() {
  FakeVesc fake;
  std::atomic<std::uint64_t> stamp_counter{1000};

  VescConfig config;
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 40ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(1000); };
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  std::atomic<int> imu_callbacks{0};
  std::atomic<int> current_from_callback{0};

  auto imu_handle = client.subscribe_imu([&](const VescIMUData& data) {
    ++imu_callbacks;
    assert(data.stamp_ns >= 1000);
    (void)client.latest_imu();
    if (current_from_callback.fetch_add(1) == 0) {
      assert(client.set_current(1.5f));
    }
  });

  auto motor_handle = client.subscribe_motor_state([&](const VescMotorState& state) {
    assert(state.stamp_ns >= 1000);
    assert(state.vin > 0.0f);
  });

  assert(client.connect());

  wait_until([&] { return fake.imu_requests.load() >= 1; }, 500ms,
             "imu poll requests were not sent");
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (imu_callbacks.load() < 2) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error(
            "imu callbacks did not arrive; requests=" + std::to_string(fake.imu_requests.load()) +
            " latest_imu=" + std::to_string(client.latest_imu().has_value()) +
            " value_requests=" + std::to_string(fake.value_requests.load()) +
            " latest_motor=" + std::to_string(client.latest_motor_state().has_value()));
      }
      std::this_thread::sleep_for(5ms);
    }
  }
  wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms,
             "motor state did not arrive");

  const auto imu = client.latest_imu();
  const auto motor = client.latest_motor_state();
  assert(imu.has_value());
  assert(motor.has_value());
  assert(imu->stamp_ns > 0);
  assert(motor->stamp_ns > 0);
  assert(fake.current_commands.load() >= 1);

  const auto before = imu_callbacks.load();
  const auto version = client.request_fw_version(200ms);
  assert(version.has_value());
  assert(version->major == 6);
  assert(version->minor == 5);
  wait_until([&] { return imu_callbacks.load() > before; }, 500ms,
             "imu polling stalled after fw query");

  std::vector<std::thread> writers;
  writers.reserve(4);
  for (int i = 0; i < 4; ++i) {
    writers.emplace_back([&client, i] {
      for (int j = 0; j < 10; ++j) {
        assert(client.set_rpm(1000 + i * 100 + j));
      }
    });
  }
  for (auto& writer : writers) {
    writer.join();
  }

  wait_until([&] { return fake.rpm_commands.load() >= 40; }, 500ms, "rpm commands were dropped");
  client.disconnect();

  (void)imu_handle;
  (void)motor_handle;
}

void test_subscription_cleanup_after_disconnect() {
  FakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 30ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  std::atomic<int> imu_callbacks{0};
  std::atomic<int> motor_callbacks{0};

  auto imu_handle = client.subscribe_imu([&](const VescIMUData&) { ++imu_callbacks; });
  auto motor_handle =
      client.subscribe_motor_state([&](const VescMotorState&) { ++motor_callbacks; });

  assert(VescClientTestAccess::imu_subscription_count(client) == 1);
  assert(VescClientTestAccess::motor_subscription_count(client) == 1);

  assert(client.connect());
  wait_until([&] { return imu_callbacks.load() >= 1; }, 500ms, "imu callback never fired");
  wait_until([&] { return motor_callbacks.load() >= 1; }, 500ms, "motor callback never fired");

  client.disconnect();

  imu_handle.reset();
  motor_handle.reset();

  assert(!imu_handle);
  assert(!motor_handle);
  assert(VescClientTestAccess::imu_subscription_count(client) == 0);
  assert(VescClientTestAccess::motor_subscription_count(client) == 0);
  assert(!client.is_connected());
}

void test_subscription_handle_can_outlive_client_destruction() {
  std::optional<VescClient::SubscriptionHandle> imu_handle;
  std::optional<VescClient::SubscriptionHandle> motor_handle;

  {
    VescClient client(VescConfig{});
    imu_handle.emplace(client.subscribe_imu([](const VescIMUData&) {}));
    motor_handle.emplace(client.subscribe_motor_state([](const VescMotorState&) {}));

    assert(VescClientTestAccess::imu_subscription_count(client) == 1);
    assert(VescClientTestAccess::motor_subscription_count(client) == 1);
  }

  assert(imu_handle.has_value());
  assert(motor_handle.has_value());

  imu_handle->reset();
  motor_handle->reset();

  assert(!*imu_handle);
  assert(!*motor_handle);
}

void test_poll_timeout_recovers() {
  FakeVesc fake(FakeVesc::Behavior::drop_imu_replies(1));
  VescConfig config;
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 30ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms,
             "imu poll did not retry after timeout");
  wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never recovered");
  wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms,
             "motor polling stalled");
  client.disconnect();
}

void test_imu_timeout_does_not_starve_motor_polling() {
  auto behavior = FakeVesc::Behavior::drop_imu_replies(2);
  FakeVesc fake(behavior);
  VescConfig config;
  config.imu_poll_interval = 10ms;
  config.motor_poll_interval = 15ms;
  config.poll_response_timeout = 30ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms,
             "imu poll did not hit repeated timeout path");
  wait_until([&] { return client.latest_motor_state().has_value(); }, 70ms,
             "motor polling was starved by imu timeout recovery");
  wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never recovered");

  client.disconnect();
}

void test_control_command_delivery() {
  FakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.max_brake_current = 1.0f;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  assert(client.set_duty(0.2f));
  assert(client.set_current_brake(1.5f));
  assert(client.set_servo_pos(0.5f));

  wait_until([&] { return fake.duty_commands.load() == 1; }, 500ms,
             "duty command was not delivered");
  wait_until([&] { return fake.brake_current_commands.load() == 1; }, 500ms,
             "brake current command was not delivered");
  wait_until([&] { return fake.servo_commands.load() == 1; }, 500ms,
             "servo command was not delivered");

  assert(fake.last_duty_raw.load() == 20000);
  assert(fake.last_brake_current_raw.load() == 1000);
  assert(fake.last_servo_raw.load() == 500);

  client.disconnect();
}

void test_brake_current_command_requires_positive_limit() {
  FakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  assert(!client.set_current_brake(1.0f));
  assert(!client.set_current_brake(0.0f));
  assert(!client.set_current_brake(-1.0f));
  std::this_thread::sleep_for(50ms);
  assert(fake.brake_current_commands.load() == 0);

  client.disconnect();
}

void test_watchdog_brake_current_safe_stop() {
  FakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.command_watchdog_timeout = 40ms;
  config.command_watchdog_action = ControlWatchdogAction::BrakeCurrent;
  config.max_brake_current = 1.5f;
  config.command_watchdog_brake_current = 2.5f;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  assert(client.set_rpm(1800));
  wait_until([&] { return fake.rpm_commands.load() == 1; }, 500ms,
             "rpm command was not delivered before watchdog arming");

  std::this_thread::sleep_for(20ms);
  assert(fake.brake_current_commands.load() == 0);

  wait_until([&] { return fake.brake_current_commands.load() == 1; }, 500ms,
             "watchdog brake command did not fire");
  assert(fake.last_brake_current_raw.load() == 1500);

  std::this_thread::sleep_for(80ms);
  assert(fake.brake_current_commands.load() == 1);

  assert(client.set_duty(0.1f));
  wait_until([&] { return fake.duty_commands.load() == 1; }, 500ms,
             "duty command did not re-arm the watchdog");
  wait_until([&] { return fake.brake_current_commands.load() == 2; }, 500ms,
             "watchdog did not re-arm after fresh control input");

  client.disconnect();
}

void test_watchdog_coast_safe_stop() {
  FakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.command_watchdog_timeout = 30ms;
  config.command_watchdog_action = ControlWatchdogAction::Coast;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  assert(client.set_duty(0.35f));
  wait_until([&] { return fake.duty_commands.load() == 1; }, 500ms,
             "duty command was not delivered before watchdog arming");
  wait_until([&] { return fake.current_commands.load() == 1; }, 500ms,
             "coast watchdog did not send zero-current command");
  assert(fake.last_current_raw.load() == 0);

  client.disconnect();
}

auto test_connect_disconnect_edges() -> void {
  constexpr auto kNoPolling = 0ms;

  FakeVesc fake;
  std::atomic<int> open_calls{0};

  VescConfig config;
  config.imu_poll_interval = kNoPolling;
  config.motor_poll_interval = kNoPolling;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake, &open_calls](const VescConfig&, int& fd_out) {
    ++open_calls;
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);

  client.disconnect();
  assert(!client.is_connected());

  const bool connected = client.connect();
  assert(connected);
  assert(client.is_connected());
  assert(open_calls.load() == 1);

  const bool reconnect_result = client.connect();
  assert(reconnect_result);
  assert(client.is_connected());
  assert(open_calls.load() == 1);

  const bool rpm_sent = client.set_rpm(1234);
  assert(rpm_sent);
  wait_until([&] { return fake.rpm_commands.load() == 1; }, 500ms, "rpm command was not delivered");

  client.disconnect();
  assert(!client.is_connected());

  client.disconnect();
  assert(!client.is_connected());
  const bool rpm_after_disconnect = client.set_rpm(2345);
  assert(!rpm_after_disconnect);
  const auto version_after_disconnect = client.request_fw_version(20ms);
  assert(!version_after_disconnect.has_value());
}

auto test_runtime_poll_interval_updates() -> void {
  constexpr auto kInitialStampNs = 1000ULL;
  constexpr auto kStampStepNs = 1000ULL;
  constexpr auto kNoPolling = 0ms;

  FakeVesc fake;
  std::atomic<std::uint64_t> stamp_counter{kInitialStampNs};

  VescConfig config;
  config.imu_poll_interval = kNoPolling;
  config.motor_poll_interval = kNoPolling;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(kStampStepNs); };
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  const bool connected = client.connect();
  assert(connected);

  std::this_thread::sleep_for(60ms);
  assert(fake.imu_requests.load() == 0);
  assert(fake.value_requests.load() == 0);
  assert(!client.latest_imu().has_value());
  assert(!client.latest_motor_state().has_value());

  client.set_imu_poll_interval(20ms);
  client.set_motor_poll_interval(30ms);

  wait_until([&] { return fake.imu_requests.load() >= 1; }, 500ms, "imu polling did not start");
  wait_until([&] { return fake.value_requests.load() >= 1; }, 500ms, "motor polling did not start");
  wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache did not update");
  wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms,
             "motor cache did not update");

  const auto first_imu = client.latest_imu();
  const auto first_motor = client.latest_motor_state();
  assert(first_imu.has_value());
  assert(first_motor.has_value());

  client.set_imu_poll_interval(0ms);
  client.set_motor_poll_interval(0ms);

  const QuiescenceWait wait{300ms, 50ms};
  const int quiet_imu_requests = wait_for_quiescence([&] { return fake.imu_requests.load(); }, wait,
                                                     "imu polling never quiesced");
  const int quiet_motor_requests = wait_for_quiescence([&] { return fake.value_requests.load(); },
                                                       wait, "motor polling never quiesced");

  client.set_imu_poll_interval(15ms);
  client.set_motor_poll_interval(15ms);

  wait_until([&] { return fake.imu_requests.load() > quiet_imu_requests; }, 500ms,
             "imu polling did not restart");
  wait_until([&] { return fake.value_requests.load() > quiet_motor_requests; }, 500ms,
             "motor polling did not restart");
  wait_until(
      [&] {
        const auto imu = client.latest_imu();
        return imu.has_value() && imu->stamp_ns > first_imu->stamp_ns;
      },
      500ms, "imu cache did not refresh after poll update");
  wait_until(
      [&] {
        const auto motor = client.latest_motor_state();
        return motor.has_value() && motor->stamp_ns > first_motor->stamp_ns;
      },
      500ms, "motor cache did not refresh after poll update");

  client.disconnect();
}

auto test_config_snapshot_tracks_runtime_polling_behavior() -> void {
  VescConfig config;
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 35ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 7ms;
  config.command_watchdog_timeout = 40ms;
  config.command_watchdog_action = ControlWatchdogAction::BrakeCurrent;
  config.max_brake_current = 3.0f;
  config.command_watchdog_brake_current = 2.5f;

  VescClient client(config);

  const auto initial = client.config_snapshot();
  assert(initial.imu_poll_interval == 20ms);
  assert(initial.motor_poll_interval == 35ms);
  assert(initial.poll_response_timeout == 15ms);
  assert(initial.query_guard_window == 7ms);
  assert(initial.command_watchdog_timeout == 40ms);
  assert(initial.command_watchdog_action == ControlWatchdogAction::BrakeCurrent);
  assert(initial.max_brake_current == 3.0f);
  assert(initial.command_watchdog_brake_current == 2.5f);

  client.set_imu_poll_interval(0ms);
  client.set_motor_poll_interval(55ms);

  const auto updated = client.config_snapshot();
  assert(updated.imu_poll_interval == 0ms);
  assert(updated.motor_poll_interval == 55ms);
  assert(updated.poll_response_timeout == initial.poll_response_timeout);
  assert(updated.query_guard_window == initial.query_guard_window);
  assert(updated.command_watchdog_timeout == initial.command_watchdog_timeout);
  assert(updated.command_watchdog_action == initial.command_watchdog_action);
  assert(updated.max_brake_current == initial.max_brake_current);
  assert(updated.command_watchdog_brake_current == initial.command_watchdog_brake_current);
}

auto test_concurrent_poll_updates_keep_client_responsive() -> void {
  constexpr auto kInitialStampNs = 5000ULL;
  constexpr auto kStampStepNs = 1000ULL;
  constexpr int kPollUpdateIterations = 24;
  constexpr int kReadIterations = 200;

  FakeVesc fake;
  std::atomic<std::uint64_t> stamp_counter{kInitialStampNs};

  VescConfig config;
  config.imu_poll_interval = 25ms;
  config.motor_poll_interval = 35ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 2ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(kStampStepNs); };
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  const bool connected = client.connect();
  assert(connected);

  wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never populated");
  wait_until([&] { return client.latest_motor_state().has_value(); }, 500ms,
             "motor cache never populated");

  std::thread updater([&client] {
    const std::array imu_intervals{0ms, 20ms, 45ms, 15ms};
    const std::array motor_intervals{0ms, 30ms, 50ms, 20ms};
    for (int i = 0; i < kPollUpdateIterations; ++i) {
      client.set_imu_poll_interval(
          imu_intervals[static_cast<std::size_t>(i) % imu_intervals.size()]);
      client.set_motor_poll_interval(
          motor_intervals[static_cast<std::size_t>(i) % motor_intervals.size()]);
      std::this_thread::sleep_for(4ms);
    }
    client.set_imu_poll_interval(20ms);
    client.set_motor_poll_interval(20ms);
  });

  std::thread reader([&client] {
    for (int i = 0; i < kReadIterations; ++i) {
      (void)client.latest_imu();
      (void)client.latest_motor_state();
      std::this_thread::sleep_for(1ms);
    }
  });

  for (int i = 0; i < 3; ++i) {
    const auto version = client.request_fw_version(250ms);
    assert(version.has_value());
    assert(version->major == 6);
    assert(version->minor == 5);
  }

  updater.join();
  reader.join();

  const auto last_imu = client.latest_imu();
  const auto last_motor = client.latest_motor_state();
  assert(last_imu.has_value());
  assert(last_motor.has_value());

  wait_until(
      [&] {
        const auto imu = client.latest_imu();
        return imu.has_value() && imu->stamp_ns > last_imu->stamp_ns;
      },
      500ms, "imu telemetry stalled after concurrent poll updates");
  wait_until(
      [&] {
        const auto motor = client.latest_motor_state();
        return motor.has_value() && motor->stamp_ns > last_motor->stamp_ns;
      },
      500ms, "motor telemetry stalled after concurrent poll updates");

  client.disconnect();
}

void test_queued_query_deadline_expires_while_older_query_waits() {
  FakeVesc fake(FakeVesc::Behavior::without_fw_response());
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  auto first =
      std::async(std::launch::async, [&client] { return client.request_fw_version(200ms); });
  wait_until([&] { return fake.fw_requests.load() == 1; }, 200ms, "first query was not sent");

  auto second =
      std::async(std::launch::async, [&client] { return client.request_fw_version(150ms); });
  std::this_thread::sleep_for(5ms);

  const auto third_started = std::chrono::steady_clock::now();
  auto third =
      std::async(std::launch::async, [&client] { return client.request_fw_version(25ms); });

  const auto third_result = third.get();
  const auto third_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - third_started);
  assert(!third_result.has_value());
  assert(third_elapsed < 45ms);

  const auto second_result = second.get();
  const auto first_result = first.get();
  assert(!second_result.has_value());
  assert(!first_result.has_value());
  assert(fake.fw_requests.load() == 1);

  client.disconnect();
}

void test_late_fw_reply_does_not_satisfy_newer_query() {
  FakeVesc fake(FakeVesc::Behavior::delayed_fw_responses({80ms, 0ms}));
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  const auto first = client.request_fw_version(20ms);
  assert(!first.has_value());
  assert(fake.fw_requests.load() == 1);

  const auto blocked_started = std::chrono::steady_clock::now();
  const auto blocked = client.request_fw_version(40ms);
  const auto blocked_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - blocked_started);
  assert(!blocked.has_value());
  assert(blocked_elapsed < 100ms);
  assert(fake.fw_requests.load() == 1);

  wait_until(
      [&] {
        return VescClientTestAccess::stale_reply_count(
                   client, static_cast<std::uint8_t>(VescPacketCommID::FwVersion)) == 0U;
      },
      200ms, "stale fw reply was not drained");

  const auto recovered = client.request_fw_version(200ms);
  assert(recovered.has_value());
  assert(recovered->major == 6);
  assert(recovered->minor == 5);
  assert(fake.fw_requests.load() == 2);

  client.disconnect();
}

void test_disconnect_unblocks_query() {
  FakeVesc fake(FakeVesc::Behavior::without_fw_response());
  VescConfig config;
  config.imu_poll_interval = 50ms;
  config.motor_poll_interval = 100ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  auto future =
      std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });

  std::this_thread::sleep_for(20ms);
  client.disconnect();
  const auto result = future.get();
  assert(!result.has_value());
}

void test_disconnect_unblocks_blocked_write_wait() {
  BlockedWriteFakeVesc fake;
  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());

  auto query_future =
      std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });

  for (int i = 0; i < 20000; ++i) {
    assert(client.set_rpm(1000 + i));
  }

  fake.wait_until_blocked(500ms);

  auto disconnect_future = std::async(std::launch::async, [&client] {
    client.disconnect();
    return true;
  });

  const auto disconnect_status = disconnect_future.wait_for(250ms);
  assert(disconnect_status == std::future_status::ready);
  assert(disconnect_future.get());
  assert(!client.is_connected());

  const auto query_status = query_future.wait_for(250ms);
  assert(query_status == std::future_status::ready);
  const auto query_result = query_future.get();
  assert(!query_result.has_value());
}

void test_command_rejected_after_disconnect_state_wins_queue_race() {
  VescClient client(VescConfig{});
  auto& scheduler_mutex = VescClientTestAccess::scheduler_mutex(&client);
  auto& running = VescClientTestAccess::running(&client);

  std::unique_lock scheduler_lock(scheduler_mutex);
  running.store(true);

  auto submit = std::async(std::launch::async, [&client] { return client.set_rpm(1234); });

  wait_until([&] { return submit.wait_for(0ms) == std::future_status::timeout; }, 100ms,
             "command submission did not block behind the scheduler lock");

  running.store(false);
  scheduler_lock.unlock();

  assert(!submit.get());
  assert(VescClientTestAccess::command_queue(&client).empty());
}

void test_disconnect_cleans_up_after_async_transport_failure() {
  WriteFailingFakeTransport fake;

  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  VescClient client(config);
  assert(client.connect());
  assert(VescClientTestAccess::io_thread_joinable(client));

  assert(client.set_rpm(1234));
  wait_until([&] { return !client.is_connected(); }, 500ms,
             "async transport failure did not stop the client");

  client.disconnect();
  client.disconnect();

  assert(!client.is_connected());
  assert(!VescClientTestAccess::io_thread_joinable(client));
  assert(VescClientTestAccess::fd(client) == -1);
  assert(VescClientTestAccess::wake_read_fd(client) == -1);
  assert(VescClientTestAccess::wake_write_fd(client) == -1);
}

void test_destruction_after_async_transport_failure_is_safe() {
  WriteFailingFakeTransport fake;

  VescConfig config;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake](const VescConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };

  {
    VescClient client(config);
    assert(client.connect());
    assert(client.set_rpm(1234));
    wait_until([&] { return !client.is_connected(); }, 500ms,
               "async transport failure did not stop the client before destruction");
  }
}

} // namespace

int main() {
  test_client_polling_and_subscriptions();
  test_subscription_cleanup_after_disconnect();
  test_subscription_handle_can_outlive_client_destruction();
  test_poll_timeout_recovers();
  test_imu_timeout_does_not_starve_motor_polling();
  test_control_command_delivery();
  test_brake_current_command_requires_positive_limit();
  test_watchdog_brake_current_safe_stop();
  test_watchdog_coast_safe_stop();
  test_connect_disconnect_edges();
  test_runtime_poll_interval_updates();
  test_config_snapshot_tracks_runtime_polling_behavior();
  test_concurrent_poll_updates_keep_client_responsive();
  test_queued_query_deadline_expires_while_older_query_waits();
  test_late_fw_reply_does_not_satisfy_newer_query();
  test_disconnect_unblocks_query();
  test_disconnect_unblocks_blocked_write_wait();
  test_command_rejected_after_disconnect_state_wins_queue_race();
  test_disconnect_cleans_up_after_async_transport_failure();
  test_destruction_after_async_transport_failure_is_safe();
  return 0;
}
