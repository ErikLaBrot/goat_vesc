#include "test_support.hpp"

#include "goat_motor_controller/packet_parser.hpp"
#include "goat_motor_controller/protocol_ids.hpp"
#include "goat_motor_controller/controller_client.hpp"
#include "goat_motor_controller/controller_protocol.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace goat_motor_controller {

struct ControllerClientTestAccess {
  static auto scheduler_mutex(ControllerClient* client) -> std::mutex& {
    return client->scheduler_mutex_;
  }

  static auto management_mutex(ControllerClient* client) -> std::timed_mutex& {
    return client->management_mutex_;
  }

  static auto running(ControllerClient* client) -> std::atomic<bool>& {
    return client->running_;
  }

  static auto command_queue(ControllerClient* client) -> std::deque<std::vector<std::uint8_t>>& {
    return client->command_queue_;
  }

  static auto due_watchdog_command(ControllerClient& client)
      -> std::optional<std::vector<std::uint8_t>> {
    return client.dequeue_due_watchdog_command(std::chrono::steady_clock::now());
  }

  static auto imu_subscription_count(const ControllerClient& client) -> std::size_t {
    const auto callbacks = client.callback_registry_;
    std::lock_guard lock(callbacks->mutex);
    return callbacks->imu_callbacks.size();
  }

  static auto motor_subscription_count(const ControllerClient& client) -> std::size_t {
    const auto callbacks = client.callback_registry_;
    std::lock_guard lock(callbacks->mutex);
    return callbacks->motor_callbacks.size();
  }

  static auto io_thread_joinable(ControllerClient& client) -> bool {
    return client.io_thread_.joinable();
  }

  static auto fd(const ControllerClient& client) -> int {
    return client.fd_;
  }

  static auto wake_read_fd(const ControllerClient& client) -> int {
    return client.wake_pipe_[0];
  }

  static auto wake_write_fd(const ControllerClient& client) -> int {
    return client.wake_pipe_[1];
  }
};

} // namespace goat_motor_controller

namespace {

using namespace goat_motor_controller;
using namespace std::chrono_literals;
using namespace test_support;

std::vector<std::uint8_t> make_fw_response() {
  return frame_payload({
      static_cast<std::uint8_t>(CommandId::FwVersion),
      6,
      5,
  });
}

std::vector<std::uint8_t> make_imu_response(float seed) {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(CommandId::GetImuData));
  append_u16(payload, 0xFFFFU);
  for (int i = 0; i < 16; ++i) {
    append_f32(payload, seed + static_cast<float>(i) * 0.25f);
  }
  return frame_payload(payload);
}

std::vector<std::uint8_t> make_values_response(std::int32_t rpm) {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(CommandId::GetValues));
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
  payload.push_back(0);
  return frame_payload(payload);
}

std::vector<std::uint8_t> make_config_response(CommandId id,
                                               const std::vector<std::uint8_t>& image) {
  std::vector<std::uint8_t> payload{static_cast<std::uint8_t>(id)};
  payload.insert(payload.end(), image.begin(), image.end());
  return frame_payload(payload);
}

std::vector<std::uint8_t> make_bool_ack(CommandId id, bool accepted) {
  return frame_payload({
      static_cast<std::uint8_t>(id),
      static_cast<std::uint8_t>(accepted),
  });
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

bool is_readable(int fd) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  struct timeval timeout {};
  const int rc = ::select(fd + 1, &rfds, nullptr, nullptr, &timeout);
  return rc > 0 && FD_ISSET(fd, &rfds);
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

template <typename Loader>
auto wait_for_quiescence(Loader load, std::chrono::milliseconds timeout,
                         std::chrono::milliseconds stable_for, const char* message) -> int {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int last_value = load();
  auto stable_since = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < deadline) {
    const int current = load();
    if (current != last_value) {
      last_value = current;
      stable_since = std::chrono::steady_clock::now();
    } else if (std::chrono::steady_clock::now() - stable_since >= stable_for) {
      return current;
    }
    std::this_thread::sleep_for(5ms);
  }

  throw std::runtime_error(message);
}

std::vector<std::uint8_t> read_exact(int fd, std::size_t byte_count) {
  std::vector<std::uint8_t> bytes(byte_count);
  std::size_t offset = 0;
  while (offset < byte_count) {
    wait_until([fd] { return is_readable(fd); }, 250ms, "pseudo-terminal request did not arrive");
    const ssize_t n = ::read(fd, bytes.data() + offset, byte_count - offset);
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n <= 0) {
      throw std::runtime_error("pseudo-terminal request read failed");
    }
    offset += static_cast<std::size_t>(n);
  }
  return bytes;
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

struct EofFakeTransport {
  EofFakeTransport() {
    int pipe_fds[2]{-1, -1};
    if (::pipe(pipe_fds) != 0) {
      throw std::runtime_error("pipe failed");
    }
    read_fd_.reset(pipe_fds[0]);
    write_fd_.reset(pipe_fds[1]);
  }

  bool open_client_fd(int& fd_out) {
    fd_out = read_fd_.release();
    return fd_out >= 0;
  }

  void close_peer() {
    write_fd_.reset();
  }

private:
  ScopedFd read_fd_;
  ScopedFd write_fd_;
};

struct PseudoTerminal {
  PseudoTerminal() : master_fd_(::posix_openpt(O_RDWR | O_NOCTTY)) {
    if (master_fd_.get() < 0 || ::grantpt(master_fd_.get()) != 0 ||
        ::unlockpt(master_fd_.get()) != 0) {
      throw std::runtime_error("pseudo-terminal setup failed");
    }
    const char* path = ::ptsname(master_fd_.get());
    if (path == nullptr) {
      throw std::runtime_error("pseudo-terminal path lookup failed");
    }
    slave_path = path;
  }

  int master_fd() const {
    return master_fd_.get();
  }

  void close_master() {
    master_fd_.reset();
  }

  std::string slave_path;

private:
  ScopedFd master_fd_;
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

    server_fd_.reset(server_fd.release());
    client_fd_.reset(client_fd.release());
    observer_fd_.reset(observer_fd.release());
  }

  bool open_client_fd(int& fd_out) {
    if (client_fd_.get() < 0) {
      return false;
    }

    fd_out = client_fd_.release();
    return true;
  }

  void wait_until_blocked(std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!is_writable(observer_fd_.get())) {
        return;
      }
      std::this_thread::sleep_for(5ms);
    }

    throw std::runtime_error("client transport never reached blocked-write state; actual "
                             "SO_SNDBUF=" +
                             std::to_string(actual_send_buffer_bytes_));
  }

  void fill_send_buffer() const {
    std::array<std::uint8_t, 512> bytes{};
    while (::write(observer_fd_.get(), bytes.data(), bytes.size()) > 0) {
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      throw std::runtime_error("failed to fill fake send buffer");
    }
  }

private:
  int actual_send_buffer_bytes_{0};
  ScopedFd server_fd_;
  ScopedFd client_fd_;
  ScopedFd observer_fd_;
};

struct FakeVesc {
  struct Behavior {
    int dropped_imu_replies{0};
    int dropped_value_replies{0};
    bool respond_to_fw{true};
    std::vector<std::chrono::milliseconds> fw_response_delays;

    static auto drop_imu_replies(int count) -> Behavior {
      return Behavior{count, 0, true, {}};
    }

    static auto without_fw_response() -> Behavior {
      return Behavior{0, 0, false, {}};
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
    server_fd_.reset(fds[0]);
    client_fd_.reset(fds[1]);
    const int flags = ::fcntl(server_fd_.get(), F_GETFL, 0);
    (void)::fcntl(server_fd_.get(), F_SETFL, flags | O_NONBLOCK);
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
  }

  bool open_client_fd(int& fd_out) {
    if (client_fd_.get() < 0) {
      return false;
    }
    fd_out = client_fd_.release();
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
  std::atomic<int> motor_config_writes{0};
  std::atomic<int> app_config_reads{0};
  std::atomic<int> app_config_persistent_writes{0};
  std::atomic<int> app_config_volatile_writes{0};
  std::atomic<int> lisp_reads{0};
  std::atomic<int> lisp_erases{0};
  std::atomic<int> lisp_writes{0};
  std::atomic<int> lisp_run_commands{0};
  std::atomic<int> foc_requests{0};
  std::atomic<int> app_disable_commands{0};
  std::atomic<std::int32_t> last_current_raw{0};
  std::atomic<std::int32_t> last_duty_raw{0};
  std::atomic<std::int32_t> last_brake_current_raw{0};
  std::atomic<std::int16_t> last_servo_raw{0};
  std::atomic<bool> respond_to_management{true};
  std::atomic<bool> reject_lisp_write{false};
  std::atomic<bool> malformed_foc_reply{false};
  std::atomic<bool> send_unsolicited_foc_configs{true};
  std::atomic<std::int16_t> foc_result{0};
  std::atomic<int> foc_response_delay_ms{0};

  std::vector<std::uint8_t> management_history() const {
    std::lock_guard lock(management_history_mutex_);
    return management_history_;
  }

  void clear_management_history() {
    std::lock_guard lock(management_history_mutex_);
    management_history_.clear();
  }

  std::vector<std::int32_t> app_disable_durations() const {
    std::lock_guard lock(app_disable_mutex_);
    return app_disable_durations_;
  }

private:
  void run() {
    PacketParser parser;
    float imu_seed = 1.0f;
    std::int32_t rpm_value = 1400;

    while (running_.load()) {
      std::uint8_t buffer[512];
      const ssize_t n = ::read(server_fd_.get(), buffer, sizeof(buffer));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          if (auto payload = parser.feed_byte(buffer[static_cast<std::size_t>(i)])) {
            const auto id = payload->front();
            if (id == static_cast<std::uint8_t>(CommandId::GetImuData)) {
              const int count = ++imu_requests;
              if (count <= dropped_imu_replies_) {
                continue;
              }
              write_all(server_fd_.get(), make_imu_response(imu_seed));
              imu_seed += 1.0f;
            } else if (id == static_cast<std::uint8_t>(CommandId::GetValues)) {
              const int count = ++value_requests;
              if (count <= dropped_value_replies_) {
                continue;
              }
              write_all(server_fd_.get(), make_values_response(rpm_value));
              rpm_value += 25;
            } else if (id == static_cast<std::uint8_t>(CommandId::FwVersion)) {
              const int request_index = fw_requests.fetch_add(1);
              if (!respond_fw_) {
                continue;
              }
              const auto request_slot = static_cast<std::size_t>(request_index);
              const auto delay = request_slot < fw_response_delays_.size()
                                     ? fw_response_delays_[request_slot]
                                     : std::chrono::milliseconds::zero();
              if (delay <= std::chrono::milliseconds::zero()) {
                write_all(server_fd_.get(), make_fw_response());
              } else {
                schedule_response(delay, make_fw_response());
              }
            } else if (id ==
                       static_cast<std::uint8_t>(CommandId::GetMotorConfig)) {
              record_management(id);
              if (respond_to_management.load()) {
                write_all(server_fd_.get(),
                          make_config_response(CommandId::GetMotorConfig, motor_config_));
              }
            } else if (id ==
                       static_cast<std::uint8_t>(CommandId::SetMotorConfig)) {
              record_management(id);
              ++motor_config_writes;
              if (respond_to_management.load() && payload->size() >= 5 &&
                  std::equal(payload->begin() + 1, payload->begin() + 5,
                             motor_config_.begin())) {
                motor_config_.assign(payload->begin() + 1, payload->end());
                write_all(server_fd_.get(), frame_payload({id}));
              }
            } else if (id ==
                       static_cast<std::uint8_t>(CommandId::GetAppConfig)) {
              record_management(id);
              ++app_config_reads;
              if (respond_to_management.load()) {
                write_all(server_fd_.get(),
                          make_config_response(CommandId::GetAppConfig, app_config_));
              }
            } else if (id == static_cast<std::uint8_t>(CommandId::SetAppConfig) ||
                       id ==
                           static_cast<std::uint8_t>(CommandId::SetAppConfigNoStore)) {
              record_management(id);
              if (id == static_cast<std::uint8_t>(CommandId::SetAppConfig)) {
                ++app_config_persistent_writes;
              } else {
                ++app_config_volatile_writes;
              }
              if (respond_to_management.load() && payload->size() >= 5 &&
                  std::equal(payload->begin() + 1, payload->begin() + 5,
                             app_config_.begin())) {
                app_config_.assign(payload->begin() + 1, payload->end());
                write_all(server_fd_.get(), frame_payload({id}));
              }
            } else if (id == static_cast<std::uint8_t>(CommandId::LispReadCode)) {
              record_management(id);
              ++lisp_reads;
              if (!respond_to_management.load() || payload->size() != 9) {
                continue;
              }
              const auto length = read_i32(*payload, 1);
              const auto offset = read_i32(*payload, 5);
              if (lisp_code_.empty()) {
                std::vector<std::uint8_t> reply{id};
                append_i32(reply, 0);
                append_i32(reply, 0);
                write_all(server_fd_.get(), frame_payload(reply));
              } else if (length > 0 && offset >= 0 &&
                         static_cast<std::size_t>(offset + length) <= lisp_code_.size()) {
                std::vector<std::uint8_t> reply{id};
                append_i32(reply, static_cast<std::int32_t>(lisp_code_.size()));
                append_i32(reply, offset);
                reply.insert(reply.end(), lisp_code_.begin() + offset,
                             lisp_code_.begin() + offset + length);
                write_all(server_fd_.get(), frame_payload(reply));
              }
            } else if (id == static_cast<std::uint8_t>(CommandId::LispEraseCode)) {
              record_management(id);
              ++lisp_erases;
              if (respond_to_management.load()) {
                lisp_code_.clear();
                packed_lisp_.clear();
                write_all(server_fd_.get(), make_bool_ack(CommandId::LispEraseCode, true));
              }
            } else if (id == static_cast<std::uint8_t>(CommandId::LispWriteCode)) {
              record_management(id);
              ++lisp_writes;
              if (!respond_to_management.load() || payload->size() < 6) {
                continue;
              }
              const auto offset = static_cast<std::uint32_t>(read_i32(*payload, 1));
              const bool accepted = !reject_lisp_write.exchange(false);
              if (accepted) {
                const auto chunk_size = payload->size() - 5;
                packed_lisp_.resize(std::max(packed_lisp_.size(),
                                             static_cast<std::size_t>(offset) + chunk_size));
                std::copy(payload->begin() + 5, payload->end(),
                          packed_lisp_.begin() + offset);
                if (packed_lisp_.size() >= 8) {
                  const auto code_size = static_cast<std::uint32_t>(read_i32(packed_lisp_, 0));
                  if (packed_lisp_.size() >= static_cast<std::size_t>(code_size) + 8U) {
                    lisp_code_.assign(packed_lisp_.begin() + 8,
                                      packed_lisp_.begin() + 8 + code_size);
                  }
                }
              }
              std::vector<std::uint8_t> reply{id, static_cast<std::uint8_t>(accepted)};
              append_i32(reply, static_cast<std::int32_t>(offset));
              write_all(server_fd_.get(), frame_payload(reply));
            } else if (id == static_cast<std::uint8_t>(CommandId::LispSetRunning)) {
              record_management(id);
              ++lisp_run_commands;
              if (respond_to_management.load() && payload->size() == 2) {
                write_all(server_fd_.get(), make_bool_ack(CommandId::LispSetRunning, true));
              }
            } else if (id ==
                           static_cast<std::uint8_t>(CommandId::AppDisableOutput) &&
                       payload->size() == 6) {
              record_management(id);
              ++app_disable_commands;
              std::lock_guard lock(app_disable_mutex_);
              app_disable_durations_.push_back(read_i32(*payload, 2));
            } else if (id ==
                           static_cast<std::uint8_t>(CommandId::DetectApplyAllFoc) &&
                       payload->size() == 22) {
              record_management(id);
              ++foc_requests;
              if (!respond_to_management.load()) {
                continue;
              }
              if (send_unsolicited_foc_configs.load()) {
                write_all(server_fd_.get(),
                          make_config_response(CommandId::GetMotorConfig, motor_config_));
                write_all(server_fd_.get(),
                          make_config_response(CommandId::GetAppConfig, app_config_));
              }
              std::vector<std::uint8_t> reply{id};
              append_i16(reply, foc_result.load());
              if (malformed_foc_reply.load()) {
                reply.pop_back();
              }
              const auto framed = frame_payload(reply);
              const auto delay = std::chrono::milliseconds(foc_response_delay_ms.load());
              if (delay > std::chrono::milliseconds::zero()) {
                schedule_response(delay, framed);
              } else {
                write_all(server_fd_.get(), framed);
              }
            } else if (id == static_cast<std::uint8_t>(CommandId::SetRpm) &&
                       payload->size() == 5) {
              ++rpm_commands;
            } else if (id == static_cast<std::uint8_t>(CommandId::SetCurrent) &&
                       payload->size() == 5) {
              last_current_raw.store(read_i32(*payload, 1));
              ++current_commands;
            } else if (id == static_cast<std::uint8_t>(CommandId::SetDuty) &&
                       payload->size() == 5) {
              last_duty_raw.store(read_i32(*payload, 1));
              ++duty_commands;
            } else if (id == static_cast<std::uint8_t>(CommandId::SetCurrentBrake) &&
                       payload->size() == 5) {
              last_brake_current_raw.store(read_i32(*payload, 1));
              ++brake_current_commands;
            } else if (id == static_cast<std::uint8_t>(CommandId::SetServoPos) &&
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

  ScopedFd server_fd_;
  ScopedFd client_fd_;
  std::atomic<bool> running_{true};
  int dropped_imu_replies_{0};
  int dropped_value_replies_{0};
  bool respond_fw_{true};
  std::vector<std::chrono::milliseconds> fw_response_delays_;
  std::vector<std::uint8_t> motor_config_{0x10, 0x20, 0x30, 0x40, 0x01};
  std::vector<std::uint8_t> app_config_{0x50, 0x60, 0x70, 0x80, 0x02};
  std::vector<std::uint8_t> lisp_code_ = std::vector<std::uint8_t>(425, 0x2A);
  std::vector<std::uint8_t> packed_lisp_;
  mutable std::mutex management_history_mutex_;
  std::vector<std::uint8_t> management_history_;
  mutable std::mutex app_disable_mutex_;
  std::vector<std::int32_t> app_disable_durations_;
  std::mutex response_threads_mutex_;
  std::vector<std::thread> response_threads_;
  std::thread worker_;

  void record_management(std::uint8_t id) {
    std::lock_guard lock(management_history_mutex_);
    management_history_.push_back(id);
  }

  void schedule_response(std::chrono::milliseconds delay, std::vector<std::uint8_t> packet) {
    std::lock_guard lock(response_threads_mutex_);
    response_threads_.emplace_back([this, delay, packet = std::move(packet)]() mutable {
      std::this_thread::sleep_for(delay);
      if (!running_.load()) {
        return;
      }
      (void)try_write_all(server_fd_.get(), packet);
    });
  }
};

template <typename FakeTransport> ControllerConfig config_for(FakeTransport& fake) {
  ControllerConfig config;
  config.open_serial_fn = [&fake](const ControllerConfig&, int& fd_out) {
    return fake.open_client_fd(fd_out);
  };
  return config;
}

void test_client_polling_and_subscriptions() {
  FakeVesc fake;
  std::atomic<std::uint64_t> stamp_counter{1000};

  auto config = config_for(fake);
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 40ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(1000); };

  ControllerClient client(config);
  std::atomic<int> imu_callbacks{0};
  std::atomic<int> current_from_callback{0};

  auto imu_handle = client.subscribe_imu([&](const ImuData& data) {
    ++imu_callbacks;
    assert(data.stamp_ns >= 1000);
    (void)client.latest_imu();
    if (current_from_callback.fetch_add(1) == 0) {
      assert(client.set_current(1.5f));
    }
  });

  auto motor_handle = client.subscribe_motor_state([&](const MotorState& state) {
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

  wait_until([&] { return fake.rpm_commands.load() > 0; }, 500ms,
             "coalesced rpm commands were not delivered");
  client.disconnect();

  (void)imu_handle;
  (void)motor_handle;
}

void test_callback_failures_and_reentrant_shutdown_are_contained() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 10ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  std::atomic<int> throwing_callbacks{0};
  std::atomic<bool> callback_returned{false};
  std::atomic<bool> callback_query_was_empty{false};

  auto throwing_handle = client.subscribe_imu([&](const ImuData&) {
    ++throwing_callbacks;
    throw std::runtime_error("subscriber failure");
  });
  auto shutdown_handle = client.subscribe_imu([&](const ImuData&) {
    if (callback_returned.load()) {
      return;
    }
    callback_query_was_empty.store(!client.request_fw_version(100ms).has_value());
    client.disconnect();
    callback_returned.store(true);
  });

  assert(client.connect());
  wait_until([&] { return callback_returned.load(); }, 500ms,
             "reentrant callback operations did not return");
  assert(callback_query_was_empty.load());
  assert(throwing_callbacks.load() > 0);
  wait_until([&] { return !client.is_connected(); }, 500ms,
             "callback disconnect did not stop the I/O loop");

  client.disconnect();
  assert(!ControllerClientTestAccess::io_thread_joinable(client));
  assert(ControllerClientTestAccess::fd(client) == -1);

  (void)throwing_handle;
  (void)shutdown_handle;
}

void test_subscription_cleanup_after_disconnect() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 30ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  std::atomic<int> imu_callbacks{0};
  std::atomic<int> motor_callbacks{0};

  auto imu_handle = client.subscribe_imu([&](const ImuData&) { ++imu_callbacks; });
  auto motor_handle =
      client.subscribe_motor_state([&](const MotorState&) { ++motor_callbacks; });

  assert(ControllerClientTestAccess::imu_subscription_count(client) == 1);
  assert(ControllerClientTestAccess::motor_subscription_count(client) == 1);

  assert(client.connect());
  wait_until([&] { return imu_callbacks.load() >= 1; }, 500ms, "imu callback never fired");
  wait_until([&] { return motor_callbacks.load() >= 1; }, 500ms, "motor callback never fired");

  client.disconnect();

  imu_handle.reset();
  motor_handle.reset();

  assert(!imu_handle);
  assert(!motor_handle);
  assert(ControllerClientTestAccess::imu_subscription_count(client) == 0);
  assert(ControllerClientTestAccess::motor_subscription_count(client) == 0);
  assert(!client.is_connected());
}

void test_subscription_handle_can_outlive_client_destruction() {
  std::optional<ControllerClient::SubscriptionHandle> imu_handle;
  std::optional<ControllerClient::SubscriptionHandle> motor_handle;

  {
    ControllerClient client(ControllerConfig{});
    imu_handle.emplace(client.subscribe_imu([](const ImuData&) {}));
    motor_handle.emplace(client.subscribe_motor_state([](const MotorState&) {}));

    assert(ControllerClientTestAccess::imu_subscription_count(client) == 1);
    assert(ControllerClientTestAccess::motor_subscription_count(client) == 1);
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
  auto config = config_for(fake);
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 30ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
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
  auto config = config_for(fake);
  config.imu_poll_interval = 10ms;
  config.motor_poll_interval = 15ms;
  config.poll_response_timeout = 30ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());

  wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms,
             "imu poll did not hit repeated timeout path");
  wait_until([&] { return client.latest_motor_state().has_value(); }, 250ms,
             "motor polling was starved by imu timeout recovery");
  wait_until([&] { return client.latest_imu().has_value(); }, 500ms, "imu cache never recovered");

  client.disconnect();
}

void test_control_command_delivery() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.max_brake_current = 1.0f;

  ControllerClient client(config);
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

void test_invalid_control_commands_are_rejected() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.max_brake_current = 1.0f;

  ControllerClient client(config);
  assert(client.connect());

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const float largest = std::numeric_limits<float>::max();
  assert(!client.set_duty(nan));
  assert(!client.set_duty(1.01f));
  assert(!client.set_current(infinity));
  assert(!client.set_current(largest));
  assert(!client.set_current_brake(nan));
  assert(!client.set_servo_pos(nan));
  assert(!client.set_servo_pos(1.01f));

  std::this_thread::sleep_for(20ms);
  assert(fake.duty_commands.load() == 0);
  assert(fake.current_commands.load() == 0);
  assert(fake.brake_current_commands.load() == 0);
  assert(fake.servo_commands.load() == 0);

  client.disconnect();
}

void test_control_queue_keeps_only_latest_command_per_type() {
  ControllerClient client(ControllerConfig{});
  ControllerClientTestAccess::running(&client).store(true);

  assert(client.set_rpm(1000));
  assert(client.set_rpm(2000));
  assert(client.set_duty(0.1f));
  assert(client.set_rpm(3000));

  const auto& queue = ControllerClientTestAccess::command_queue(&client);
  assert(queue.size() == 2);
  assert(queue[0][2] == static_cast<std::uint8_t>(CommandId::SetDuty));
  assert(queue[1][2] == static_cast<std::uint8_t>(CommandId::SetRpm));
  assert(read_i32(queue[1], 3) == 3000);

  ControllerClientTestAccess::running(&client).store(false);
}

void test_watchdog_discards_stale_control_backlog() {
  ControllerConfig config;
  config.command_watchdog_timeout = 1ms;
  config.command_watchdog_action = ControlWatchdogAction::Coast;

  ControllerClient client(config);
  ControllerClientTestAccess::running(&client).store(true);
  assert(client.set_rpm(1000));
  assert(client.set_duty(0.1f));

  std::this_thread::sleep_for(2ms);
  const auto watchdog = ControllerClientTestAccess::due_watchdog_command(client);
  assert(watchdog.has_value());
  assert(!watchdog->empty());
  assert(ControllerClientTestAccess::command_queue(&client).empty());

  ControllerClientTestAccess::running(&client).store(false);
}

void test_brake_current_command_requires_positive_limit() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
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
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.command_watchdog_timeout = 40ms;
  config.command_watchdog_action = ControlWatchdogAction::BrakeCurrent;
  config.max_brake_current = 1.5f;
  config.command_watchdog_brake_current = 2.5f;

  ControllerClient client(config);
  assert(client.connect());

  assert(client.set_rpm(1800));
  wait_until([&] { return fake.rpm_commands.load() == 1; }, 500ms,
             "rpm command was not delivered before watchdog arming");

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
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.command_watchdog_timeout = 30ms;
  config.command_watchdog_action = ControlWatchdogAction::Coast;

  ControllerClient client(config);
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

  ControllerConfig config;
  config.imu_poll_interval = kNoPolling;
  config.motor_poll_interval = kNoPolling;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.open_serial_fn = [&fake, &open_calls](const ControllerConfig&, int& fd_out) {
    ++open_calls;
    return fake.open_client_fd(fd_out);
  };

  ControllerClient client(config);

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

void test_concurrent_lifecycle_calls_are_serialized() {
  FakeVesc fake;
  std::atomic<int> open_calls{0};
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.open_serial_fn = [&fake, &open_calls](const ControllerConfig&, int& fd_out) {
    ++open_calls;
    std::this_thread::sleep_for(20ms);
    return fake.open_client_fd(fd_out);
  };

  ControllerClient client(config);
  std::promise<void> start;
  const auto ready = start.get_future().share();
  auto first = std::async(std::launch::async, [&] {
    ready.wait();
    return client.connect();
  });
  auto second = std::async(std::launch::async, [&] {
    ready.wait();
    return client.connect();
  });
  start.set_value();

  assert(first.get());
  assert(second.get());
  assert(open_calls.load() == 1);

  auto first_disconnect = std::async(std::launch::async, [&] { client.disconnect(); });
  auto second_disconnect = std::async(std::launch::async, [&] { client.disconnect(); });
  first_disconnect.get();
  second_disconnect.get();
  assert(!client.is_connected());
}

auto test_runtime_poll_interval_updates() -> void {
  constexpr auto kInitialStampNs = 1000ULL;
  constexpr auto kStampStepNs = 1000ULL;
  constexpr auto kNoPolling = 0ms;

  FakeVesc fake;
  std::atomic<std::uint64_t> stamp_counter{kInitialStampNs};

  auto config = config_for(fake);
  config.imu_poll_interval = kNoPolling;
  config.motor_poll_interval = kNoPolling;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 5ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(kStampStepNs); };

  ControllerClient client(config);
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

  client.set_imu_poll_interval(-1ms);
  client.set_motor_poll_interval(0ms);

  const int quiet_imu_requests =
      wait_for_quiescence([&] { return fake.imu_requests.load(); }, 300ms, 50ms,
                          "imu polling never quiesced");
  const int quiet_motor_requests =
      wait_for_quiescence([&] { return fake.value_requests.load(); }, 300ms, 50ms,
                          "motor polling never quiesced");

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

void test_shorter_poll_interval_takes_effect_immediately() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 10s;

  ControllerClient client(config);
  assert(client.connect());
  wait_until([&] { return fake.value_requests.load() == 1; }, 500ms,
             "initial motor poll did not run");

  client.set_motor_poll_interval(10ms);
  wait_until([&] { return fake.value_requests.load() >= 2; }, 500ms,
             "shorter motor poll interval kept the old deadline");
  client.disconnect();
}

auto test_config_snapshot_tracks_runtime_polling_behavior() -> void {
  ControllerConfig config;
  config.imu_poll_interval = 20ms;
  config.motor_poll_interval = 35ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 7ms;
  config.command_watchdog_timeout = 40ms;
  config.command_watchdog_action = ControlWatchdogAction::BrakeCurrent;
  config.max_brake_current = 3.0f;
  config.command_watchdog_brake_current = 2.5f;

  ControllerClient client(config);

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

  auto config = config_for(fake);
  config.imu_poll_interval = 25ms;
  config.motor_poll_interval = 35ms;
  config.poll_response_timeout = 15ms;
  config.query_guard_window = 2ms;
  config.wall_time_ns = [&stamp_counter] { return stamp_counter.fetch_add(kStampStepNs); };

  ControllerClient client(config);
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

void test_configuration_management_and_polling() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 15ms;
  config.motor_poll_interval = 25ms;
  config.poll_response_timeout = 10ms;
  config.query_guard_window = 2ms;

  ControllerClient client(config);
  assert(client.connect());
  wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms,
             "polling did not start before configuration operations");

  const auto motor = client.request_motor_config(500ms);
  assert(motor &&
         motor->bytes == std::vector<std::uint8_t>({0x10, 0x20, 0x30, 0x40, 0x01}));

  auto updated_motor = *motor;
  updated_motor.bytes.back() = 0x22;
  assert(client.write_motor_config(updated_motor, 500ms) == OperationResult::Success);
  assert(client.request_motor_config(500ms)->bytes == updated_motor.bytes);

  auto incompatible_motor = updated_motor;
  incompatible_motor.bytes.front() ^= 0xFF;
  const auto writes_before_rejection = fake.motor_config_writes.load();
  assert(client.write_motor_config(incompatible_motor, 500ms) ==
         OperationResult::IncompatibleData);
  assert(fake.motor_config_writes.load() == writes_before_rejection);
  assert(client.write_motor_config(MotorConfigImage{{1, 2, 3}}, 500ms) ==
         OperationResult::InvalidData);

  const auto app = client.request_app_config(500ms);
  assert(app &&
         app->bytes == std::vector<std::uint8_t>({0x50, 0x60, 0x70, 0x80, 0x02}));

  auto volatile_app = *app;
  volatile_app.bytes.back() = 0x33;
  assert(client.write_app_config(volatile_app, AppConfigStorage::Volatile, 500ms) ==
         OperationResult::Success);
  assert(fake.app_config_volatile_writes.load() == 1);

  auto persistent_app = volatile_app;
  persistent_app.bytes.back() = 0x44;
  assert(client.write_app_config(persistent_app, AppConfigStorage::Persistent, 500ms) ==
         OperationResult::Success);
  assert(fake.app_config_persistent_writes.load() == 1);
  assert(client.request_app_config(500ms)->bytes == persistent_app.bytes);

  const auto polls_before = fake.imu_requests.load();
  wait_until([&] { return fake.imu_requests.load() > polls_before; }, 500ms,
             "polling did not resume after configuration operations");
  client.disconnect();
}

void test_management_operations_are_serialized() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());
  const auto current = client.request_app_config(500ms);
  assert(current);

  auto first = *current;
  auto second = *current;
  first.bytes.back() = 0xA1;
  second.bytes.back() = 0xA2;
  fake.clear_management_history();

  auto first_write = std::async(std::launch::async, [&] {
    return client.write_app_config(first, AppConfigStorage::Persistent, 500ms);
  });
  auto second_write = std::async(std::launch::async, [&] {
    return client.write_app_config(second, AppConfigStorage::Persistent, 500ms);
  });

  assert(first_write.get() == OperationResult::Success);
  assert(second_write.get() == OperationResult::Success);
  const auto history = fake.management_history();
  assert(history == std::vector<std::uint8_t>({
                        static_cast<std::uint8_t>(CommandId::GetAppConfig),
                        static_cast<std::uint8_t>(CommandId::SetAppConfig),
                        static_cast<std::uint8_t>(CommandId::GetAppConfig),
                        static_cast<std::uint8_t>(CommandId::SetAppConfig),
                    }));
  client.disconnect();
}

void test_lisp_management() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());

  const auto original = client.request_lisp_code(500ms);
  assert(original && original->bytes.size() == 425);
  assert(fake.lisp_reads.load() == 3);

  LispCodeImage replacement{std::vector<std::uint8_t>(800, 0x3A)};
  assert(client.write_lisp_code(replacement, 1s) == OperationResult::Success);
  assert(fake.lisp_erases.load() == 1);
  assert(fake.lisp_writes.load() == 3);
  const auto reread = client.request_lisp_code(1s);
  assert(reread && reread->bytes == replacement.bytes);

  assert(client.set_lisp_running(false, 500ms) == OperationResult::Success);
  assert(client.set_lisp_running(true, 500ms) == OperationResult::Success);
  assert(fake.lisp_run_commands.load() == 2);

  fake.reject_lisp_write.store(true);
  assert(client.write_lisp_code(replacement, 1s) == OperationResult::Rejected);
  assert(client.write_lisp_code(LispCodeImage{}, 500ms) ==
         OperationResult::InvalidData);

  assert(client.erase_lisp_code(500ms) == OperationResult::Success);
  const auto empty = client.request_lisp_code(500ms);
  assert(empty && empty->bytes.empty());
  client.disconnect();
}

void test_foc_calibration_and_polling() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 15ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());
  wait_until([&] { return fake.imu_requests.load() >= 2; }, 500ms,
             "polling did not start before FOC calibration");

  const FocCalibrationParameters parameters{50.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const auto success = client.run_foc_calibration(parameters, 500ms);
  assert(success.operation == OperationResult::Success);
  assert(success.firmware_code == 0);
  wait_until([&] { return fake.app_disable_commands.load() == 2; }, 500ms,
             "FOC calibration did not reenable application output");
  assert(fake.app_disable_durations() == std::vector<std::int32_t>({5500, 0}));

  const auto requests_before_invalid = fake.foc_requests.load();
  auto invalid = parameters;
  invalid.max_power_loss_w = std::numeric_limits<float>::quiet_NaN();
  const auto invalid_result = client.run_foc_calibration(invalid, 500ms);
  assert(invalid_result.operation == OperationResult::InvalidData);
  assert(!invalid_result.firmware_code);
  assert(fake.foc_requests.load() == requests_before_invalid);

  fake.foc_result.store(-10);
  const auto rejected = client.run_foc_calibration(parameters, 500ms);
  assert(rejected.operation == OperationResult::Rejected);
  assert(rejected.firmware_code == -10);

  fake.malformed_foc_reply.store(true);
  const auto malformed = client.run_foc_calibration(parameters, 500ms);
  assert(malformed.operation == OperationResult::Rejected);
  assert(!malformed.firmware_code);

  const auto polls_before = fake.imu_requests.load();
  wait_until([&] { return fake.imu_requests.load() > polls_before; }, 500ms,
             "polling did not resume after FOC calibration");
  client.disconnect();
}

void test_foc_calibration_serializes_with_management_operations() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());
  const auto app = client.request_app_config(500ms);
  assert(app);

  fake.clear_management_history();
  fake.foc_response_delay_ms.store(40);
  auto calibration = std::async(std::launch::async, [&] {
    return client.run_foc_calibration({50.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 500ms);
  });
  wait_until([&] { return fake.foc_requests.load() == 1; }, 250ms,
             "FOC calibration request was not sent");

  auto updated_app = *app;
  updated_app.bytes.back() = 0x77;
  auto app_write = std::async(std::launch::async, [&] {
    return client.write_app_config(updated_app, AppConfigStorage::Persistent, 500ms);
  });

  assert(calibration.get().operation == OperationResult::Success);
  assert(app_write.get() == OperationResult::Success);
  wait_until([&] { return fake.app_disable_commands.load() == 2; }, 250ms,
             "application output reenable was not sent");
  assert(fake.management_history() ==
         std::vector<std::uint8_t>({
             static_cast<std::uint8_t>(CommandId::AppDisableOutput),
             static_cast<std::uint8_t>(CommandId::DetectApplyAllFoc),
             static_cast<std::uint8_t>(CommandId::AppDisableOutput),
             static_cast<std::uint8_t>(CommandId::GetAppConfig),
             static_cast<std::uint8_t>(CommandId::SetAppConfig),
         }));
  client.disconnect();
}

void test_foc_calibration_timeout_stops_connection() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());
  fake.respond_to_management.store(false);
  const auto result =
      client.run_foc_calibration({50.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30ms);
  assert(result.operation == OperationResult::NoReply);
  assert(!result.firmware_code);
  wait_until([&] { return !client.is_connected(); }, 250ms,
             "FOC timeout did not stop the connection");
  assert(fake.foc_requests.load() == 1);
  assert(fake.app_disable_commands.load() == 1);
  client.disconnect();
}

void test_management_timeout_stops_connection() {
  FakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());

  auto& management_mutex = ControllerClientTestAccess::management_mutex(&client);
  std::unique_lock held(management_mutex);
  auto lock_timeout =
      std::async(std::launch::async, [&client] { return client.request_app_config(30ms); });
  assert(!lock_timeout.get().has_value());
  assert(fake.app_config_reads.load() == 0);
  held.unlock();

  fake.respond_to_management.store(false);
  assert(!client.request_motor_config(30ms).has_value());
  wait_until([&] { return !client.is_connected(); }, 250ms,
             "management timeout did not stop the connection");
  assert(client.write_app_config(AppConfigImage{{1, 2, 3, 4}}, AppConfigStorage::Persistent,
                                 50ms) == OperationResult::NoReply);
  client.disconnect();
}

void test_queued_query_deadline_expires_while_older_query_waits() {
  FakeVesc fake(FakeVesc::Behavior::without_fw_response());
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());

  auto first =
      std::async(std::launch::async, [&client] { return client.request_fw_version(200ms); });
  wait_until([&] { return fake.fw_requests.load() == 1; }, 200ms, "first query was not sent");

  auto second =
      std::async(std::launch::async, [&client] { return client.request_fw_version(150ms); });
  std::this_thread::sleep_for(5ms);

  auto third =
      std::async(std::launch::async, [&client] { return client.request_fw_version(25ms); });

  const auto third_result = third.get();
  assert(!third_result.has_value());
  assert(first.wait_for(0ms) == std::future_status::timeout);

  const auto second_result = second.get();
  const auto first_result = first.get();
  assert(!second_result.has_value());
  assert(!first_result.has_value());
  assert(fake.fw_requests.load() == 1);

  client.disconnect();
}

void test_fw_query_recovers_after_timeout_and_late_reply() {
  FakeVesc fake(FakeVesc::Behavior::delayed_fw_responses({80ms, 0ms}));
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());

  const auto first = client.request_fw_version(20ms);
  assert(!first.has_value());
  assert(fake.fw_requests.load() == 1);

  const auto recovered = client.request_fw_version(100ms);
  assert(recovered.has_value());
  assert(recovered->major == 6);
  assert(recovered->minor == 5);
  assert(fake.fw_requests.load() == 2);

  std::this_thread::sleep_for(100ms);
  const auto after_late_reply = client.request_fw_version(100ms);
  assert(after_late_reply.has_value());
  assert(fake.fw_requests.load() == 3);

  client.disconnect();
}

void test_disconnect_unblocks_query() {
  FakeVesc fake(FakeVesc::Behavior::without_fw_response());
  auto config = config_for(fake);
  config.imu_poll_interval = 50ms;
  config.motor_poll_interval = 100ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());

  auto future =
      std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });

  wait_until([&] { return fake.fw_requests.load() == 1; }, 200ms, "query was not sent");
  client.disconnect();
  const auto result = future.get();
  assert(!result.has_value());
}

void test_disconnect_unblocks_blocked_write_wait() {
  BlockedWriteFakeVesc fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());

  fake.fill_send_buffer();
  assert(client.set_rpm(1000));
  fake.wait_until_blocked(500ms);

  auto query_future =
      std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });

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
  ControllerClient client(ControllerConfig{});
  auto& scheduler_mutex = ControllerClientTestAccess::scheduler_mutex(&client);
  auto& running = ControllerClientTestAccess::running(&client);

  std::unique_lock scheduler_lock(scheduler_mutex);
  running.store(true);

  auto submit = std::async(std::launch::async, [&client] { return client.set_rpm(1234); });

  wait_until([&] { return submit.wait_for(0ms) == std::future_status::timeout; }, 100ms,
             "command submission did not block behind the scheduler lock");

  running.store(false);
  scheduler_lock.unlock();

  assert(!submit.get());
  assert(ControllerClientTestAccess::command_queue(&client).empty());
}

void test_query_rejected_after_disconnect_state_wins_queue_race() {
  ControllerClient client(ControllerConfig{});
  auto& scheduler_mutex = ControllerClientTestAccess::scheduler_mutex(&client);
  auto& running = ControllerClientTestAccess::running(&client);

  std::unique_lock scheduler_lock(scheduler_mutex);
  running.store(true);
  auto query =
      std::async(std::launch::async, [&client] { return client.request_fw_version(100ms); });

  wait_until([&] { return query.wait_for(0ms) == std::future_status::timeout; }, 100ms,
             "query did not block behind the scheduler lock");
  running.store(false);
  scheduler_lock.unlock();

  assert(query.wait_for(100ms) == std::future_status::ready);
  assert(!query.get().has_value());
}

void test_disconnect_cleans_up_after_async_transport_failure() {
  WriteFailingFakeTransport fake;

  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  ControllerClient client(config);
  assert(client.connect());
  assert(ControllerClientTestAccess::io_thread_joinable(client));

  assert(client.set_rpm(1234));
  wait_until([&] { return !client.is_connected(); }, 500ms,
             "async transport failure did not stop the client");

  client.disconnect();
  client.disconnect();

  assert(!client.is_connected());
  assert(!ControllerClientTestAccess::io_thread_joinable(client));
  assert(ControllerClientTestAccess::fd(client) == -1);
  assert(ControllerClientTestAccess::wake_read_fd(client) == -1);
  assert(ControllerClientTestAccess::wake_write_fd(client) == -1);
}

void test_query_write_failure_unblocks_caller() {
  WriteFailingFakeTransport fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());

  auto query =
      std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });
  assert(query.wait_for(250ms) == std::future_status::ready);
  assert(!query.get().has_value());
  wait_until([&] { return !client.is_connected(); }, 250ms,
             "query write failure did not stop the client");
  client.disconnect();
}

void test_transport_eof_stops_client() {
  EofFakeTransport fake;
  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  ControllerClient client(config);
  assert(client.connect());
  fake.close_peer();
  wait_until([&] { return !client.is_connected(); }, 500ms,
             "transport EOF did not stop the client");
  client.disconnect();
}

void test_custom_opener_failures_are_contained() {
  int pipe_fds[2]{-1, -1};
  assert(::pipe(pipe_fds) == 0);
  ScopedFd client_fd(pipe_fds[0]);
  ScopedFd peer_fd(pipe_fds[1]);
  const int assigned_fd = client_fd.get();

  ControllerConfig rejected_config;
  rejected_config.open_serial_fn = [&client_fd](const ControllerConfig&, int& fd_out) {
    fd_out = client_fd.release();
    return false;
  };
  ControllerClient rejected_client(rejected_config);
  assert(!rejected_client.connect());
  assert(::fcntl(assigned_fd, F_GETFD) == -1);
  assert(errno == EBADF);

  ControllerConfig throwing_config;
  throwing_config.open_serial_fn = [](const ControllerConfig&, int&) -> bool {
    throw std::runtime_error("opener failure");
  };
  ControllerClient throwing_client(throwing_config);
  assert(!throwing_client.connect());
}

void test_real_tty_empty_read_does_not_stop_client() {
  PseudoTerminal tty;
  ControllerConfig config;
  config.device_path = tty.slave_path;
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;

  auto invalid_baud_config = config;
  invalid_baud_config.baud = 12345;
  ControllerClient invalid_baud_client(invalid_baud_config);
  assert(!invalid_baud_client.connect());

  ControllerClient client(config);
  assert(client.connect());
  const auto expected_request = ControllerProtocol::build_fw_version_request();
  const auto round_trip = [&] {
    auto future =
        std::async(std::launch::async, [&client] { return client.request_fw_version(500ms); });
    assert(read_exact(tty.master_fd(), expected_request.size()) == expected_request);
    write_all(tty.master_fd(), make_fw_response());
    assert(future.wait_for(250ms) == std::future_status::ready);
    assert(future.get().has_value());
  };
  round_trip();
  round_trip();
  assert(client.is_connected());

  client.disconnect();
  assert(client.connect());
  assert(client.is_connected());

  tty.close_master();
  wait_until([&] { return !client.is_connected(); }, 500ms,
             "pseudo-terminal hangup did not stop the client");
  client.disconnect();
}

void test_destruction_after_async_transport_failure_is_safe() {
  WriteFailingFakeTransport fake;

  auto config = config_for(fake);
  config.imu_poll_interval = 0ms;
  config.motor_poll_interval = 0ms;
  config.poll_response_timeout = 20ms;
  config.query_guard_window = 5ms;

  {
    ControllerClient client(config);
    assert(client.connect());
    assert(client.set_rpm(1234));
    wait_until([&] { return !client.is_connected(); }, 500ms,
               "async transport failure did not stop the client before destruction");
  }
}

} // namespace

int main() {
  test_client_polling_and_subscriptions();
  test_callback_failures_and_reentrant_shutdown_are_contained();
  test_subscription_cleanup_after_disconnect();
  test_subscription_handle_can_outlive_client_destruction();
  test_poll_timeout_recovers();
  test_imu_timeout_does_not_starve_motor_polling();
  test_control_command_delivery();
  test_invalid_control_commands_are_rejected();
  test_control_queue_keeps_only_latest_command_per_type();
  test_watchdog_discards_stale_control_backlog();
  test_brake_current_command_requires_positive_limit();
  test_watchdog_brake_current_safe_stop();
  test_watchdog_coast_safe_stop();
  test_connect_disconnect_edges();
  test_concurrent_lifecycle_calls_are_serialized();
  test_runtime_poll_interval_updates();
  test_shorter_poll_interval_takes_effect_immediately();
  test_config_snapshot_tracks_runtime_polling_behavior();
  test_concurrent_poll_updates_keep_client_responsive();
  test_configuration_management_and_polling();
  test_management_operations_are_serialized();
  test_lisp_management();
  test_foc_calibration_and_polling();
  test_foc_calibration_serializes_with_management_operations();
  test_foc_calibration_timeout_stops_connection();
  test_management_timeout_stops_connection();
  test_queued_query_deadline_expires_while_older_query_waits();
  test_fw_query_recovers_after_timeout_and_late_reply();
  test_disconnect_unblocks_query();
  test_disconnect_unblocks_blocked_write_wait();
  test_command_rejected_after_disconnect_state_wins_queue_race();
  test_query_rejected_after_disconnect_state_wins_queue_race();
  test_disconnect_cleans_up_after_async_transport_failure();
  test_query_write_failure_unblocks_caller();
  test_transport_eof_stops_client();
  test_custom_opener_failures_are_contained();
  test_real_tty_empty_read_does_not_stop_client();
  test_destruction_after_async_transport_failure_is_safe();
  return 0;
}
