#include "goat_vesc/vesc_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <glob.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace goat_vesc {

namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr auto kWriteWaitPollInterval = std::chrono::milliseconds(50);

int baud_to_constant(int baud) {
  switch (baud) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  default:
    return -1;
  }
}

bool open_serial(const std::string& path, int baud, int& fd_out) {
  const int baud_const = baud_to_constant(baud);
  if (baud_const < 0) {
    return false;
  }

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }

  struct termios tty {};
  if (::tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return false;
  }

  ::cfsetispeed(&tty, static_cast<speed_t>(baud_const));
  ::cfsetospeed(&tty, static_cast<speed_t>(baud_const));
  ::cfmakeraw(&tty);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return false;
  }

  fd_out = fd;
  return true;
}

bool set_nonblocking(int fd) {
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

float clamp_brake_current(float requested_amps, float max_brake_current) {
  if (requested_amps <= 0.0f || max_brake_current <= 0.0f) {
    return 0.0f;
  }
  return std::min(requested_amps, max_brake_current);
}

std::optional<std::chrono::microseconds> micros_until(const SteadyClock::time_point& deadline,
                                                      const SteadyClock::time_point& now) {
  if (deadline <= now) {
    return std::chrono::microseconds::zero();
  }
  return std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
}

// Callers must pass a non-blocking fd so draining stops at EAGAIN/EWOULDBLOCK.
void drain_fd(int fd) {
  if (fd < 0) {
    return;
  }

  std::uint8_t buffer[64];
  while (::read(fd, buffer, sizeof(buffer)) > 0) {
  }
}

template <typename CallbackMap, typename Sample>
void invoke_callbacks(const CallbackMap& callbacks, const Sample& sample) {
  for (const auto& [id, cb] : callbacks) {
    (void)id;
    if (cb) {
      cb(sample);
    }
  }
}

} // namespace

VescClient::SubscriptionHandle::SubscriptionHandle(std::function<void()> unsubscribe)
    : unsubscribe_(std::move(unsubscribe)) {}

VescClient::SubscriptionHandle::SubscriptionHandle(SubscriptionHandle&& other) noexcept
    : unsubscribe_(std::move(other.unsubscribe_)) {}

VescClient::SubscriptionHandle&
VescClient::SubscriptionHandle::operator=(SubscriptionHandle&& other) noexcept {
  if (this != &other) {
    reset();
    unsubscribe_ = std::move(other.unsubscribe_);
  }
  return *this;
}

VescClient::SubscriptionHandle::~SubscriptionHandle() {
  reset();
}

void VescClient::SubscriptionHandle::reset() {
  if (unsubscribe_) {
    unsubscribe_();
    unsubscribe_ = nullptr;
  }
}

std::vector<std::string> VescClient::find_devices() {
  ::glob_t g{};
  std::vector<std::string> result;
  if (::glob("/dev/ttyACM*", 0, nullptr, &g) == 0) {
    for (std::size_t i = 0; i < g.gl_pathc; ++i) {
      result.emplace_back(g.gl_pathv[i]);
    }
  }
  ::globfree(&g);
  return result;
}

VescClient::VescClient(VescConfig config)
    : config_(std::move(config)), callback_registry_(std::make_shared<CallbackRegistry>()) {
  imu_channel_.interval_ms.store(config_.imu_poll_interval.count());
  motor_channel_.interval_ms.store(config_.motor_poll_interval.count());
}

VescClient::~VescClient() {
  disconnect();
}

bool VescClient::connect() {
  if (running_.load()) {
    return true;
  }

  cleanup_transport_state();

  if (config_.open_serial_fn) {
    if (!config_.open_serial_fn(config_, fd_)) {
      return false;
    }
  } else {
    if (config_.device_path.empty()) {
      const auto devices = find_devices();
      if (devices.empty()) {
        return false;
      }
      config_.device_path = devices.front();
    }

    if (!open_serial(config_.device_path, config_.baud, fd_)) {
      return false;
    }
  }

  if (::pipe(wake_pipe_) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (!set_nonblocking(fd_) || !set_nonblocking(wake_pipe_[0]) || !set_nonblocking(wake_pipe_[1])) {
    ::close(fd_);
    fd_ = -1;
    ::close(wake_pipe_[0]);
    wake_pipe_[0] = -1;
    ::close(wake_pipe_[1]);
    wake_pipe_[1] = -1;
    return false;
  }

  parser_.reset();
  {
    std::lock_guard lock(cache_mutex_);
    imu_data_cache_.reset();
    motor_state_cache_.reset();
  }
  clear_pending_work();

  const auto now = SteadyClock::now();
  imu_channel_.next_due = now;
  motor_channel_.next_due = now;
  imu_channel_.last_sample = SteadyClock::time_point{};
  motor_channel_.last_sample = SteadyClock::time_point{};

  running_.store(true);
  io_thread_ = std::thread(&VescClient::io_loop, this);
  return true;
}

void VescClient::disconnect() {
  {
    std::lock_guard lock(scheduler_mutex_);
    running_.store(false);
  }

  wake_io_thread();
  cleanup_transport_state();
}

void VescClient::cleanup_transport_state() {
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  clear_pending_work();

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (wake_pipe_[0] >= 0) {
    ::close(wake_pipe_[0]);
    wake_pipe_[0] = -1;
  }
  if (wake_pipe_[1] >= 0) {
    ::close(wake_pipe_[1]);
    wake_pipe_[1] = -1;
  }
}

bool VescClient::is_connected() const {
  return running_.load();
}

void VescClient::set_motor_poll_interval(std::chrono::milliseconds interval) {
  motor_channel_.interval_ms.store(interval.count());
  wake_io_thread();
}

void VescClient::set_imu_poll_interval(std::chrono::milliseconds interval) {
  imu_channel_.interval_ms.store(interval.count());
  wake_io_thread();
}

VescClientConfigSnapshot VescClient::config_snapshot() const {
  VescClientConfigSnapshot snapshot;
  snapshot.motor_poll_interval = std::chrono::milliseconds(motor_channel_.interval_ms.load());
  snapshot.imu_poll_interval = std::chrono::milliseconds(imu_channel_.interval_ms.load());
  snapshot.poll_response_timeout = config_.poll_response_timeout;
  snapshot.query_guard_window = config_.query_guard_window;
  snapshot.command_watchdog_timeout = config_.command_watchdog_timeout;
  snapshot.command_watchdog_action = config_.command_watchdog_action;
  snapshot.max_brake_current = config_.max_brake_current;
  snapshot.command_watchdog_brake_current = config_.command_watchdog_brake_current;
  return snapshot;
}

std::optional<VescMotorState> VescClient::latest_motor_state() const {
  std::lock_guard lock(cache_mutex_);
  return motor_state_cache_;
}

std::optional<VescIMUData> VescClient::latest_imu() const {
  std::lock_guard lock(cache_mutex_);
  return imu_data_cache_;
}

VescClient::SubscriptionHandle VescClient::subscribe_imu(ImuCallback callback) {
  const auto callbacks = callback_registry_;
  std::lock_guard callbacks_lock(callbacks->mutex);
  const std::size_t id = callbacks->next_subscription_id++;
  callbacks->imu_callbacks.emplace(id, std::move(callback));
  return SubscriptionHandle([callbacks, id] {
    std::lock_guard unsubscribe_lock(callbacks->mutex);
    callbacks->imu_callbacks.erase(id);
  });
}

VescClient::SubscriptionHandle VescClient::subscribe_motor_state(MotorStateCallback callback) {
  const auto callbacks = callback_registry_;
  std::lock_guard callbacks_lock(callbacks->mutex);
  const std::size_t id = callbacks->next_subscription_id++;
  callbacks->motor_callbacks.emplace(id, std::move(callback));
  return SubscriptionHandle([callbacks, id] {
    std::lock_guard unsubscribe_lock(callbacks->mutex);
    callbacks->motor_callbacks.erase(id);
  });
}

bool VescClient::set_rpm(std::int32_t rpm) {
  return enqueue_control_command(VescProtocol::build_set_rpm_command(rpm));
}

bool VescClient::set_duty(float duty) {
  return enqueue_control_command(VescProtocol::build_set_duty_command(duty));
}

bool VescClient::set_current(float amps) {
  return enqueue_control_command(VescProtocol::build_set_current_command(amps));
}

bool VescClient::set_current_brake(float amps) {
  const float clamped_amps = clamp_brake_current(amps, config_.max_brake_current);
  if (clamped_amps <= 0.0f) {
    return false;
  }

  return enqueue_control_command(VescProtocol::build_set_current_brake_command(clamped_amps));
}

bool VescClient::set_servo_pos(float position) {
  return enqueue_control_command(VescProtocol::build_set_servo_pos_command(position));
}

std::optional<FwVersion> VescClient::request_fw_version(std::chrono::milliseconds timeout) {
  if (!running_.load() || timeout <= std::chrono::milliseconds::zero()) {
    return std::nullopt;
  }

  auto state = std::make_shared<QueryState>();
  const auto deadline = SteadyClock::now() + timeout;

  ScheduledRequest request;
  request.kind = ScheduledRequest::Kind::FwVersion;
  request.expected_id = static_cast<std::uint8_t>(VescPacketCommID::FwVersion);
  request.packet = VescProtocol::build_fw_version_request();
  request.deadline = deadline;
  request.on_success = [state](const Payload& payload) {
    const auto parsed = VescProtocol::parse_fw_version(payload);
    std::lock_guard lock(state->mutex);
    state->result = parsed;
    state->completed = true;
    state->cv.notify_all();
  };
  request.on_timeout = [state] {
    std::lock_guard lock(state->mutex);
    state->result = std::nullopt;
    state->completed = true;
    state->cv.notify_all();
  };

  schedule_query(std::move(request));

  std::unique_lock lock(state->mutex);
  state->cv.wait(lock, [&state] { return state->completed; });
  return state->result;
}

std::uint64_t VescClient::default_wall_time_ns() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::uint64_t VescClient::wall_time_ns() const {
  if (config_.wall_time_ns) {
    return config_.wall_time_ns();
  }
  return default_wall_time_ns();
}

void VescClient::io_loop() {
  const int nfds = std::max(fd_, wake_pipe_[0]) + 1;

  while (running_.load()) {
    handle_request_timeout();

    const auto now = SteadyClock::now();
    std::optional<std::chrono::microseconds> wait_time = std::chrono::milliseconds(50);

    auto tighten_wait = [&](const std::optional<std::chrono::microseconds>& candidate) {
      if (!candidate) {
        return;
      }
      if (!wait_time || *candidate < *wait_time) {
        wait_time = candidate;
      }
    };

    {
      std::lock_guard lock(scheduler_mutex_);
      if (in_flight_request_) {
        tighten_wait(micros_until(in_flight_request_->deadline, now));
      }
      if (control_watchdog_.armed) {
        tighten_wait(micros_until(control_watchdog_.deadline, now));
      }
      const auto next_query =
          std::min_element(request_queue_.begin(), request_queue_.end(),
                           [](const ScheduledRequest& lhs, const ScheduledRequest& rhs) {
                             return lhs.deadline < rhs.deadline;
                           });
      if (next_query != request_queue_.end()) {
        tighten_wait(micros_until(next_query->deadline, now));
      }
    }

    for (PollChannel* channel : {&imu_channel_, &motor_channel_}) {
      const auto interval_ms = channel->interval_ms.load();
      if (interval_ms <= 0) {
        continue;
      }
      tighten_wait(micros_until(channel->next_due, now));
    }

    struct timeval tv {};
    struct timeval* tv_ptr = nullptr;
    if (wait_time) {
      const auto raw = wait_time->count();
      const auto bounded = raw > 0 ? raw : 0;
      tv.tv_sec = static_cast<decltype(tv.tv_sec)>(bounded / 1000000);
      tv.tv_usec = static_cast<decltype(tv.tv_usec)>(bounded % 1000000);
      tv_ptr = &tv;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    FD_SET(wake_pipe_[0], &rfds);

    const int rc = ::select(nfds, &rfds, nullptr, nullptr, tv_ptr);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      running_.store(false);
      break;
    }

    if (FD_ISSET(wake_pipe_[0], &rfds)) {
      drain_fd(wake_pipe_[0]);
    }

    if (FD_ISSET(fd_, &rfds)) {
      std::uint8_t buffer[kMaxFramedPacketBytes];
      for (;;) {
        const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n > 0) {
          for (ssize_t i = 0; i < n; ++i) {
            if (auto payload = parser_.feed_byte(buffer[static_cast<std::size_t>(i)])) {
              dispatch_payload(*payload, wall_time_ns());
            }
          }
          continue;
        }
        if (n < 0 && errno == EINTR) {
          continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          break;
        }
        if (n < 0) {
          running_.store(false);
        }
        break;
      }
    }

    std::deque<std::vector<std::uint8_t>> commands;
    {
      std::lock_guard lock(scheduler_mutex_);
      commands.swap(command_queue_);
    }
    // Drain queued control commands before issuing any new reply-bearing work so
    // fire-and-forget actuation stays highest priority on the wire.
    const bool wrote_all_commands = std::all_of(
        commands.begin(), commands.end(),
        [&](const std::vector<std::uint8_t>& command) { return write_packet(command); });
    if (!wrote_all_commands) {
      running_.store(false);
      break;
    }

    handle_request_timeout();

    if (auto watchdog_command = dequeue_due_watchdog_command(SteadyClock::now())) {
      if (!write_packet(*watchdog_command)) {
        running_.store(false);
        break;
      }
    }

    bool has_in_flight = false;
    {
      std::lock_guard lock(scheduler_mutex_);
      has_in_flight = in_flight_request_.has_value();
    }
    if (has_in_flight) {
      continue;
    }

    const auto after_io = SteadyClock::now();

    if (PollChannel* poll_channel = select_due_poll_channel(after_io)) {
      auto poll = make_due_poll_request(*poll_channel, after_io);
      if (!poll || !write_packet(poll->packet)) {
        running_.store(false);
        break;
      }
      std::lock_guard lock(scheduler_mutex_);
      in_flight_request_ = std::move(*poll);
      continue;
    }

    if (auto query = dequeue_ready_query(after_io)) {
      if (!write_packet(query->packet)) {
        running_.store(false);
        break;
      }
      std::lock_guard lock(scheduler_mutex_);
      in_flight_request_ = std::move(*query);
    }
  }

  clear_pending_work();
}

void VescClient::dispatch_payload(const Payload& payload, std::uint64_t stamp_ns) {
  if (payload.empty()) {
    return;
  }

  std::optional<ScheduledRequest> completed_request;
  bool drop_stale_query_reply = false;
  {
    std::lock_guard lock(scheduler_mutex_);
    // Timed-out blocking queries may still produce a late reply on the wire.
    // Count and drop those replies so they cannot satisfy a newer query with
    // the same expected packet ID.
    auto stale_reply = stale_query_reply_counts_.find(payload[0]);
    if (stale_reply != stale_query_reply_counts_.end()) {
      if (stale_reply->second > 1) {
        --stale_reply->second;
      } else {
        stale_query_reply_counts_.erase(stale_reply);
      }
      drop_stale_query_reply = true;
    } else if (in_flight_request_ && in_flight_request_->expected_id == payload[0]) {
      completed_request = std::move(in_flight_request_);
      in_flight_request_.reset();
    }
  }

  if (drop_stale_query_reply) {
    return;
  }

  if (completed_request) {
    if (completed_request->kind == ScheduledRequest::Kind::PollImu) {
      if (auto data = VescProtocol::parse_get_imu_data(payload)) {
        data->stamp_ns = stamp_ns;
        imu_channel_.last_sample = SteadyClock::now();
        {
          std::lock_guard lock(cache_mutex_);
          imu_data_cache_ = *data;
        }
        publish_imu(*data);
      }
      return;
    }

    if (completed_request->kind == ScheduledRequest::Kind::PollMotorState) {
      if (auto state = VescProtocol::parse_get_values(payload)) {
        state->stamp_ns = stamp_ns;
        motor_channel_.last_sample = SteadyClock::now();
        {
          std::lock_guard lock(cache_mutex_);
          motor_state_cache_ = *state;
        }
        publish_motor_state(*state);
      }
      return;
    }

    if (completed_request->on_success) {
      completed_request->on_success(payload);
    }
  }
}

void VescClient::handle_request_timeout() {
  std::optional<ScheduledRequest> timed_out;
  std::vector<ScheduledRequest> expired_queries;
  {
    std::lock_guard lock(scheduler_mutex_);
    const auto now = SteadyClock::now();
    if (in_flight_request_ && in_flight_request_->deadline <= now) {
      timed_out = std::move(in_flight_request_);
      if (timed_out->kind == ScheduledRequest::Kind::FwVersion) {
        // Only a request that was actually sent can still yield a stale late reply.
        ++stale_query_reply_counts_[timed_out->expected_id];
      }
      in_flight_request_.reset();
    }

    for (auto it = request_queue_.begin(); it != request_queue_.end();) {
      if (it->deadline <= now) {
        expired_queries.push_back(std::move(*it));
        it = request_queue_.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (timed_out && timed_out->on_timeout) {
    timed_out->on_timeout();
  }
  for (auto& expired : expired_queries) {
    if (expired.on_timeout) {
      expired.on_timeout();
    }
  }
}

void VescClient::schedule_query(ScheduledRequest request) {
  {
    std::lock_guard lock(scheduler_mutex_);
    request_queue_.push_back(std::move(request));
  }
  wake_io_thread();
}

bool VescClient::enqueue_control_command(std::vector<std::uint8_t> packet) {
  {
    std::lock_guard lock(scheduler_mutex_);
    if (!running_.load()) {
      return false;
    }
    if (control_watchdog_enabled()) {
      control_watchdog_.deadline = SteadyClock::now() + config_.command_watchdog_timeout;
      control_watchdog_.armed = true;
    }
    command_queue_.push_back(std::move(packet));
  }
  wake_io_thread();
  return true;
}

bool VescClient::control_watchdog_enabled() const {
  if (config_.command_watchdog_timeout <= std::chrono::milliseconds::zero()) {
    return false;
  }

  switch (config_.command_watchdog_action) {
  case ControlWatchdogAction::Disabled:
    return false;
  case ControlWatchdogAction::Coast:
    return true;
  case ControlWatchdogAction::BrakeCurrent:
    return config_.command_watchdog_brake_current > 0.0f && config_.max_brake_current > 0.0f;
  }

  return false;
}

std::optional<std::vector<std::uint8_t>>
VescClient::dequeue_due_watchdog_command(const SteadyClock::time_point& now) {
  if (!running_.load() || !control_watchdog_enabled()) {
    return std::nullopt;
  }

  {
    std::lock_guard lock(scheduler_mutex_);
    if (!control_watchdog_.armed || control_watchdog_.deadline > now) {
      return std::nullopt;
    }
    control_watchdog_ = ControlWatchdogState{};
  }

  if (config_.command_watchdog_action == ControlWatchdogAction::Coast) {
    return VescProtocol::build_set_current_command(0.0f);
  }
  if (config_.command_watchdog_action == ControlWatchdogAction::BrakeCurrent) {
    return VescProtocol::build_set_current_brake_command(
        clamp_brake_current(config_.command_watchdog_brake_current, config_.max_brake_current));
  }
  return std::nullopt;
}

void VescClient::wake_io_thread() const {
  if (wake_pipe_[1] < 0) {
    return;
  }
  const std::uint8_t byte = 1;
  (void)::write(wake_pipe_[1], &byte, 1);
}

bool VescClient::write_packet(const std::vector<std::uint8_t>& pkt) {
  const std::uint8_t* cursor = pkt.data();
  std::size_t remaining = pkt.size();

  while (remaining > 0 && running_.load()) {
    const ssize_t n = ::write(fd_, cursor, remaining);
    if (n > 0) {
      cursor += n;
      remaining -= static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // Writes stay on the I/O thread, but a full kernel buffer should still
      // react promptly to disconnects or scheduler wakeups.
      if (!wait_until_writable()) {
        return false;
      }
      continue;
    }
    return false;
  }

  return remaining == 0;
}

bool VescClient::wait_until_writable() {
  while (running_.load()) {
    fd_set rfds;
    fd_set wfds;
    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_SET(fd_, &wfds);
    FD_SET(wake_pipe_[0], &rfds);

    struct timeval timeout {};
    timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(kWriteWaitPollInterval.count() / 1000);
    timeout.tv_usec =
        static_cast<decltype(timeout.tv_usec)>((kWriteWaitPollInterval.count() % 1000) * 1000);

    const int nfds = std::max(fd_, wake_pipe_[0]) + 1;
    const int rc = ::select(nfds, &rfds, &wfds, nullptr, &timeout);
    if (rc > 0) {
      const bool wake_ready = FD_ISSET(wake_pipe_[0], &rfds);
      const bool fd_writable = FD_ISSET(fd_, &wfds);

      if (wake_ready) {
        drain_fd(wake_pipe_[0]);
        if (!running_.load()) {
          return false;
        }
      }
      if (fd_writable) {
        return true;
      }

      // A wake-only event means another thread changed scheduler or shutdown
      // state while this thread was blocked waiting for write readiness.
      continue;
    }
    if (rc == 0) {
      continue;
    }
    if (errno == EINTR) {
      continue;
    }
    return false;
  }
  return false;
}

void VescClient::publish_imu(const VescIMUData& data) {
  const auto registry = callback_registry_;
  std::unordered_map<std::size_t, ImuCallback> callbacks;
  {
    std::lock_guard lock(registry->mutex);
    callbacks = registry->imu_callbacks;
  }
  invoke_callbacks(callbacks, data);
}

void VescClient::publish_motor_state(const VescMotorState& state) {
  const auto registry = callback_registry_;
  std::unordered_map<std::size_t, MotorStateCallback> callbacks;
  {
    std::lock_guard lock(registry->mutex);
    callbacks = registry->motor_callbacks;
  }
  invoke_callbacks(callbacks, state);
}

void VescClient::clear_pending_work() {
  std::deque<ScheduledRequest> queued_requests;
  std::optional<ScheduledRequest> in_flight;
  {
    std::lock_guard lock(scheduler_mutex_);
    command_queue_.clear();
    queued_requests.swap(request_queue_);
    in_flight = std::move(in_flight_request_);
    in_flight_request_.reset();
    stale_query_reply_counts_.clear();
    control_watchdog_ = ControlWatchdogState{};
  }

  for (auto& request : queued_requests) {
    if (request.on_timeout) {
      request.on_timeout();
    }
  }
  if (in_flight && in_flight->on_timeout) {
    in_flight->on_timeout();
  }
}

VescClient::PollChannel* VescClient::select_due_poll_channel(const SteadyClock::time_point& now) {
  PollChannel* selected = nullptr;
  SteadyClock::time_point selected_due{};

  for (PollChannel* channel : {&imu_channel_, &motor_channel_}) {
    const auto interval_ms = channel->interval_ms.load();
    if (interval_ms <= 0) {
      continue;
    }

    const auto due = channel->next_due.time_since_epoch().count() == 0 ? now : channel->next_due;
    if (due > now) {
      continue;
    }

    if (!selected || due < selected_due ||
        (due == selected_due && channel->kind == PollChannel::Kind::Imu)) {
      selected = channel;
      selected_due = due;
    }
  }

  return selected;
}

std::optional<VescClient::ScheduledRequest>
VescClient::make_due_poll_request(PollChannel& channel, const SteadyClock::time_point& now) {
  const auto interval_ms = channel.interval_ms.load();
  if (interval_ms <= 0) {
    return std::nullopt;
  }

  const auto interval = std::chrono::milliseconds(interval_ms);
  if (channel.next_due.time_since_epoch().count() == 0) {
    channel.next_due = now;
  }
  if (channel.next_due > now) {
    return std::nullopt;
  }

  while (channel.next_due <= now) {
    channel.next_due += interval;
  }

  ScheduledRequest request;
  request.expected_id = static_cast<std::uint8_t>(channel.kind == PollChannel::Kind::Imu
                                                      ? VescPacketCommID::GetImuData
                                                      : VescPacketCommID::GetValues);
  request.deadline = now + config_.poll_response_timeout;

  if (channel.kind == PollChannel::Kind::Imu) {
    request.kind = ScheduledRequest::Kind::PollImu;
    request.packet = VescProtocol::build_get_imu_data_request();
  } else {
    request.kind = ScheduledRequest::Kind::PollMotorState;
    request.packet = VescProtocol::build_get_values_request();
  }
  return request;
}

std::optional<VescClient::ScheduledRequest>
VescClient::dequeue_ready_query(const SteadyClock::time_point& now) {
  std::vector<ScheduledRequest> expired_queries;
  std::optional<ScheduledRequest> ready_query;
  {
    std::lock_guard lock(scheduler_mutex_);
    for (auto it = request_queue_.begin(); it != request_queue_.end();) {
      if (it->deadline <= now) {
        expired_queries.push_back(std::move(*it));
        it = request_queue_.erase(it);
      } else {
        ++it;
      }
    }
    if (!request_queue_.empty()) {
      // connect() seeds next_due before the I/O loop runs, so treating a zero epoch as
      // "due now" here is only a fallback for partially initialized test setups.
      const auto imu_next_due =
          imu_channel_.next_due.time_since_epoch().count() == 0 ? now : imu_channel_.next_due;
      const auto motor_next_due =
          motor_channel_.next_due.time_since_epoch().count() == 0 ? now : motor_channel_.next_due;
      const auto time_until_imu =
          imu_channel_.interval_ms.load() > 0 ? imu_next_due - now : SteadyClock::duration::max();
      const auto time_until_motor = motor_channel_.interval_ms.load() > 0
                                        ? motor_next_due - now
                                        : SteadyClock::duration::max();
      const auto next_poll_due = std::min(time_until_imu, time_until_motor);

      if (next_poll_due > config_.query_guard_window) {
        for (auto it = request_queue_.begin(); it != request_queue_.end(); ++it) {
          if (stale_query_reply_counts_.count(it->expected_id) != 0U) {
            // If a stale reply never arrives, the queued query still completes by expiry.
            continue;
          }
          ready_query = std::move(*it);
          request_queue_.erase(it);
          break;
        }
      }
    }
  }

  for (auto& expired : expired_queries) {
    if (expired.on_timeout) {
      expired.on_timeout();
    }
  }

  return ready_query;
}

} // namespace goat_vesc
