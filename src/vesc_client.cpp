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

std::optional<std::chrono::microseconds> micros_until(const SteadyClock::time_point& deadline,
                                                      const SteadyClock::time_point& now) {
  if (deadline <= now) {
    return std::chrono::microseconds::zero();
  }
  return std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
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

VescClient::VescClient(VescConfig config) : config_(std::move(config)) {
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

  const int fd_flags = ::fcntl(fd_, F_GETFL, 0);
  if (fd_flags >= 0) {
    (void)::fcntl(fd_, F_SETFL, fd_flags | O_NONBLOCK);
  }
  (void)::fcntl(wake_pipe_[0], F_SETFL, O_NONBLOCK);
  (void)::fcntl(wake_pipe_[1], F_SETFL, O_NONBLOCK);

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
    if (!running_.load()) {
      return;
    }
    running_.store(false);
  }

  wake_io_thread();

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

std::optional<VescMotorState> VescClient::latest_motor_state() const {
  std::lock_guard lock(cache_mutex_);
  return motor_state_cache_;
}

std::optional<VescIMUData> VescClient::latest_imu() const {
  std::lock_guard lock(cache_mutex_);
  return imu_data_cache_;
}

VescClient::SubscriptionHandle VescClient::subscribe_imu(ImuCallback callback) {
  std::lock_guard lock(callback_mutex_);
  const std::size_t id = next_subscription_id_++;
  imu_callbacks_.emplace(id, std::move(callback));
  return SubscriptionHandle([this, id] { unsubscribe_imu(id); });
}

VescClient::SubscriptionHandle VescClient::subscribe_motor_state(MotorStateCallback callback) {
  std::lock_guard lock(callback_mutex_);
  const std::size_t id = next_subscription_id_++;
  motor_callbacks_.emplace(id, std::move(callback));
  return SubscriptionHandle([this, id] { unsubscribe_motor_state(id); });
}

bool VescClient::set_rpm(std::int32_t rpm) {
  std::vector<std::uint8_t> packet;
  {
    std::lock_guard lock(protocol_mutex_);
    packet = cmd_protocol_.build_set_rpm_command(rpm);
  }
  return enqueue_command(std::move(packet));
}

bool VescClient::set_duty(float duty) {
  std::vector<std::uint8_t> packet;
  {
    std::lock_guard lock(protocol_mutex_);
    packet = cmd_protocol_.build_set_duty_command(duty);
  }
  return enqueue_command(std::move(packet));
}

bool VescClient::set_current(float amps) {
  std::vector<std::uint8_t> packet;
  {
    std::lock_guard lock(protocol_mutex_);
    packet = cmd_protocol_.build_set_current_command(amps);
  }
  return enqueue_command(std::move(packet));
}

bool VescClient::set_current_brake(float amps) {
  std::vector<std::uint8_t> packet;
  {
    std::lock_guard lock(protocol_mutex_);
    packet = cmd_protocol_.build_set_current_brake_command(amps);
  }
  return enqueue_command(std::move(packet));
}

bool VescClient::set_servo_pos(float position) {
  std::vector<std::uint8_t> packet;
  {
    std::lock_guard lock(protocol_mutex_);
    packet = cmd_protocol_.build_set_servo_pos_command(position);
  }
  return enqueue_command(std::move(packet));
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
  {
    std::lock_guard lock(protocol_mutex_);
    request.packet = cmd_protocol_.build_fw_version_request();
  }
  request.deadline = deadline;
  request.on_success = [this, state](const Payload& payload) {
    const auto parsed = io_protocol_.parse_fw_version(payload);
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
      if (!request_queue_.empty()) {
        tighten_wait(micros_until(request_queue_.front().deadline, now));
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
      std::uint8_t buffer[64];
      while (::read(wake_pipe_[0], buffer, sizeof(buffer)) > 0) {
      }
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
    for (const auto& command : commands) {
      if (!write_packet(command)) {
        running_.store(false);
        break;
      }
    }
    if (!running_.load()) {
      break;
    }

    handle_request_timeout();

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
      if (auto data = io_protocol_.parse_get_imu_data(payload)) {
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
      if (auto state = io_protocol_.parse_get_values(payload)) {
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

bool VescClient::enqueue_command(std::vector<std::uint8_t> packet) {
  {
    std::lock_guard lock(scheduler_mutex_);
    if (!running_.load()) {
      return false;
    }
    command_queue_.push_back(std::move(packet));
  }
  wake_io_thread();
  return true;
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
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd_, &wfds);

    const int rc = ::select(fd_ + 1, nullptr, &wfds, nullptr, nullptr);
    if (rc > 0) {
      return true;
    }
    if (rc < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return false;
}

void VescClient::publish_imu(const VescIMUData& data) {
  std::unordered_map<std::size_t, ImuCallback> callbacks;
  {
    std::lock_guard lock(callback_mutex_);
    callbacks = imu_callbacks_;
  }
  invoke_callbacks(callbacks, data);
}

void VescClient::publish_motor_state(const VescMotorState& state) {
  std::unordered_map<std::size_t, MotorStateCallback> callbacks;
  {
    std::lock_guard lock(callback_mutex_);
    callbacks = motor_callbacks_;
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

void VescClient::unsubscribe_imu(std::size_t id) {
  std::lock_guard lock(callback_mutex_);
  imu_callbacks_.erase(id);
}

void VescClient::unsubscribe_motor_state(std::size_t id) {
  std::lock_guard lock(callback_mutex_);
  motor_callbacks_.erase(id);
}

VescClient::PollChannel* VescClient::select_due_poll_channel(const SteadyClock::time_point& now) {
  PollChannel* selected = nullptr;
  SteadyClock::time_point selected_due{};

  for (PollChannel* channel : {&imu_channel_, &motor_channel_}) {
    const auto interval_ms = channel->interval_ms.load();
    if (interval_ms <= 0) {
      continue;
    }

    const auto due =
        channel->next_due.time_since_epoch().count() == 0 ? now : channel->next_due;
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
    request.packet = io_protocol_.build_get_imu_data_request();
  } else {
    request.kind = ScheduledRequest::Kind::PollMotorState;
    request.packet = io_protocol_.build_get_values_request();
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
      const auto imu_next_due = imu_channel_.next_due.time_since_epoch().count() == 0 ? now
                                                                                       : imu_channel_.next_due;
      const auto motor_next_due = motor_channel_.next_due.time_since_epoch().count() == 0
                                      ? now
                                      : motor_channel_.next_due;
      const auto time_until_imu = imu_channel_.interval_ms.load() > 0
                                      ? imu_next_due - now
                                      : SteadyClock::duration::max();
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
