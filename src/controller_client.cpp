#include "goat_motor_controller/controller_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include <fcntl.h>
#include <glob.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace goat_motor_controller {

namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr auto kWriteWaitPollInterval = std::chrono::milliseconds(50);
constexpr std::size_t kConfigSignatureBytes = 4;
constexpr std::uint32_t kInitialLispReadBytes = 10;
constexpr std::uint32_t kLispReadChunkBytes = 400;
constexpr std::size_t kLispWriteChunkBytes = 384;
constexpr std::uint32_t kEmptyLispEraseBytes = 16;
constexpr std::uint32_t kLispEraseMarginBytes = 100;
thread_local ControllerClient* active_io_client = nullptr;

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

  if (::cfsetispeed(&tty, static_cast<speed_t>(baud_const)) != 0 ||
      ::cfsetospeed(&tty, static_cast<speed_t>(baud_const)) != 0) {
    ::close(fd);
    return false;
  }
  ::cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 1;
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
  if (!std::isfinite(requested_amps) || !std::isfinite(max_brake_current) ||
      requested_amps <= 0.0f || max_brake_current <= 0.0f) {
    return 0.0f;
  }
  return std::min(requested_amps, max_brake_current);
}

std::chrono::microseconds micros_until(const SteadyClock::time_point& deadline,
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
      try {
        cb(sample);
      } catch (...) {
      }
    }
  }
}

bool same_config_signature(const std::vector<std::uint8_t>& lhs,
                           const std::vector<std::uint8_t>& rhs) {
  return lhs.size() >= kConfigSignatureBytes && rhs.size() >= kConfigSignatureBytes &&
         std::equal(lhs.begin(), lhs.begin() + kConfigSignatureBytes, rhs.begin());
}

} // namespace

ControllerClient::SubscriptionHandle::SubscriptionHandle(std::function<void()> unsubscribe)
    : unsubscribe_(std::move(unsubscribe)) {}

ControllerClient::SubscriptionHandle::SubscriptionHandle(SubscriptionHandle&& other) noexcept
    : unsubscribe_(std::move(other.unsubscribe_)) {}

ControllerClient::SubscriptionHandle&
ControllerClient::SubscriptionHandle::operator=(SubscriptionHandle&& other) noexcept {
  if (this != &other) {
    reset();
    unsubscribe_ = std::move(other.unsubscribe_);
  }
  return *this;
}

ControllerClient::SubscriptionHandle::~SubscriptionHandle() {
  reset();
}

void ControllerClient::SubscriptionHandle::reset() {
  if (unsubscribe_) {
    unsubscribe_();
    unsubscribe_ = nullptr;
  }
}

std::vector<std::string> ControllerClient::find_devices() {
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

ControllerClient::ControllerClient(ControllerConfig config)
    : config_(std::move(config)), callback_registry_(std::make_shared<CallbackRegistry>()) {
  imu_channel_.interval_ms.store(std::max<std::int64_t>(config_.imu_poll_interval.count(), 0));
  motor_channel_.interval_ms.store(std::max<std::int64_t>(config_.motor_poll_interval.count(), 0));
}

ControllerClient::~ControllerClient() {
  disconnect();
}

bool ControllerClient::connect() {
  if (active_io_client == this) {
    return running_.load();
  }

  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  if (running_.load()) {
    return true;
  }

  cleanup_transport_state();

  if (config_.open_serial_fn) {
    bool opened = false;
    try {
      opened = config_.open_serial_fn(config_, fd_);
    } catch (...) {
    }
    if (!opened) {
      if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
      }
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

  running_.store(true);
  try {
    io_thread_ = std::thread(&ControllerClient::io_loop, this);
  } catch (...) {
    running_.store(false);
    cleanup_transport_state();
    return false;
  }
  return true;
}

void ControllerClient::disconnect() {
  if (active_io_client == this) {
    std::lock_guard lock(scheduler_mutex_);
    running_.store(false);
    return;
  }

  std::lock_guard lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard lock(scheduler_mutex_);
    running_.store(false);
    wake_io_thread();
  }

  cleanup_transport_state();
}

void ControllerClient::cleanup_transport_state() {
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

bool ControllerClient::is_connected() const {
  return running_.load();
}

void ControllerClient::set_motor_poll_interval(std::chrono::milliseconds interval) {
  std::lock_guard lock(scheduler_mutex_);
  motor_channel_.interval_ms.store(std::max<std::int64_t>(interval.count(), 0));
  motor_channel_.reschedule.store(true);
  if (running_.load()) {
    wake_io_thread();
  }
}

void ControllerClient::set_imu_poll_interval(std::chrono::milliseconds interval) {
  std::lock_guard lock(scheduler_mutex_);
  imu_channel_.interval_ms.store(std::max<std::int64_t>(interval.count(), 0));
  imu_channel_.reschedule.store(true);
  if (running_.load()) {
    wake_io_thread();
  }
}

ControllerConfigSnapshot ControllerClient::config_snapshot() const {
  ControllerConfigSnapshot snapshot;
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

std::optional<MotorState> ControllerClient::latest_motor_state() const {
  std::lock_guard lock(cache_mutex_);
  return motor_state_cache_;
}

std::optional<ImuData> ControllerClient::latest_imu() const {
  std::lock_guard lock(cache_mutex_);
  return imu_data_cache_;
}

ControllerClient::SubscriptionHandle ControllerClient::subscribe_imu(ImuCallback callback) {
  const auto callbacks = callback_registry_;
  std::lock_guard callbacks_lock(callbacks->mutex);
  const std::size_t id = callbacks->next_subscription_id++;
  callbacks->imu_callbacks.emplace(id, std::move(callback));
  return SubscriptionHandle([callbacks, id] {
    std::lock_guard unsubscribe_lock(callbacks->mutex);
    callbacks->imu_callbacks.erase(id);
  });
}

ControllerClient::SubscriptionHandle ControllerClient::subscribe_motor_state(MotorStateCallback callback) {
  const auto callbacks = callback_registry_;
  std::lock_guard callbacks_lock(callbacks->mutex);
  const std::size_t id = callbacks->next_subscription_id++;
  callbacks->motor_callbacks.emplace(id, std::move(callback));
  return SubscriptionHandle([callbacks, id] {
    std::lock_guard unsubscribe_lock(callbacks->mutex);
    callbacks->motor_callbacks.erase(id);
  });
}

bool ControllerClient::set_rpm(std::int32_t rpm) {
  return enqueue_control_command(ControllerProtocol::build_set_rpm_command(rpm));
}

bool ControllerClient::set_duty(float duty) {
  return enqueue_control_command(ControllerProtocol::build_set_duty_command(duty));
}

bool ControllerClient::set_current(float amps) {
  return enqueue_control_command(ControllerProtocol::build_set_current_command(amps));
}

bool ControllerClient::set_current_brake(float amps) {
  const float clamped_amps = clamp_brake_current(amps, config_.max_brake_current);
  if (clamped_amps <= 0.0f) {
    return false;
  }

  return enqueue_control_command(ControllerProtocol::build_set_current_brake_command(clamped_amps));
}

bool ControllerClient::set_servo_pos(float position) {
  return enqueue_control_command(ControllerProtocol::build_set_servo_pos_command(position));
}

std::optional<FwVersion> ControllerClient::request_fw_version(std::chrono::milliseconds timeout) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    return std::nullopt;
  }

  const auto payload =
      request_payload(ControllerProtocol::build_fw_version_request(), CommandId::FwVersion,
                      SteadyClock::now() + timeout, false);
  return payload ? ControllerProtocol::parse_fw_version(*payload) : std::nullopt;
}

std::optional<ControllerClient::Payload>
ControllerClient::request_payload(std::vector<std::uint8_t> packet, CommandId expected_id,
                            SteadyClock::time_point deadline, bool stop_on_timeout) {
  if (active_io_client == this || packet.empty() || deadline <= SteadyClock::now()) {
    return std::nullopt;
  }

  auto state = std::make_shared<QueryState>();

  ScheduledRequest request;
  request.kind = ScheduledRequest::Kind::Query;
  request.expected_id = static_cast<std::uint8_t>(expected_id);
  request.packet = std::move(packet);
  request.deadline = deadline;
  request.stop_on_timeout = stop_on_timeout;
  request.on_success = [state](const Payload& payload) {
    std::lock_guard lock(state->mutex);
    state->result = payload;
    state->completed = true;
    state->cv.notify_all();
  };
  request.on_timeout = [state] {
    std::lock_guard lock(state->mutex);
    state->completed = true;
    state->cv.notify_all();
  };

  {
    std::lock_guard scheduler_lock(scheduler_mutex_);
    if (!running_.load()) {
      return std::nullopt;
    }
    request_queue_.push_back(std::move(request));
    wake_io_thread();
  }

  std::unique_lock lock(state->mutex);
  state->cv.wait(lock, [&state] { return state->completed; });
  return state->result;
}

std::optional<MotorConfigImage>
ControllerClient::request_motor_config_until(SteadyClock::time_point deadline) {
  const auto payload =
      request_payload(ControllerProtocol::build_get_motor_config_request(),
                      CommandId::GetMotorConfig, deadline, true);
  return payload ? ControllerProtocol::parse_motor_config(*payload) : std::nullopt;
}

std::optional<MotorConfigImage>
ControllerClient::request_motor_config(std::chrono::milliseconds timeout) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    return std::nullopt;
  }
  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (!lock.try_lock_until(deadline)) {
    return std::nullopt;
  }
  return request_motor_config_until(deadline);
}

OperationResult ControllerClient::write_motor_config(const MotorConfigImage& image,
                                                   std::chrono::milliseconds timeout) {
  const auto packet = ControllerProtocol::build_set_motor_config_request(image);
  if (packet.empty()) {
    return OperationResult::InvalidData;
  }

  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return OperationResult::NoReply;
  }

  const auto current = request_motor_config_until(deadline);
  if (!current) {
    return OperationResult::NoReply;
  }
  if (!same_config_signature(image.bytes, current->bytes)) {
    return OperationResult::IncompatibleData;
  }

  const auto reply =
      request_payload(packet, CommandId::SetMotorConfig, deadline, true);
  if (!reply) {
    return OperationResult::NoReply;
  }
  return ControllerProtocol::parse_config_ack(*reply, CommandId::SetMotorConfig)
             ? OperationResult::Success
             : OperationResult::Rejected;
}

std::optional<AppConfigImage>
ControllerClient::request_app_config_until(SteadyClock::time_point deadline) {
  const auto payload =
      request_payload(ControllerProtocol::build_get_app_config_request(), CommandId::GetAppConfig,
                      deadline, true);
  return payload ? ControllerProtocol::parse_app_config(*payload) : std::nullopt;
}

std::optional<AppConfigImage>
ControllerClient::request_app_config(std::chrono::milliseconds timeout) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    return std::nullopt;
  }
  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (!lock.try_lock_until(deadline)) {
    return std::nullopt;
  }
  return request_app_config_until(deadline);
}

OperationResult ControllerClient::write_app_config(const AppConfigImage& image,
                                                 AppConfigStorage storage,
                                                 std::chrono::milliseconds timeout) {
  const auto packet = ControllerProtocol::build_set_app_config_request(image, storage);
  if (packet.empty()) {
    return OperationResult::InvalidData;
  }

  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return OperationResult::NoReply;
  }

  const auto current = request_app_config_until(deadline);
  if (!current) {
    return OperationResult::NoReply;
  }
  if (!same_config_signature(image.bytes, current->bytes)) {
    return OperationResult::IncompatibleData;
  }

  const auto expected_id = storage == AppConfigStorage::Persistent
                               ? CommandId::SetAppConfig
                               : CommandId::SetAppConfigNoStore;
  const auto reply = request_payload(packet, expected_id, deadline, true);
  if (!reply) {
    return OperationResult::NoReply;
  }
  return ControllerProtocol::parse_config_ack(*reply, expected_id) ? OperationResult::Success
                                                             : OperationResult::Rejected;
}

std::optional<LispCodeImage> ControllerClient::request_lisp_code(std::chrono::milliseconds timeout) {
  if (timeout <= std::chrono::milliseconds::zero()) {
    return std::nullopt;
  }
  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (!lock.try_lock_until(deadline)) {
    return std::nullopt;
  }

  const auto first =
      request_payload(ControllerProtocol::build_lisp_read_request(kInitialLispReadBytes, 0),
                      CommandId::LispReadCode, deadline, true);
  if (!first) {
    return std::nullopt;
  }

  std::uint32_t total_size = 0;
  std::uint32_t offset = 0;
  auto chunk = ControllerProtocol::parse_lisp_read_reply(*first, total_size, offset);
  if (!chunk || offset != 0) {
    return std::nullopt;
  }
  if (total_size == 0) {
    return LispCodeImage{};
  }
  if (total_size < kInitialLispReadBytes || chunk->size() != kInitialLispReadBytes) {
    return std::nullopt;
  }

  LispCodeImage image;
  image.bytes.reserve(total_size);
  image.bytes.insert(image.bytes.end(), chunk->begin(), chunk->end());

  while (image.bytes.size() < total_size) {
    const auto remaining = total_size - static_cast<std::uint32_t>(image.bytes.size());
    const auto requested = std::min(kLispReadChunkBytes, remaining);
    const auto reply =
        request_payload(ControllerProtocol::build_lisp_read_request(
                            requested, static_cast<std::uint32_t>(image.bytes.size())),
                        CommandId::LispReadCode, deadline, true);
    if (!reply) {
      return std::nullopt;
    }

    std::uint32_t reply_total = 0;
    std::uint32_t reply_offset = 0;
    chunk = ControllerProtocol::parse_lisp_read_reply(*reply, reply_total, reply_offset);
    if (!chunk || reply_total != total_size || reply_offset != image.bytes.size() ||
        chunk->size() != requested) {
      return std::nullopt;
    }
    image.bytes.insert(image.bytes.end(), chunk->begin(), chunk->end());
  }

  return image;
}

OperationResult ControllerClient::erase_lisp_code_until(std::uint32_t size,
                                                      SteadyClock::time_point deadline) {
  const auto packet = ControllerProtocol::build_lisp_erase_request(size);
  if (packet.empty()) {
    return OperationResult::InvalidData;
  }

  const auto reply =
      request_payload(packet, CommandId::LispEraseCode, deadline, true);
  if (!reply) {
    return OperationResult::NoReply;
  }
  return ControllerProtocol::parse_bool_ack(*reply, CommandId::LispEraseCode)
             ? OperationResult::Success
             : OperationResult::Rejected;
}

OperationResult ControllerClient::erase_lisp_code(std::chrono::milliseconds timeout) {
  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return OperationResult::NoReply;
  }
  return erase_lisp_code_until(kEmptyLispEraseBytes, deadline);
}

OperationResult ControllerClient::write_lisp_code(const LispCodeImage& image,
                                                std::chrono::milliseconds timeout) {
  auto packed = ControllerProtocol::pack_lisp_code(image);
  if (packed.empty()) {
    return OperationResult::InvalidData;
  }

  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return OperationResult::NoReply;
  }

  const auto erase_size = static_cast<std::uint32_t>(
      image.bytes.size() + 2U + static_cast<std::size_t>(kLispEraseMarginBytes));
  const auto erase_result = erase_lisp_code_until(erase_size, deadline);
  if (erase_result != OperationResult::Success) {
    return erase_result;
  }

  std::uint32_t offset = 0;
  while (offset < packed.size()) {
    const auto chunk_size = std::min(kLispWriteChunkBytes, packed.size() - offset);
    const auto chunk_begin = packed.begin() + static_cast<std::ptrdiff_t>(offset);
    Payload chunk(chunk_begin, chunk_begin + static_cast<std::ptrdiff_t>(chunk_size));
    const auto reply =
        request_payload(ControllerProtocol::build_lisp_write_request(chunk, offset),
                        CommandId::LispWriteCode, deadline, true);
    if (!reply) {
      return OperationResult::NoReply;
    }
    if (!ControllerProtocol::parse_lisp_write_ack(*reply, offset)) {
      return OperationResult::Rejected;
    }
    offset += static_cast<std::uint32_t>(chunk_size);
  }

  return OperationResult::Success;
}

OperationResult ControllerClient::set_lisp_running(bool running,
                                                 std::chrono::milliseconds timeout) {
  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return OperationResult::NoReply;
  }

  const auto reply =
      request_payload(ControllerProtocol::build_lisp_set_running_request(running),
                      CommandId::LispSetRunning, deadline, true);
  if (!reply) {
    return OperationResult::NoReply;
  }
  return ControllerProtocol::parse_bool_ack(*reply, CommandId::LispSetRunning)
             ? OperationResult::Success
             : OperationResult::Rejected;
}

bool ControllerClient::set_app_output_disabled(std::chrono::milliseconds duration) {
  return enqueue_command(ControllerProtocol::build_app_disable_output_command(duration), false);
}

FocCalibrationResult
ControllerClient::run_foc_calibration(const FocCalibrationParameters& parameters,
                                std::chrono::milliseconds timeout) {
  const auto packet = ControllerProtocol::build_foc_calibration_request(parameters);
  if (packet.empty()) {
    return {OperationResult::InvalidData, std::nullopt};
  }

  const auto deadline = SteadyClock::now() + timeout;
  std::unique_lock lock(management_mutex_, std::defer_lock);
  if (timeout <= std::chrono::milliseconds::zero() || !lock.try_lock_until(deadline)) {
    return {OperationResult::NoReply, std::nullopt};
  }

  constexpr auto kSuppressionMargin = std::chrono::seconds(5);
  const auto max_duration =
      std::chrono::milliseconds(std::numeric_limits<std::int32_t>::max());
  const auto suppression_duration =
      timeout > max_duration - kSuppressionMargin ? max_duration : timeout + kSuppressionMargin;
  if (!set_app_output_disabled(suppression_duration)) {
    return {OperationResult::NoReply, std::nullopt};
  }

  const auto reply =
      request_payload(packet, CommandId::DetectApplyAllFoc, deadline, true);
  if (running_.load()) {
    (void)set_app_output_disabled(std::chrono::milliseconds::zero());
  }
  if (!reply) {
    return {OperationResult::NoReply, std::nullopt};
  }

  const auto code = ControllerProtocol::parse_foc_calibration_reply(*reply);
  if (!code) {
    return {OperationResult::Rejected, std::nullopt};
  }
  return {*code < 0 ? OperationResult::Rejected : OperationResult::Success, code};
}

std::uint64_t ControllerClient::wall_time_ns() const {
  if (config_.wall_time_ns) {
    try {
      return config_.wall_time_ns();
    } catch (...) {
    }
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

void ControllerClient::io_loop() {
  active_io_client = this;
  const int nfds = std::max(fd_, wake_pipe_[0]) + 1;

  while (running_.load()) {
    const auto schedule_now = SteadyClock::now();
    for (PollChannel* channel : {&imu_channel_, &motor_channel_}) {
      if (channel->reschedule.exchange(false)) {
        channel->next_due = schedule_now;
      }
    }

    if (handle_request_timeout()) {
      break;
    }

    const auto now = SteadyClock::now();
    auto wait_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::milliseconds(50));

    const auto tighten_wait = [&](std::chrono::microseconds candidate) {
      wait_time = std::min(wait_time, candidate);
    };

    {
      std::lock_guard lock(scheduler_mutex_);
      if (in_flight_request_) {
        tighten_wait(micros_until(in_flight_request_->deadline, now));
      }
      if (control_watchdog_.armed) {
        tighten_wait(micros_until(control_watchdog_.deadline, now));
      }
      if (!command_queue_.empty()) {
        tighten_wait(std::chrono::microseconds::zero());
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
    const auto raw = wait_time.count();
    const auto bounded = raw > 0 ? raw : 0;
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(bounded / 1000000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>(bounded % 1000000);

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    FD_SET(wake_pipe_[0], &rfds);

    const int rc = ::select(nfds, &rfds, nullptr, nullptr, &tv);
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
        running_.store(false);
        break;
      }
    }

    if (auto watchdog_command = dequeue_due_watchdog_command(SteadyClock::now())) {
      if (!write_packet(*watchdog_command)) {
        running_.store(false);
        break;
      }
      continue;
    }

    std::optional<std::vector<std::uint8_t>> command;
    {
      std::lock_guard lock(scheduler_mutex_);
      if (!command_queue_.empty()) {
        command = std::move(command_queue_.front());
        command_queue_.pop_front();
      }
    }
    if (command && !write_packet(*command)) {
      running_.store(false);
      break;
    }

    if (handle_request_timeout()) {
      break;
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
        if (query->on_timeout) {
          query->on_timeout();
        }
        running_.store(false);
        break;
      }
      std::lock_guard lock(scheduler_mutex_);
      in_flight_request_ = std::move(*query);
    }
  }

  clear_pending_work();
  active_io_client = nullptr;
}

void ControllerClient::dispatch_payload(const Payload& payload, std::uint64_t stamp_ns) {
  if (payload.empty()) {
    return;
  }

  std::optional<ScheduledRequest> completed_request;
  {
    std::lock_guard lock(scheduler_mutex_);
    if (in_flight_request_ && in_flight_request_->expected_id == payload[0]) {
      completed_request = std::move(in_flight_request_);
      in_flight_request_.reset();
    }
  }

  if (completed_request) {
    if (completed_request->kind == ScheduledRequest::Kind::PollImu) {
      if (auto data = ControllerProtocol::parse_get_imu_data(payload)) {
        data->stamp_ns = stamp_ns;
        {
          std::lock_guard lock(cache_mutex_);
          imu_data_cache_ = *data;
        }
        publish_imu(*data);
      }
      return;
    }

    if (completed_request->kind == ScheduledRequest::Kind::PollMotorState) {
      if (auto state = ControllerProtocol::parse_get_values(payload)) {
        state->stamp_ns = stamp_ns;
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

bool ControllerClient::handle_request_timeout() {
  std::optional<ScheduledRequest> timed_out;
  std::vector<ScheduledRequest> expired_queries;
  {
    std::lock_guard lock(scheduler_mutex_);
    const auto now = SteadyClock::now();
    if (in_flight_request_ && in_flight_request_->deadline <= now) {
      timed_out = std::move(in_flight_request_);
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

  const bool stop_transport = timed_out && timed_out->stop_on_timeout;
  if (stop_transport) {
    running_.store(false);
  }
  if (timed_out && timed_out->on_timeout) {
    timed_out->on_timeout();
  }
  for (auto& expired : expired_queries) {
    if (expired.on_timeout) {
      expired.on_timeout();
    }
  }
  return stop_transport;
}

bool ControllerClient::enqueue_command(std::vector<std::uint8_t> packet, bool refresh_watchdog) {
  std::lock_guard lock(scheduler_mutex_);
  if (!running_.load() || packet.size() < 3) {
    return false;
  }
  if (refresh_watchdog && control_watchdog_enabled()) {
    control_watchdog_.deadline = SteadyClock::now() + config_.command_watchdog_timeout;
    control_watchdog_.armed = true;
  }
  const auto command_id = packet[2];
  command_queue_.erase(
      std::remove_if(command_queue_.begin(), command_queue_.end(),
                     [command_id](const auto& queued) {
                       return queued.size() >= 3 && queued[2] == command_id;
                     }),
      command_queue_.end());
  command_queue_.push_back(std::move(packet));
  wake_io_thread();
  return true;
}

bool ControllerClient::enqueue_control_command(std::vector<std::uint8_t> packet) {
  return enqueue_command(std::move(packet), true);
}

bool ControllerClient::control_watchdog_enabled() const {
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
ControllerClient::dequeue_due_watchdog_command(const SteadyClock::time_point& now) {
  if (!running_.load() || !control_watchdog_enabled()) {
    return std::nullopt;
  }

  {
    std::lock_guard lock(scheduler_mutex_);
    if (!control_watchdog_.armed || control_watchdog_.deadline > now) {
      return std::nullopt;
    }
    control_watchdog_ = ControlWatchdogState{};
    command_queue_.clear();
  }

  if (config_.command_watchdog_action == ControlWatchdogAction::Coast) {
    return ControllerProtocol::build_set_current_command(0.0f);
  }
  if (config_.command_watchdog_action == ControlWatchdogAction::BrakeCurrent) {
    return ControllerProtocol::build_set_current_brake_command(
        clamp_brake_current(config_.command_watchdog_brake_current, config_.max_brake_current));
  }
  return std::nullopt;
}

void ControllerClient::wake_io_thread() const {
  if (wake_pipe_[1] < 0) {
    return;
  }
  const std::uint8_t byte = 1;
  (void)::write(wake_pipe_[1], &byte, 1);
}

bool ControllerClient::write_packet(const std::vector<std::uint8_t>& pkt) {
  if (pkt.empty()) {
    return false;
  }
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

bool ControllerClient::wait_until_writable() {
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

void ControllerClient::publish_imu(const ImuData& data) {
  const auto registry = callback_registry_;
  std::unordered_map<std::size_t, ImuCallback> callbacks;
  {
    std::lock_guard lock(registry->mutex);
    callbacks = registry->imu_callbacks;
  }
  invoke_callbacks(callbacks, data);
}

void ControllerClient::publish_motor_state(const MotorState& state) {
  const auto registry = callback_registry_;
  std::unordered_map<std::size_t, MotorStateCallback> callbacks;
  {
    std::lock_guard lock(registry->mutex);
    callbacks = registry->motor_callbacks;
  }
  invoke_callbacks(callbacks, state);
}

void ControllerClient::clear_pending_work() {
  std::deque<ScheduledRequest> queued_requests;
  std::optional<ScheduledRequest> in_flight;
  {
    std::lock_guard lock(scheduler_mutex_);
    command_queue_.clear();
    queued_requests.swap(request_queue_);
    in_flight = std::move(in_flight_request_);
    in_flight_request_.reset();
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

ControllerClient::PollChannel* ControllerClient::select_due_poll_channel(const SteadyClock::time_point& now) {
  PollChannel* selected = nullptr;
  SteadyClock::time_point selected_due{};

  for (PollChannel* channel : {&imu_channel_, &motor_channel_}) {
    const auto interval_ms = channel->interval_ms.load();
    if (interval_ms <= 0) {
      continue;
    }

    if (channel->next_due > now) {
      continue;
    }

    if (!selected || channel->next_due < selected_due ||
        (channel->next_due == selected_due && channel->kind == PollChannel::Kind::Imu)) {
      selected = channel;
      selected_due = channel->next_due;
    }
  }

  return selected;
}

std::optional<ControllerClient::ScheduledRequest>
ControllerClient::make_due_poll_request(PollChannel& channel, const SteadyClock::time_point& now) {
  const auto interval_ms = channel.interval_ms.load();
  if (interval_ms <= 0) {
    return std::nullopt;
  }

  const auto interval = std::chrono::milliseconds(interval_ms);
  if (channel.next_due > now) {
    return std::nullopt;
  }

  channel.next_due = now + interval;

  ScheduledRequest request;
  request.expected_id = static_cast<std::uint8_t>(channel.kind == PollChannel::Kind::Imu
                                                      ? CommandId::GetImuData
                                                      : CommandId::GetValues);
  request.deadline = now + config_.poll_response_timeout;

  if (channel.kind == PollChannel::Kind::Imu) {
    request.kind = ScheduledRequest::Kind::PollImu;
    request.packet = ControllerProtocol::build_get_imu_data_request();
  } else {
    request.kind = ScheduledRequest::Kind::PollMotorState;
    request.packet = ControllerProtocol::build_get_values_request();
  }
  return request;
}

std::optional<ControllerClient::ScheduledRequest>
ControllerClient::dequeue_ready_query(const SteadyClock::time_point& now) {
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
      const auto time_until_imu = imu_channel_.interval_ms.load() > 0
                                      ? imu_channel_.next_due - now
                                      : SteadyClock::duration::max();
      const auto time_until_motor = motor_channel_.interval_ms.load() > 0
                                        ? motor_channel_.next_due - now
                                        : SteadyClock::duration::max();
      const auto next_poll_due = std::min(time_until_imu, time_until_motor);

      if (next_poll_due > config_.query_guard_window) {
        ready_query = std::move(request_queue_.front());
        request_queue_.pop_front();
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

} // namespace goat_motor_controller
