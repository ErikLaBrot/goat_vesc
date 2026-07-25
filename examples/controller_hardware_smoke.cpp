/*
 * Integrated real-hardware smoke example.
 *
 * Demonstrates:
 * - firmware queries
 * - continuous IMU and motor-state polling
 * - callback-based telemetry subscriptions
 * - concurrent command streaming under live telemetry load
 * - best-effort neutral output queueing at the end
 *
 * This touches real hardware and is intended as a manual smoke tool for the
 * shipped public API surface.
 */

#include "goat_motor_controller/controller_client.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace goat_motor_controller;
using namespace std::chrono_literals;

namespace {

using SteadyClock = std::chrono::steady_clock;

struct Options {
  std::string device_path;
  int baud{115200};
  std::chrono::milliseconds imu_poll_interval{20};
  std::chrono::milliseconds motor_poll_interval{50};
  std::chrono::milliseconds phase_duration{1500};
  std::chrono::milliseconds status_interval{500};
  float duty{0.05f};
  float current{10.0f};
  int rpm{1000};
  float brake_current{0.75f};
  float servo_center{0.50f};
  float servo_amplitude{0.10f};
  bool arm_actuators{false};
};

struct PhaseResult {
  std::string name;
  bool ok{false};
  std::size_t commands_sent{0};
};

struct ThrottleSteeringCommand {
  float duty{0.0f};
  float servo{0.50f};
};

struct TelemetrySnapshot {
  std::size_t imu_samples{0};
  std::size_t motor_samples{0};
  std::optional<ImuData> imu;
  std::optional<MotorState> motor;
  std::optional<std::chrono::milliseconds> imu_age;
  std::optional<std::chrono::milliseconds> motor_age;
  bool fault_observed{false};
};

class TelemetryMonitor {
public:
  void on_imu(const ImuData& imu) {
    std::lock_guard lock(mutex_);
    ++imu_samples_;
    last_imu_ = imu;
    last_imu_seen_ = SteadyClock::now();
  }

  void on_motor(const MotorState& motor) {
    std::lock_guard lock(mutex_);
    ++motor_samples_;
    last_motor_ = motor;
    last_motor_seen_ = SteadyClock::now();
    fault_observed_ = fault_observed_ || motor.fault_code != 0;
  }

  TelemetrySnapshot snapshot() const {
    std::lock_guard lock(mutex_);
    TelemetrySnapshot snapshot;
    snapshot.imu_samples = imu_samples_;
    snapshot.motor_samples = motor_samples_;
    snapshot.imu = last_imu_;
    snapshot.motor = last_motor_;
    snapshot.fault_observed = fault_observed_;

    const auto now = SteadyClock::now();
    if (last_imu_) {
      snapshot.imu_age =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_seen_);
    }
    if (last_motor_) {
      snapshot.motor_age =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motor_seen_);
    }
    return snapshot;
  }

private:
  mutable std::mutex mutex_;
  std::size_t imu_samples_{0};
  std::size_t motor_samples_{0};
  std::optional<ImuData> last_imu_;
  std::optional<MotorState> last_motor_;
  SteadyClock::time_point last_imu_seen_;
  SteadyClock::time_point last_motor_seen_;
  bool fault_observed_{false};
};

void print_usage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options]\n"
            << "  --device PATH              Serial device; required when actuators are armed\n"
            << "  --baud BAUD               Baud rate, default 115200\n"
            << "  --imu-poll-ms MS          IMU poll interval, default 20\n"
            << "  --motor-poll-ms MS        Motor poll interval, default 50\n"
            << "  --phase-ms MS             Per-phase command duration, default 1500\n"
            << "  --status-ms MS            Status print interval, default 500\n"
            << "  --duty VALUE              Duty amplitude, default 0.05\n"
            << "  --current AMPS            Current amplitude in amps, default 10.0\n"
            << "  --rpm VALUE               RPM target, default 1000\n"
            << "  --brake-current AMPS      Brake current amplitude; 0 skips braking, default 0.75\n"
            << "  --servo-center VALUE      Servo center in [0, 1], default 0.50\n"
            << "  --servo-amplitude VALUE   Servo half-range, default 0.10\n"
            << "  --arm-actuators           Enable live command phases\n"
            << "  -h, --help                Show this help text\n";
}

float clamp_servo(float value) {
  return std::clamp(value, 0.0f, 1.0f);
}

template <typename T> T parse_value(const std::string& flag, const std::string& text) {
  std::istringstream stream(text);
  T value{};
  stream >> value;
  if (!stream || !stream.eof()) {
    throw std::runtime_error("invalid value for " + flag + ": " + text);
  }
  return value;
}

std::string require_next_value(int& index, int argc, char** argv, const std::string& flag) {
  if (index + 1 >= argc) {
    throw std::runtime_error("missing value for " + flag);
  }
  ++index;
  return argv[index];
}

Options parse_args(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--device") {
      options.device_path = require_next_value(i, argc, argv, arg);
    } else if (arg == "--baud") {
      options.baud = parse_value<int>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--imu-poll-ms") {
      options.imu_poll_interval =
          std::chrono::milliseconds(parse_value<int>(arg, require_next_value(i, argc, argv, arg)));
    } else if (arg == "--motor-poll-ms") {
      options.motor_poll_interval =
          std::chrono::milliseconds(parse_value<int>(arg, require_next_value(i, argc, argv, arg)));
    } else if (arg == "--phase-ms") {
      options.phase_duration =
          std::chrono::milliseconds(parse_value<int>(arg, require_next_value(i, argc, argv, arg)));
    } else if (arg == "--status-ms") {
      options.status_interval =
          std::chrono::milliseconds(parse_value<int>(arg, require_next_value(i, argc, argv, arg)));
    } else if (arg == "--duty") {
      options.duty = parse_value<float>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--current") {
      options.current = parse_value<float>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--rpm") {
      options.rpm = parse_value<int>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--brake-current") {
      options.brake_current = parse_value<float>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--servo-center") {
      options.servo_center = parse_value<float>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--servo-amplitude") {
      options.servo_amplitude = parse_value<float>(arg, require_next_value(i, argc, argv, arg));
    } else if (arg == "--arm-actuators") {
      options.arm_actuators = true;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (options.baud <= 0) {
    throw std::runtime_error("baud must be positive");
  }
  if (options.imu_poll_interval <= 0ms) {
    throw std::runtime_error("imu poll interval must be positive");
  }
  if (options.motor_poll_interval <= 0ms) {
    throw std::runtime_error("motor poll interval must be positive");
  }
  if (options.phase_duration <= 0ms) {
    throw std::runtime_error("phase duration must be positive");
  }
  if (options.status_interval <= 0ms) {
    throw std::runtime_error("status interval must be positive");
  }
  if (!std::isfinite(options.duty) || options.duty < 0.0f || options.duty > 0.20f) {
    throw std::runtime_error("duty must be in [0.0, 0.20]");
  }
  if (!std::isfinite(options.current) || options.current <= 0.0f || options.current > 20.0f) {
    throw std::runtime_error("current must be in (0.0, 20.0]");
  }
  if (options.rpm < 1000 || options.rpm > 3000) {
    throw std::runtime_error("rpm must be in [1000, 3000] for the hardware smoke test");
  }
  if (!std::isfinite(options.brake_current) || options.brake_current < 0.0f ||
      options.brake_current > 5.0f) {
    throw std::runtime_error("brake current must be in [0.0, 5.0]");
  }
  if (!std::isfinite(options.servo_center) || options.servo_center < 0.0f ||
      options.servo_center > 1.0f) {
    throw std::runtime_error("servo center must be in [0.0, 1.0]");
  }
  if (!std::isfinite(options.servo_amplitude) || options.servo_amplitude < 0.0f ||
      options.servo_amplitude > 0.40f) {
    throw std::runtime_error("servo amplitude must be in [0.0, 0.40]");
  }
  if (options.arm_actuators && options.device_path.empty()) {
    throw std::runtime_error("--device is required with --arm-actuators");
  }

  return options;
}

template <typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = SteadyClock::now() + timeout;
  while (SteadyClock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}

template <typename T, typename Sender>
PhaseResult stream_phase(const std::string& name, const std::vector<T>& values, ControllerClient& client,
                         std::chrono::milliseconds phase_duration,
                         std::chrono::milliseconds command_period,
                         const std::atomic<bool>& stop_requested, Sender sender) {
  PhaseResult result{name};

  std::cout << "Starting " << name << " phase\n";
  const auto deadline = SteadyClock::now() + phase_duration;
  std::size_t index = 0;
  while (SteadyClock::now() < deadline && !stop_requested.load()) {
    const T value = values[index % values.size()];
    if (!sender(client, value)) {
      std::cerr << name << " phase command failed\n";
      return result;
    }
    ++result.commands_sent;
    ++index;
    std::this_thread::sleep_for(command_period);
  }

  if (stop_requested.load()) {
    return result;
  }
  result.ok = true;
  return result;
}

std::string age_string(const std::optional<std::chrono::milliseconds>& age) {
  if (!age) {
    return "n/a";
  }
  return std::to_string(age->count()) + "ms";
}

void print_status(const TelemetrySnapshot& snapshot) {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "[status] imu_samples=" << snapshot.imu_samples
            << " motor_samples=" << snapshot.motor_samples
            << " imu_age=" << age_string(snapshot.imu_age)
            << " motor_age=" << age_string(snapshot.motor_age);

  if (snapshot.motor) {
    const float power_w = snapshot.motor->vin * snapshot.motor->current_in;
    std::cout << " rpm=" << snapshot.motor->rpm << " vin=" << snapshot.motor->vin
              << " current_in=" << snapshot.motor->current_in << " power_w=" << power_w;
  }
  if (snapshot.imu) {
    std::cout << " gyro_z=" << snapshot.imu->gyro_z;
  }
  std::cout << '\n';
}

bool queue_safe_outputs(ControllerClient& client, float servo_center) {
  const bool ok =
      client.set_current(0.0f) && client.set_servo_pos(clamp_servo(servo_center));
  std::this_thread::sleep_for(300ms);
  return ok;
}

} // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_args(argc, argv);

    ControllerConfig config;
    config.device_path = options.device_path;
    config.baud = options.baud;
    config.imu_poll_interval = options.imu_poll_interval;
    config.motor_poll_interval = options.motor_poll_interval;
    config.poll_response_timeout = 100ms;
    config.max_brake_current = options.brake_current;
    config.command_watchdog_timeout = 500ms;
    config.command_watchdog_action = ControlWatchdogAction::Coast;

    std::cout << "Motor-controller hardware smoke starting\n";
    std::cout << "  device: "
              << (config.device_path.empty() ? "auto-detect first /dev/ttyACM*"
                                             : config.device_path)
              << '\n';
    std::cout << "  baud: " << config.baud << '\n';
    std::cout << "  imu_poll_ms: " << config.imu_poll_interval.count() << '\n';
    std::cout << "  motor_poll_ms: " << config.motor_poll_interval.count() << '\n';
    std::cout << "  phase_ms: " << options.phase_duration.count() << '\n';
    std::cout << "  status_ms: " << options.status_interval.count() << '\n';
    std::cout << "  arm_actuators: " << (options.arm_actuators ? "yes" : "no") << '\n';

    ControllerClient client(config);
    if (!client.connect()) {
      std::cerr << "Connect failed\n";
      return 2;
    }

    TelemetryMonitor monitor;
    auto imu_handle =
        client.subscribe_imu([&monitor](const ImuData& imu) { monitor.on_imu(imu); });
    auto motor_handle = client.subscribe_motor_state(
        [&monitor](const MotorState& motor) { monitor.on_motor(motor); });
    (void)imu_handle;
    (void)motor_handle;

    const auto initial_fw = client.request_fw_version(400ms);
    if (!initial_fw) {
      std::cerr << "Initial firmware query failed\n";
      client.disconnect();
      return 3;
    }
    std::cout << "Initial firmware query: " << static_cast<int>(initial_fw->major) << '.'
              << static_cast<int>(initial_fw->minor) << '\n';

    const bool got_imu =
        wait_until([&monitor] { return monitor.snapshot().imu.has_value(); }, 1500ms);
    const bool got_motor =
        wait_until([&monitor] { return monitor.snapshot().motor.has_value(); }, 1500ms);
    if (!got_imu || !got_motor) {
      std::cerr << "Did not receive required startup telemetry samples\n";
      client.disconnect();
      return 4;
    }

    std::cout << "Telemetry subscriptions are live\n";
    const auto startup_snapshot = monitor.snapshot();
    print_status(startup_snapshot);
    if (startup_snapshot.fault_observed) {
      std::cerr << "Controller reported a fault; refusing command phases\n";
      client.disconnect();
      return 5;
    }

    std::atomic<bool> command_thread_done{false};
    std::atomic<int> active_phase{0};
    std::atomic<bool> stop_commands{false};
    std::vector<PhaseResult> phase_results;
    std::atomic<bool> phase_failure{false};
    bool fw_query_under_load_completed = false;
    bool fw_query_under_load_success = false;
    bool imu_progress_during_commands = false;
    bool motor_progress_during_commands = false;
    float rpm_phase_peak_abs_rpm = 0.0f;
    auto last_progress_snapshot = monitor.snapshot();
    const auto telemetry_stale_timeout =
        std::max(options.imu_poll_interval, options.motor_poll_interval) * 4 +
        config.poll_response_timeout;

    std::thread command_thread;
    if (options.arm_actuators) {
      command_thread = std::thread([&] {
        const float left = clamp_servo(options.servo_center - options.servo_amplitude);
        const float right = clamp_servo(options.servo_center + options.servo_amplitude);

        auto record_result = [&](const PhaseResult& result) {
          if (!result.ok) {
            phase_failure.store(true);
          }
          phase_results.push_back(result);
        };

        auto run_phase = [&](int phase_index, auto&& runner) {
          active_phase.store(phase_index);
          const PhaseResult result = runner();
          record_result(result);
          active_phase.store(0);
          return result.ok;
        };

        if (!run_phase(1, [&] {
              return stream_phase<float>(
                  "duty", {0.0f, options.duty * 0.5f, options.duty, options.duty * 0.5f, 0.0f},
                  client, options.phase_duration, 120ms, stop_commands,
                  [](ControllerClient& c, float value) { return c.set_duty(value); });
            })) {
          command_thread_done.store(true);
          return;
        }
        (void)client.set_duty(0.0f);

        if (!run_phase(2, [&] {
              return stream_phase<float>(
                  "current",
                  {0.0f, options.current * 0.5f, options.current, options.current * 0.5f, 0.0f},
                  client, options.phase_duration, 120ms, stop_commands,
                  [](ControllerClient& c, float value) { return c.set_current(value); });
            })) {
          command_thread_done.store(true);
          return;
        }
        (void)client.set_current(0.0f);

        if (!run_phase(3, [&] {
              return stream_phase<int>("rpm", {0, options.rpm / 2, options.rpm, options.rpm / 2, 0},
                                       client, options.phase_duration, 120ms, stop_commands,
                                       [](ControllerClient& c, int value) { return c.set_rpm(value); });
            })) {
          command_thread_done.store(true);
          return;
        }
        (void)client.set_rpm(0);

        if (!run_phase(4, [&] {
              return stream_phase<ThrottleSteeringCommand>(
                  "throttle_steering",
                  {{0.0f, options.servo_center},
                   {options.duty * 0.5f, left},
                   {options.duty, options.servo_center},
                   {options.duty * 0.5f, right},
                   {0.0f, options.servo_center}},
                  client, options.phase_duration, 140ms, stop_commands,
                  [](ControllerClient& c, const ThrottleSteeringCommand& command) {
                    return c.set_duty(command.duty) && c.set_servo_pos(clamp_servo(command.servo));
                  });
            })) {
          command_thread_done.store(true);
          return;
        }
        (void)client.set_duty(0.0f);
        (void)client.set_servo_pos(clamp_servo(options.servo_center));

        if (options.brake_current > 0.0f) {
          if (!run_phase(5, [&] {
                return stream_phase<float>(
                    "brake_current",
                    {options.brake_current * 0.5f, options.brake_current,
                     options.brake_current * 0.5f},
                    client, options.phase_duration, 120ms, stop_commands,
                    [](ControllerClient& c, float value) { return c.set_current_brake(value); });
              })) {
            command_thread_done.store(true);
            return;
          }
          (void)client.set_current(0.0f);
        } else {
          std::cout << "Brake-current phase skipped at 0 A\n";
        }

        if (!run_phase(6, [&] {
              return stream_phase<float>(
                  "servo",
                  {options.servo_center, left, options.servo_center, right, options.servo_center},
                  client, options.phase_duration, 160ms, stop_commands,
                  [](ControllerClient& c, float value) { return c.set_servo_pos(clamp_servo(value)); });
            })) {
          command_thread_done.store(true);
          return;
        }
        (void)client.set_servo_pos(clamp_servo(options.servo_center));

        active_phase.store(0);
        command_thread_done.store(true);
      });
    } else {
      std::cout
          << "Actuator phases skipped. Re-run with --arm-actuators for the full command smoke.\n";
    }

    const auto monitoring_deadline =
        SteadyClock::now() +
        (options.arm_actuators ? (options.phase_duration * 6 + 2s) : std::chrono::seconds(3));
    auto next_status = SteadyClock::now();

    while (SteadyClock::now() < monitoring_deadline) {
      const auto snapshot = monitor.snapshot();

      if (options.arm_actuators && active_phase.load() != 0) {
        const bool telemetry_stale =
            !snapshot.imu_age || !snapshot.motor_age ||
            *snapshot.imu_age > telemetry_stale_timeout ||
            *snapshot.motor_age > telemetry_stale_timeout;
        if (snapshot.fault_observed || telemetry_stale) {
          std::cerr << (snapshot.fault_observed ? "Controller fault" : "Telemetry stale")
                    << " during an actuator phase; stopping commands\n";
          phase_failure.store(true);
          stop_commands.store(true);
          break;
        }
        if (snapshot.imu_samples > last_progress_snapshot.imu_samples) {
          imu_progress_during_commands = true;
        }
        if (snapshot.motor_samples > last_progress_snapshot.motor_samples) {
          motor_progress_during_commands = true;
        }
        if (active_phase.load() == 3 && snapshot.motor) {
          rpm_phase_peak_abs_rpm = std::max(rpm_phase_peak_abs_rpm, std::abs(snapshot.motor->rpm));
        }
      }
      last_progress_snapshot = snapshot;

      if (SteadyClock::now() >= next_status) {
        print_status(snapshot);
        next_status += options.status_interval;
      }

      if (options.arm_actuators && !fw_query_under_load_completed && active_phase.load() != 0) {
        std::cout << "Running firmware query under load\n";
        fw_query_under_load_success = client.request_fw_version(300ms).has_value();
        fw_query_under_load_completed = true;
        std::cout << "Firmware query under load: "
                  << (fw_query_under_load_success ? "success" : "timeout/clean miss") << '\n';
      }

      if (options.arm_actuators && command_thread_done.load() &&
          (fw_query_under_load_completed || phase_failure.load())) {
        break;
      }

      std::this_thread::sleep_for(50ms);
    }

    if (command_thread.joinable()) {
      command_thread.join();
    }

    bool safe_outputs_queued = true;
    if (options.arm_actuators) {
      safe_outputs_queued =
          client.is_connected() && queue_safe_outputs(client, options.servo_center);
    }

    const auto final_snapshot = monitor.snapshot();
    print_status(final_snapshot);
    const bool transport_connected = client.is_connected();
    const bool telemetry_fresh =
        final_snapshot.imu_age && final_snapshot.motor_age &&
        *final_snapshot.imu_age <= telemetry_stale_timeout &&
        *final_snapshot.motor_age <= telemetry_stale_timeout;

    client.disconnect();

    const bool rpm_feedback_observed = rpm_phase_peak_abs_rpm > 1.0f;
    bool success = true;
    if (!options.arm_actuators) {
      success = final_snapshot.imu_samples > 0 && final_snapshot.motor_samples > 0 &&
                transport_connected && telemetry_fresh && !final_snapshot.fault_observed;
    } else {
      success = !phase_failure.load() && imu_progress_during_commands &&
                motor_progress_during_commands && fw_query_under_load_success &&
                safe_outputs_queued && rpm_feedback_observed && transport_connected &&
                telemetry_fresh && !final_snapshot.fault_observed;
    }

    std::cout << "Smoke summary:\n";
    std::cout << "  initial_fw_query: success\n";
    const char* fw_query_status =
        !options.arm_actuators
            ? "skipped"
            : (fw_query_under_load_success
                   ? "success"
                   : (fw_query_under_load_completed ? "timeout/clean miss" : "not-run"));
    std::cout << "  fw_query_under_load: "
              << fw_query_status << '\n';
    std::cout << "  imu_samples: " << final_snapshot.imu_samples << '\n';
    std::cout << "  motor_samples: " << final_snapshot.motor_samples << '\n';
    std::cout << "  transport_connected: " << (transport_connected ? "yes" : "no") << '\n';
    std::cout << "  telemetry_fresh: " << (telemetry_fresh ? "yes" : "no") << '\n';
    std::cout << "  controller_fault_observed: "
              << (final_snapshot.fault_observed ? "yes" : "no") << '\n';
    std::cout << "  rpm_feedback_observed: " << (rpm_feedback_observed ? "yes" : "no") << '\n';
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  rpm_phase_peak_abs_rpm: " << rpm_phase_peak_abs_rpm << '\n';
    if (final_snapshot.motor) {
      const float power_w = final_snapshot.motor->vin * final_snapshot.motor->current_in;
      std::cout << "  last_vin: " << final_snapshot.motor->vin << '\n';
      std::cout << "  last_current_in: " << final_snapshot.motor->current_in << '\n';
      std::cout << "  last_power_w: " << power_w << '\n';
    }

    for (const auto& phase : phase_results) {
      std::cout << "  phase_" << phase.name << ": " << (phase.ok ? "ok" : "failed") << " ("
                << phase.commands_sent << " commands)\n";
    }

    if (!success) {
      std::cerr << "Hardware smoke failed\n";
      return 6;
    }

    std::cout << "Hardware smoke OK\n";
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Hardware smoke error: " << ex.what() << '\n';
    return 7;
  }
}
