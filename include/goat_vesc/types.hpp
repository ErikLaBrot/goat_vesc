#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace goat_vesc {

enum class ControlWatchdogAction {
  Disabled,
  Coast,
  BrakeCurrent,
};

struct VescConfig {
  // Serial device path. Ignored if open_serial_fn is supplied.
  std::string device_path{};
  // Supported baud rates are those handled by vesc_client.cpp::baud_to_constant().
  int baud{115200};
  // Periodic telemetry poll cadence.
  std::chrono::milliseconds motor_poll_interval{50};
  std::chrono::milliseconds imu_poll_interval{10};
  // Timeout for a sent poll/query waiting on a response.
  std::chrono::milliseconds poll_response_timeout{20};
  // Do not start a one-shot query if a periodic poll is due sooner than this.
  std::chrono::milliseconds query_guard_window{5};
  // Optional stale-command watchdog. A timeout of 0 ms leaves watchdog behavior disabled.
  std::chrono::milliseconds command_watchdog_timeout{0};
  // Safe-stop action to send once when command input goes stale.
  ControlWatchdogAction command_watchdog_action{ControlWatchdogAction::Disabled};
  // Maximum active brake current magnitude allowed by the bridge for this hardware.
  float max_brake_current{0.0f};
  // Requested brake current used by BrakeCurrent watchdog mode before clamping.
  float command_watchdog_brake_current{0.0f};
  // Optional wall-clock source used to stamp decoded samples in nanoseconds.
  std::function<std::uint64_t()> wall_time_ns;
  // Optional transport injector for tests or custom serial backends.
  std::function<bool(const VescConfig&, int&)> open_serial_fn;
};

struct VescClientConfigSnapshot {
  std::chrono::milliseconds motor_poll_interval{0};
  std::chrono::milliseconds imu_poll_interval{0};
  std::chrono::milliseconds poll_response_timeout{0};
  std::chrono::milliseconds query_guard_window{0};
  std::chrono::milliseconds command_watchdog_timeout{0};
  ControlWatchdogAction command_watchdog_action{ControlWatchdogAction::Disabled};
  float max_brake_current{0.0f};
  float command_watchdog_brake_current{0.0f};
};

struct FwVersion {
  std::uint8_t major{0};
  std::uint8_t minor{0};
};

struct VescMotorState {
  std::uint64_t stamp_ns{0};
  float rpm{0.0f};
  float current_motor{0.0f};
  float current_in{0.0f};
  float duty_cycle{0.0f};
  float vin{0.0f};
  float temp_motor{0.0f};
  float temp_fet{0.0f};
  std::int32_t tachometer{0};
  std::int32_t tachometer_abs{0};
  std::uint8_t fault_code{0};
};

struct VescIMUData {
  std::uint64_t stamp_ns{0};
  float roll{0.0f};
  float pitch{0.0f};
  float yaw{0.0f};
  float acc_x{0.0f};
  float acc_y{0.0f};
  float acc_z{0.0f};
  float gyro_x{0.0f};
  float gyro_y{0.0f};
  float gyro_z{0.0f};
  float mag_x{0.0f};
  float mag_y{0.0f};
  float mag_z{0.0f};
  float quat_w{1.0f};
  float quat_x{0.0f};
  float quat_y{0.0f};
  float quat_z{0.0f};
};

} // namespace goat_vesc
