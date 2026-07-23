/**
 * @file types.hpp
 * @brief Public configuration and data types used by `goat_vesc`.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace goat_vesc {

/**
 * @brief Host-side stale-command safety behavior used by `VescClient`.
 */
enum class ControlWatchdogAction {
  /** Disable the host-side command watchdog. */
  Disabled,
  /** Send a zero-current command when the watchdog expires. */
  Coast,
  /** Send a bounded brake-current command when the watchdog expires. */
  BrakeCurrent,
};

/**
 * @brief Runtime configuration used to construct a `VescClient`.
 */
struct VescConfig {
  /** Serial device path. Ignored when `open_serial_fn` is supplied. */
  std::string device_path{};
  /** Serial baud rate. Supported values match the built-in termios mapping. */
  int baud{115200};
  /** Periodic motor-state polling cadence. `0 ms` disables motor polling. */
  std::chrono::milliseconds motor_poll_interval{50};
  /** Periodic IMU polling cadence. `0 ms` disables IMU polling. */
  std::chrono::milliseconds imu_poll_interval{10};
  /** Deadline for a sent periodic poll to receive a reply. */
  std::chrono::milliseconds poll_response_timeout{20};
  /** Guard band that keeps one-shot queries from cutting too close to polls. */
  std::chrono::milliseconds query_guard_window{5};
  /** Host-side stale-command watchdog timeout. `0 ms` disables it. */
  std::chrono::milliseconds command_watchdog_timeout{0};
  /** Safe-stop action sent once when control input goes stale. */
  ControlWatchdogAction command_watchdog_action{ControlWatchdogAction::Disabled};
  /** Maximum allowed active brake-current magnitude in amps. */
  float max_brake_current{0.0f};
  /** Requested watchdog brake current before clamping to `max_brake_current`. */
  float command_watchdog_brake_current{0.0f};
  /** Optional wall-clock source used to stamp decoded samples in nanoseconds. */
  std::function<std::uint64_t()> wall_time_ns;
  /** Optional transport opener used for tests or custom serial backends. */
  std::function<bool(const VescConfig&, int&)> open_serial_fn;
};

/**
 * @brief Snapshot of the bridge-facing configuration currently visible to the client.
 *
 * Poll intervals reflect runtime updates applied through the setter methods.
 */
struct VescClientConfigSnapshot {
  /** Current motor-state polling cadence. */
  std::chrono::milliseconds motor_poll_interval{0};
  /** Current IMU polling cadence. */
  std::chrono::milliseconds imu_poll_interval{0};
  /** Periodic poll reply timeout. */
  std::chrono::milliseconds poll_response_timeout{0};
  /** Query guard window before a due periodic poll. */
  std::chrono::milliseconds query_guard_window{0};
  /** Host-side command watchdog timeout. */
  std::chrono::milliseconds command_watchdog_timeout{0};
  /** Safe-stop behavior chosen for the command watchdog. */
  ControlWatchdogAction command_watchdog_action{ControlWatchdogAction::Disabled};
  /** Maximum allowed active brake-current magnitude in amps. */
  float max_brake_current{0.0f};
  /** Requested watchdog brake current before clamping. */
  float command_watchdog_brake_current{0.0f};
};

/**
 * @brief Parsed firmware version reported by the VESC.
 */
struct FwVersion {
  /** Major firmware version number. */
  std::uint8_t major{0};
  /** Minor firmware version number. */
  std::uint8_t minor{0};
};

/**
 * @brief Cached or freshly decoded motor telemetry sample.
 */
struct VescMotorState {
  /** Sample timestamp in nanoseconds from the configured wall-clock source. */
  std::uint64_t stamp_ns{0};
  /** Electrical RPM reported by the controller. */
  float rpm{0.0f};
  /** Motor current in amps. */
  float current_motor{0.0f};
  /** Input current in amps. */
  float current_in{0.0f};
  /** Duty cycle reported by the controller. */
  float duty_cycle{0.0f};
  /** Input voltage in volts. */
  float vin{0.0f};
  /** Motor temperature in degrees Celsius. */
  float temp_motor{0.0f};
  /** FET temperature in degrees Celsius. */
  float temp_fet{0.0f};
  /** Signed tachometer count. */
  std::int32_t tachometer{0};
  /** Absolute tachometer count. */
  std::int32_t tachometer_abs{0};
  /** Raw VESC fault code byte. */
  std::uint8_t fault_code{0};
};

/**
 * @brief Cached or freshly decoded IMU sample.
 */
struct VescIMUData {
  /** Sample timestamp in nanoseconds from the configured wall-clock source. */
  std::uint64_t stamp_ns{0};
  /** Roll in radians or firmware-native units reported by the controller. */
  float roll{0.0f};
  /** Pitch in radians or firmware-native units reported by the controller. */
  float pitch{0.0f};
  /** Yaw in radians or firmware-native units reported by the controller. */
  float yaw{0.0f};
  /** Accelerometer X-axis value. */
  float acc_x{0.0f};
  /** Accelerometer Y-axis value. */
  float acc_y{0.0f};
  /** Accelerometer Z-axis value. */
  float acc_z{0.0f};
  /** Gyroscope X-axis value. */
  float gyro_x{0.0f};
  /** Gyroscope Y-axis value. */
  float gyro_y{0.0f};
  /** Gyroscope Z-axis value. */
  float gyro_z{0.0f};
  /** Magnetometer X-axis value. */
  float mag_x{0.0f};
  /** Magnetometer Y-axis value. */
  float mag_y{0.0f};
  /** Magnetometer Z-axis value. */
  float mag_z{0.0f};
  /** Quaternion W component. */
  float quat_w{1.0f};
  /** Quaternion X component. */
  float quat_x{0.0f};
  /** Quaternion Y component. */
  float quat_y{0.0f};
  /** Quaternion Z component. */
  float quat_z{0.0f};
};

} // namespace goat_vesc
