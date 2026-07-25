/**
 * @file protocol_ids.hpp
 * @brief Protocol constants and command IDs.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace goat_motor_controller {

/**
 * @brief Maximum payload size accepted by the supported VESC framing modes.
 *
 * This limit mirrors the firmware-defined payload ceiling that the library is
 * willing to frame and parse.
 */
constexpr std::size_t kMaxPayloadBytes = 512;
static_assert(kMaxPayloadBytes <= 0xFFFF,
              "Payload ceilings above 65535 bytes require 24-bit framing support.");

/**
 * @brief Maximum number of bytes in a fully framed packet on the wire.
 *
 * The bound includes the start byte, length bytes, payload, CRC, and stop
 * byte.
 */
constexpr std::size_t kMaxFramedPacketBytes = kMaxPayloadBytes + 6;

/**
 * @brief Maximum LispBM code image accepted by the client.
 *
 * This mirrors the largest current VESC Tool code partition while bounding
 * aggregate allocation from a controller-supplied length.
 */
constexpr std::size_t kMaxLispCodeBytes = (512U * 1024U) - 14U;

/**
 * @brief Packet command IDs used by the supported message set.
 */
enum class CommandId : std::uint8_t {
  /** Firmware version request/reply. */
  FwVersion = 0,
  /** Motor telemetry request/reply. */
  GetValues = 4,
  /** Duty-cycle command. */
  SetDuty = 5,
  /** Motor current command. */
  SetCurrent = 6,
  /** Active brake-current command. */
  SetCurrentBrake = 7,
  /** RPM command. */
  SetRpm = 8,
  /** Servo position command. */
  SetServoPos = 12,
  /** Persist a firmware-native motor configuration. */
  SetMotorConfig = 13,
  /** Read the active motor configuration. */
  GetMotorConfig = 14,
  /** Persist a firmware-native application configuration. */
  SetAppConfig = 16,
  /** Read the active application configuration. */
  GetAppConfig = 17,
  /** Exchange application-defined data with the running custom app. */
  CustomAppData = 36,
  /** Detect and persist all local FOC motor parameters. */
  DetectApplyAllFoc = 58,
  /** Temporarily suppress application-generated motor output. */
  AppDisableOutput = 63,
  /** IMU telemetry request/reply. */
  GetImuData = 65,
  /** Read a chunk of stored LispBM code. */
  LispReadCode = 130,
  /** Write a chunk of packed LispBM code. */
  LispWriteCode = 131,
  /** Erase stored LispBM code. */
  LispEraseCode = 132,
  /** Start or stop LispBM execution. */
  LispSetRunning = 133,
  /** Apply an application configuration without persisting it. */
  SetAppConfigNoStore = 149,
};

} // namespace goat_motor_controller
