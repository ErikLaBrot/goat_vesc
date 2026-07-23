/**
 * @file protocol_ids.hpp
 * @brief Protocol constants and command IDs.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace goat_vesc {

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
 * @brief Packet command IDs used by the supported message set.
 */
enum class VescPacketCommID : std::uint8_t {
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
  /** IMU telemetry request/reply. */
  GetImuData = 65,
};

} // namespace goat_vesc
