/**
 * @file protocol_ids.hpp
 * @brief Protocol constants, command IDs, and telemetry field masks.
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

/**
 * @brief Bitmask fields that can be requested in `COMM_GET_IMU_DATA`.
 */
enum class VescImuMask : std::uint16_t {
  /** Roll field. */
  Roll = (1u << 0),
  /** Pitch field. */
  Pitch = (1u << 1),
  /** Yaw field. */
  Yaw = (1u << 2),
  /** Accelerometer X field. */
  AccX = (1u << 3),
  /** Accelerometer Y field. */
  AccY = (1u << 4),
  /** Accelerometer Z field. */
  AccZ = (1u << 5),
  /** Gyroscope X field. */
  GyroX = (1u << 6),
  /** Gyroscope Y field. */
  GyroY = (1u << 7),
  /** Gyroscope Z field. */
  GyroZ = (1u << 8),
  /** Magnetometer X field. */
  MagX = (1u << 9),
  /** Magnetometer Y field. */
  MagY = (1u << 10),
  /** Magnetometer Z field. */
  MagZ = (1u << 11),
  /** Quaternion W field. */
  QuatW = (1u << 12),
  /** Quaternion X field. */
  QuatX = (1u << 13),
  /** Quaternion Y field. */
  QuatY = (1u << 14),
  /** Quaternion Z field. */
  QuatZ = (1u << 15)
};

/**
 * @brief Combines two `VescImuMask` bits into a raw mask value.
 * @param lhs Left-hand mask value.
 * @param rhs Right-hand mask value.
 * @return Combined raw bitmask value.
 */
constexpr std::uint16_t operator|(VescImuMask lhs, VescImuMask rhs) {
  return static_cast<std::uint16_t>(lhs) | static_cast<std::uint16_t>(rhs);
}

/**
 * @brief Default IMU field mask requested by the library.
 *
 * This mask requests every field represented by `VescIMUData`.
 */
constexpr std::uint16_t DefaultVescImuMask = 0xFFFFU;

} // namespace goat_vesc
