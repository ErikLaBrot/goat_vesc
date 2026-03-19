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
 * @brief Frame-length encodings used by the VESC serial protocol.
 */
enum class VescPacketLength : std::uint8_t {
  /** One-byte payload length field. */
  Short = 0x02,
  /** Two-byte payload length field. */
  Medium = 0x03,
  /** Three-byte payload length field. Present in the protocol, unused here. */
  Long = 0x04,
};

/**
 * @brief Packet command IDs used by the supported message set.
 *
 * Some values are exposed even when the higher-level client does not yet
 * provide a typed wrapper for them.
 */
enum class VescPacketCommID : std::uint8_t {
  /** Firmware version request/reply. */
  FwVersion = 0,
  /** Bootloader jump command. */
  JumpToBootloader = 1,
  /** Firmware erase command. */
  EraseNewApp = 2,
  /** Firmware write chunk command. */
  WriteNewAppData = 3,
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
  /** Position command. */
  SetPos = 9,
  /** Handbrake command. */
  SetHandbrake = 10,
  /** Detection command. */
  SetDetect = 11,
  /** Servo position command. */
  SetServoPos = 12,
  /** IMU telemetry request/reply. */
  GetImuData = 65,
};

/**
 * @brief Bitmask fields that can appear in `COMM_GET_VALUES` responses.
 */
enum class VescValuesMask : std::uint32_t {
  /** FET temperature field. */
  FetTemp = (1u << 0),
  /** Motor temperature field. */
  MotorTemp = (1u << 1),
  /** Motor current field. */
  CurrentMotor = (1u << 2),
  /** Input current field. */
  CurrentIn = (1u << 3),
  /** Direct-axis current field. */
  CurrentId = (1u << 4),
  /** Quadrature-axis current field. */
  CurrentIq = (1u << 5),
  /** Duty-cycle field. */
  DutyCycle = (1u << 6),
  /** RPM field. */
  Rpm = (1u << 7),
  /** Input voltage field. */
  Vin = (1u << 8),
  /** Consumed amp-hours field. */
  AmpHours = (1u << 9),
  /** Charged amp-hours field. */
  AmpHoursCharged = (1u << 10),
  /** Consumed watt-hours field. */
  WattHours = (1u << 11),
  /** Charged watt-hours field. */
  WattHoursCharged = (1u << 12),
  /** Tachometer field. */
  Tachometer = (1u << 13),
  /** Absolute tachometer field. */
  TachometerAbs = (1u << 14),
  /** Fault-code field. */
  FaultCode = (1u << 15),
  /** PID position field. */
  PidPosNow = (1u << 16),
  /** Controller ID field. */
  ControllerId = (1u << 17),
  /** MOSFET temperature array field. */
  MosTemps = (1u << 18),
  /** Direct-axis voltage field. */
  Vd = (1u << 19),
  /** Quadrature-axis voltage field. */
  Vq = (1u << 20),
  /** Status flags field. */
  StatusFlags = (1u << 21)
};

/**
 * @brief Combines two `VescValuesMask` bits into a raw mask value.
 * @param lhs Left-hand mask value.
 * @param rhs Right-hand mask value.
 * @return Combined raw bitmask value.
 */
constexpr std::uint32_t operator|(VescValuesMask lhs, VescValuesMask rhs) {
  return static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs);
}

/**
 * @brief Default `COMM_GET_VALUES` field mask used by the library.
 *
 * The protocol helpers expose this constant for callers that want the library's
 * standard telemetry field set.
 */
constexpr std::uint32_t DefaultVescValuesMask =
    static_cast<std::uint32_t>(VescValuesMask::FetTemp) |
    static_cast<std::uint32_t>(VescValuesMask::MotorTemp) |
    static_cast<std::uint32_t>(VescValuesMask::CurrentMotor) |
    static_cast<std::uint32_t>(VescValuesMask::CurrentIn) |
    static_cast<std::uint32_t>(VescValuesMask::CurrentId) |
    static_cast<std::uint32_t>(VescValuesMask::CurrentIq) |
    static_cast<std::uint32_t>(VescValuesMask::DutyCycle) |
    static_cast<std::uint32_t>(VescValuesMask::Rpm) |
    static_cast<std::uint32_t>(VescValuesMask::Vin) |
    static_cast<std::uint32_t>(VescValuesMask::AmpHours) |
    static_cast<std::uint32_t>(VescValuesMask::AmpHoursCharged) |
    static_cast<std::uint32_t>(VescValuesMask::WattHours) |
    static_cast<std::uint32_t>(VescValuesMask::WattHoursCharged) |
    static_cast<std::uint32_t>(VescValuesMask::Tachometer) |
    static_cast<std::uint32_t>(VescValuesMask::TachometerAbs) |
    static_cast<std::uint32_t>(VescValuesMask::FaultCode) |
    static_cast<std::uint32_t>(VescValuesMask::PidPosNow) |
    static_cast<std::uint32_t>(VescValuesMask::ControllerId) |
    static_cast<std::uint32_t>(VescValuesMask::MosTemps) |
    static_cast<std::uint32_t>(VescValuesMask::Vd) |
    static_cast<std::uint32_t>(VescValuesMask::Vq) |
    static_cast<std::uint32_t>(VescValuesMask::StatusFlags);

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
constexpr std::uint16_t DefaultVescImuMask =
    static_cast<std::uint16_t>(VescImuMask::Roll) | static_cast<std::uint16_t>(VescImuMask::Pitch) |
    static_cast<std::uint16_t>(VescImuMask::Yaw) | static_cast<std::uint16_t>(VescImuMask::AccX) |
    static_cast<std::uint16_t>(VescImuMask::AccY) | static_cast<std::uint16_t>(VescImuMask::AccZ) |
    static_cast<std::uint16_t>(VescImuMask::GyroX) |
    static_cast<std::uint16_t>(VescImuMask::GyroY) |
    static_cast<std::uint16_t>(VescImuMask::GyroZ) | static_cast<std::uint16_t>(VescImuMask::MagX) |
    static_cast<std::uint16_t>(VescImuMask::MagY) | static_cast<std::uint16_t>(VescImuMask::MagZ) |
    static_cast<std::uint16_t>(VescImuMask::QuatW) |
    static_cast<std::uint16_t>(VescImuMask::QuatX) |
    static_cast<std::uint16_t>(VescImuMask::QuatY) | static_cast<std::uint16_t>(VescImuMask::QuatZ);

} // namespace goat_vesc
