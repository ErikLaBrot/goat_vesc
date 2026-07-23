/**
 * @file vesc_protocol.hpp
 * @brief Typed request builders and response parsers for supported VESC messages.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "goat_vesc/protocol_ids.hpp"
#include "goat_vesc/types.hpp"

namespace goat_vesc {

/**
 * @brief Builds supported VESC requests and parses supported VESC replies.
 *
 * `VescProtocol` sits above packet framing primitives and below the transport
 * owner. It knows how to encode and decode the message layouts used by the
 * current public client API.
 */
class VescProtocol {
public:
  /** @brief Unframed VESC payload bytes. */
  using Payload = std::vector<std::uint8_t>;

  /**
   * @brief Builds a `COMM_FW_VERSION` request packet.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_fw_version_request();
  /**
   * @brief Builds a `COMM_GET_VALUES` request packet.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_get_values_request();
  /**
   * @brief Builds a `COMM_GET_IMU_DATA` request packet.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_get_imu_data_request();
  /**
   * @brief Builds a `COMM_SET_RPM` command packet.
   * @param rpm Target RPM value.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_rpm_command(std::int32_t rpm);
  /**
   * @brief Builds a `COMM_SET_DUTY` command packet.
   * @param duty Duty-cycle request, typically in the `[-1.0, 1.0]` range.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_duty_command(float duty);
  /**
   * @brief Builds a `COMM_SET_CURRENT` command packet.
   * @param amps Target motor current in amps.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_current_command(float amps);
  /**
   * @brief Builds a `COMM_SET_CURRENT_BRAKE` command packet.
   * @param amps Active brake-current magnitude in amps.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_current_brake_command(float amps);
  /**
   * @brief Builds a `COMM_SET_SERVO_POS` command packet.
   * @param position Servo position in controller-specific normalized units.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_servo_pos_command(float position);

  /**
   * @brief Parses a `COMM_FW_VERSION` payload.
   * @param payload Raw payload with framing and CRC already stripped.
   * @return Parsed firmware version or `std::nullopt` if malformed.
   */
  static std::optional<FwVersion> parse_fw_version(const Payload& payload);
  /**
   * @brief Parses a `COMM_GET_VALUES` payload.
   * @param payload Raw payload with framing and CRC already stripped.
   * @return Parsed motor-state sample or `std::nullopt` if malformed.
   */
  static std::optional<VescMotorState> parse_get_values(const Payload& payload);
  /**
   * @brief Parses a `COMM_GET_IMU_DATA` payload.
   * @param payload Raw payload with framing and CRC already stripped.
   * @return Parsed IMU sample or `std::nullopt` if malformed.
   */
  static std::optional<VescIMUData> parse_get_imu_data(const Payload& payload);

private:
  // Wraps a payload in the VESC framing: [start | len... | payload | crc | stop]
  static Payload frame(const Payload& payload);
  static std::uint16_t crc16ccitt(const Payload& data) noexcept;
};

} // namespace goat_vesc
