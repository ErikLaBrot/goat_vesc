/**
 * @file controller_protocol.hpp
 * @brief Typed request builders and response parsers for supported controller messages.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "goat_motor_controller/protocol_ids.hpp"
#include "goat_motor_controller/types.hpp"

namespace goat_motor_controller {

/**
 * @brief Builds supported controller requests and parses supported replies.
 *
 * `ControllerProtocol` sits above packet framing primitives and below the transport
 * owner. It knows how to encode and decode the message layouts used by the
 * current public client API.
 *
 * Control builders return an empty packet when a floating-point command is
 * non-finite, outside its documented normalized range, or not representable on
 * the wire.
 */
class ControllerProtocol {
public:
  /** @brief Unframed controller payload bytes. */
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
  /** @brief Builds a request for the active motor configuration. */
  static Payload build_get_motor_config_request();
  /** @brief Builds a request that persists a motor configuration image. */
  static Payload build_set_motor_config_request(const MotorConfigImage& image);
  /** @brief Builds a request for the active application configuration. */
  static Payload build_get_app_config_request();
  /** @brief Builds a request that applies an application configuration image. */
  static Payload build_set_app_config_request(const AppConfigImage& image,
                                              AppConfigStorage storage);
  /** @brief Builds a request for a chunk of stored LispBM code. */
  static Payload build_lisp_read_request(std::uint32_t length, std::uint32_t offset);
  /** @brief Builds a request to erase the LispBM code partition. */
  static Payload build_lisp_erase_request(std::uint32_t size);
  /** @brief Builds a request to write one packed LispBM code chunk. */
  static Payload build_lisp_write_request(const Payload& chunk, std::uint32_t offset);
  /** @brief Builds a request to start or stop LispBM execution. */
  static Payload build_lisp_set_running_request(bool running);
  /** @brief Builds a `COMM_CUSTOM_APP_DATA` request for the running custom app. */
  static Payload build_custom_app_data_request(const Payload& data);
  /** @brief Builds the firmware-7.00 local FOC detection-and-apply request. */
  static Payload build_foc_calibration_request(const FocCalibrationParameters& parameters);
  /** @brief Builds a fire-and-forget application-output suppression command. */
  static Payload build_app_disable_output_command(std::chrono::milliseconds duration);
  /** @brief Adds the firmware storage header, CRC, and zero flags to LispBM code. */
  static Payload pack_lisp_code(const LispCodeImage& image);
  /**
   * @brief Builds a `COMM_SET_RPM` command packet.
   * @param rpm Target RPM value.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_rpm_command(std::int32_t rpm);
  /**
   * @brief Builds a `COMM_SET_DUTY` command packet.
   * @param duty Duty-cycle request in the `[-1.0, 1.0]` range.
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
   * @param amps Signed brake-current command in amps.
   * @return Fully framed packet ready to write to the transport.
   */
  static Payload build_set_current_brake_command(float amps);
  /**
   * @brief Builds a `COMM_SET_SERVO_POS` command packet.
   * @param position Servo position in the `[0.0, 1.0]` range.
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
  static std::optional<MotorState> parse_get_values(const Payload& payload);
  /**
   * @brief Parses a `COMM_GET_IMU_DATA` payload.
   * @param payload Raw payload with framing and CRC already stripped.
   * @return Parsed IMU sample or `std::nullopt` if malformed.
   */
  static std::optional<ImuData> parse_get_imu_data(const Payload& payload);
  /** @brief Parses and strips a motor-configuration response ID. */
  static std::optional<MotorConfigImage> parse_motor_config(const Payload& payload);
  /** @brief Parses and strips an application-configuration response ID. */
  static std::optional<AppConfigImage> parse_app_config(const Payload& payload);
  /**
   * @brief Parses a LispBM code chunk and returns its data bytes.
   *
   * @param payload Raw reply payload.
   * @param total_size Receives the complete code-image size.
   * @param offset Receives this chunk's offset.
   */
  static std::optional<Payload> parse_lisp_read_reply(const Payload& payload,
                                                      std::uint32_t& total_size,
                                                      std::uint32_t& offset);
  /** @brief Validates a one-byte configuration acknowledgement. */
  static bool parse_config_ack(const Payload& payload, CommandId expected_id);
  /** @brief Validates an accepted LispBM write acknowledgement and offset. */
  static bool parse_lisp_write_ack(const Payload& payload, std::uint32_t expected_offset);
  /** @brief Validates an accepted ID-plus-boolean acknowledgement. */
  static bool parse_bool_ack(const Payload& payload, CommandId expected_id);
  /** @brief Parses and strips a `COMM_CUSTOM_APP_DATA` reply ID. */
  static std::optional<Payload> parse_custom_app_data_reply(const Payload& payload);
  /** @brief Parses the signed firmware result from a FOC calibration reply. */
  static std::optional<std::int16_t> parse_foc_calibration_reply(const Payload& payload);

private:
  // Wraps a payload in the firmware framing: [start | len... | payload | crc | stop]
  static Payload frame(const Payload& payload);
  static std::uint16_t crc16ccitt(const Payload& data) noexcept;
};

} // namespace goat_motor_controller
