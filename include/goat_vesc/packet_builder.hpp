/**
 * @file packet_builder.hpp
 * @brief Helpers for serializing typed packet fields into VESC payload bytes.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <variant>
#include <vector>

namespace goat_vesc {

/**
 * @brief Field types accepted by `VescPacketBuilder`.
 *
 * The builder serializes each value in big-endian order and appends the result
 * into a payload buffer. Floating-point fields are handled by higher-level
 * protocol helpers before they reach this layer.
 */
using Field = std::variant<std::uint8_t, std::uint16_t, std::uint32_t, std::int8_t, std::int16_t,
                           std::int32_t>;

/**
 * @brief Serializes typed integral fields into an unframed VESC payload.
 *
 * This class is the lowest-level write-side helper in the library. It does not
 * add packet framing or CRC bytes; higher-level code is expected to wrap the
 * payload before it is written to the wire.
 */
class VescPacketBuilder {
private:
  std::vector<std::uint8_t> buffer_;

public:
  /** @brief Creates a builder with the default internal buffer size. */
  VescPacketBuilder();
  /**
   * @brief Creates a builder with a pre-sized internal buffer.
   * @param buffer_size Capacity reserved for the reusable payload buffer.
   */
  explicit VescPacketBuilder(std::size_t buffer_size);

  /**
   * @brief Builds an unframed payload from a vector of typed fields.
   * @param fields Field list to serialize in order.
   * @return Serialized payload bytes without VESC framing or CRC.
   */
  std::vector<std::uint8_t> build_packet(const std::vector<Field>& fields);
  /**
   * @brief Builds an unframed payload from an initializer list of typed fields.
   * @param fields Field list to serialize in order.
   * @return Serialized payload bytes without VESC framing or CRC.
   */
  std::vector<std::uint8_t> build_packet(std::initializer_list<Field> fields);
};

} // namespace goat_vesc
