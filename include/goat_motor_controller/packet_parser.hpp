/**
 * @file packet_parser.hpp
 * @brief Incremental parser for framed controller serial packets.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace goat_motor_controller {

/**
 * @brief Incrementally decodes framed controller packets from a byte stream.
 *
 * The parser owns an internal buffer and can be fed one byte at a time or in
 * batches. It validates framing and CRC before returning a payload. Because
 * length bytes are not escaped, a corrupted length may consume later frames
 * within one declared candidate before a subsequent frame restores
 * synchronization.
 */
class PacketParser {
public:
  /** @brief Framing-free controller packet payload. */
  using Payload = std::vector<std::uint8_t>;

  /** @brief Creates an empty parser with no buffered bytes. */
  PacketParser() = default;

  /** @brief Clears buffered state and restarts parsing from an empty stream. */
  void reset();

  /**
   * @brief Feeds one byte of transport data into the parser.
   * @param byte Raw byte read from the transport.
   * @return A decoded payload when a full valid packet is available.
   */
  std::optional<Payload> feed_byte(std::uint8_t byte);
  /**
   * @brief Feeds a batch of transport bytes into the parser.
   * @param bytes Raw bytes read from the transport.
   * @return Every decoded payload completed while consuming the batch.
   */
  std::vector<Payload> feed_bytes(const std::vector<std::uint8_t>& bytes);

private:
  std::vector<std::uint8_t> buffer_;

  enum class DecodeResult { Success, NeedMoreData, InvalidHeader, InvalidPacket };

  DecodeResult try_decode_packet_(Payload& payload_out, std::size_t& packet_size_out) const;
  static std::uint16_t crc16ccitt_(const std::vector<std::uint8_t>& data) noexcept;
};

} // namespace goat_motor_controller
