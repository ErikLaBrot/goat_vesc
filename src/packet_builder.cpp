#include "goat_vesc/packet_builder.hpp"

namespace goat_vesc {

VescPacketBuilder::VescPacketBuilder() = default;

VescPacketBuilder::VescPacketBuilder(std::size_t buffer_size) {
  buffer_.reserve(buffer_size);
}

template <typename T> void append_integral_be(std::vector<std::uint8_t>& buffer, T value) {
  static_assert(std::is_integral_v<T>, "append_integral_be requires an integral type");

  using U = std::make_unsigned_t<T>;
  U raw = static_cast<U>(value);

  for (int i = sizeof(U) - 1; i >= 0; --i) {
    buffer.push_back(static_cast<std::uint8_t>((raw >> (8 * i)) & 0xFF));
  }
}

std::vector<std::uint8_t> VescPacketBuilder::build_packet(const std::vector<Field>& fields) {
  buffer_.clear();

  for (const auto& field : fields) {
    std::visit([this](const auto& arg) { append_integral_be(buffer_, arg); }, field);
  }

  return buffer_;
}

std::vector<std::uint8_t> VescPacketBuilder::build_packet(std::initializer_list<Field> fields) {
  return build_packet(std::vector<Field>(fields));
}

} // namespace goat_vesc
