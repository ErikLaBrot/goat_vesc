#pragma once

#include <vector>
#include <cstdint>
#include <variant>
#include <initializer_list>
#include <type_traits>

namespace goat_vesc {

using Field = std::variant<
    std::uint8_t,
    std::uint16_t,
    std::uint32_t,
    std::int8_t,
    std::int16_t,
    std::int32_t
>;

class VescPacketBuilder {
private:
    std::vector<std::uint8_t> buffer_;
public:
    VescPacketBuilder();
    explicit VescPacketBuilder(std::size_t buffer_size);

    std::vector<std::uint8_t> build_packet(const std::vector<Field>& fields);
    std::vector<std::uint8_t> build_packet(std::initializer_list<Field> fields);
};

} // namespace goat_vesc
