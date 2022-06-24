/*
文字列を送信するとかしたければ、こいつを特殊化してくれると嬉しい(/can_txを使うros::Publisherが被らないように)。
*/

#pragma once

#include <type_traits>
#include <memory>

#include "StewLib/judge_lower_cost_than_ref.hpp"
#include "StewLib/reverse_buffer.hpp"
#include "StewLib/serialize.hpp"

static_assert(sizeof(std::byte) == sizeof(std::uint8_t), "This program can't work correctly in this environment.");

namespace CRSLib
{
    template<class RawData>
    struct CanDataConvertor;

    template<class RawData>
#ifdef __cpp_concepts
    requires std::is_arithmetic_v<RawData> || std::is_enum_v<RawData>
#endif
    struct CanDataConvertor<RawData> final
    {
        static constexpr StewLib::Serialize<RawData, 8> convert(const StewLib::low_cost_ref_val_t<RawData> raw_data) noexcept
        {
            StewLib::ReverseBuffer<RawData> reverse_buffer = raw_data;
            reverse_buffer.reverse();

            // RawDataが算術型or列挙型だからできる。
            return {static_cast<RawData>(reverse_buffer)};
        }

        static constexpr RawData deconvert(const StewLib::low_cost_ref_val_t<StewLib::Serialize<RawData, 8>> serialized_data) noexcept
        {
            StewLib::ReverseBuffer<RawData> reverse_buffer;
            std::memcpy(reverse_buffer.buffer, serialized_data.chunks, sizeof(RawData));
            reverse_buffer.reversed();

            return {reverse_buffer};
        }
    };
}