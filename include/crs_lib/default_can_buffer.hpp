#pragma once

#include <cstring>
#include <type_traits>

#include <can_plugins/Frame.h>

#include "StewLib/serialize.hpp"

namespace CRSLib
{
    template<class RawData>
#ifdef __cpp_concepts
    requires (sizeof(RawData) <= 8) && (std::is_arithmetic_v<RawData> || std::is_enum_v<RawData>)
#endif
    struct DefaultCanBuffer final
    {
        RawData value{};

        bool push(const can_plugins::Frame::ConstPtr& frame_p) noexcept
        {
            std::memcpy(&value, &frame_p->data[0], sizeof(RawData));
            return true;
        }

        RawData get_value() const noexcept
        {
            return value;
        }

        void clear() const noexcept
        {}
    };
}