/*
必要になったら増やす
*/

#pragma once

#include <cstdint>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>



namespace CRSLib
{
    namespace Implement::StdMsgsTypeConvertorImp
        {
        template<class RawData>
        struct StdMsgsTypeConvertor;

#define Stew_define_StdMsgsTypeConvertor(RawData, Message) \
    template<> \
    struct StdMsgsTypeConvertor<RawData> final {using type = std_msgs::Message;};

    Stew_define_StdMsgsTypeConvertor(bool, Bool)
    Stew_define_StdMsgsTypeConvertor(float, Float32)
    Stew_define_StdMsgsTypeConvertor(std::uint8_t, UInt8)

#undef Stew_define_StdMsgsTypeConvertor
    }

    template<class RawData>
    using StdMsgsTypeConvertor = Implement::StdMsgsTypeConvertorImp::StdMsgsTypeConvertor<RawData>::type;
}