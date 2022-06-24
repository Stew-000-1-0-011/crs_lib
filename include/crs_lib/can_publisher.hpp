#pragma once

#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_plugins/Frame.h>

#include "StewLib/judge_lower_cost_than_ref.hpp"
#include "StewLib/serialize.hpp"
#include "StewLib/reverse_buffer.hpp"

#include "std_msgs_type_convertor.hpp"
#include "can_data_convertor.hpp"

namespace CRSLib
{
    class CanPublisher final
    {
        inline static ros::Publisher pub{};
        inline static ros::Publisher non_can_pub{};
    public:
        CanPublisher() = default;
        
        CanPublisher(ros::NodeHandle& nh) noexcept
        {
            if(!pub)
            {
                // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                pub = nh.advertise<can_plugins::Frame>("can_tx", 1000);
                /// DEBUG:
                ROS_INFO("Stew: can_publisher.pub has initialized.");
            }
            if(!non_can_pub)
            {
                // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                non_can_pub = nh.advertise<std_msgs::String>("crs_lib/debug/can_tx", 1000);
                /// DEBUG:
                ROS_INFO("Stew: can_publisher.non_can_pub has initialized.");
            }
        }

        template<class RawData>
        void can_publish(const std::uint32_t can_id, const StewLib::low_cost_ref_val_t<RawData> raw_data) const noexcept
        {
            const auto serialized_data = CanDataConvertor<RawData>::convert(raw_data);
            using SerializeType = decltype(serialized_data);

            can_plugins::FramePtr frame_msg_ptr(new can_plugins::Frame);
            frame_msg_ptr->id = can_id;
            frame_msg_ptr->is_rtr = false;
            frame_msg_ptr->is_extended = false;
            frame_msg_ptr->is_error = false;

            if constexpr(SerializeType::chunks_size > 1)
            {
                frame_msg_ptr->dlc = 8;
                const auto data_p = frame_msg_ptr->data.c_array();

                for(std::size_t i = 0; i < SerializeType::chunks_size - 1; ++i)
                {
                    std::memcpy(data_p, serialized_data.chunks[i], 8);

                    pub.publish(frame_msg_ptr);
                }

                frame_msg_ptr->dlc = SerializeType::last_size;
                std::memcpy(data_p, serialized_data.chunks[SerializeType::chunks_size - 1], SerializeType::last_size);
            }
            else
            {
                frame_msg_ptr->dlc = SerializeType::last_size;
                std::memcpy(frame_msg_ptr->data.c_array(), serialized_data.chunks[0], SerializeType::last_size);
            }
            
            pub.publish(frame_msg_ptr);
        }

        // なんでdeprecatedにしたのか思い出せなかったので外した。
        template<class RawData>
        void publish(const std::uint32_t can_id, const StewLib::low_cost_ref_val_t<RawData> raw_data) const noexcept
        {
            std_msgs::String msg;
            msg.data = std::to_string(can_id) + ":" + std::to_string(raw_data);
            non_can_pub.publish(msg);
        }
    };
}