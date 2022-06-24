#pragma once

#include <cstdint>
#include <tuple>
#include <utility>
#include <string>
#include <concepts>

#include <ros/ros.h>

#include <can_plugins/Frame.h>

#include <StewLib/type_holder.hpp>
#include "can_subscriber.hpp"

namespace CRSLib
{
    // namespace Implement::CanSubscriberManagerImp
    // {
    //     template<class T>
    //     struct Index
    //     {
    //         std::uint8_t index;
    //     };
    // }

    template<class ... CanSubscribers>
#ifdef __cpp_concepts
    requires (0 < sizeof...(CanSubscribers)) && (sizeof...(CanSubscribers) < 256) && (... && IsCanSubscriber<CanSubscribers>)
#endif
    class CanSubscriberManager
    {
        // template<class T>
        // struct TypeCalc;

        // template<std::uint8_t ... indexs>
        // struct TypeCalc<std::integer_sequence<std::uint8_t, indexs ...>>
        // {
        //     static constexpr std::tuple<Implement::CanSubscriberManagerImp::Index<typename CanSubscribers::Tag> ...> tags{indexs ...};
        // };

        // using TagTuple = TypeCalc<std::make_integer_sequence<std::uint8_t, sizeof...(CanSubscribers)>>;

        using Tags = StewLib::TypeHolder<typename CanSubscribers::Tag ...>;

        static constexpr std::uint8_t size = sizeof...(CanSubscribers);
        inline static ros::Subscriber sub;

        std::tuple<CanSubscribers ...> can_subscribers;

    public:

        CanSubscriberManager(ros::NodeHandle& nh, const CanSubscribers& ... can_subscribers) noexcept:
            can_subscribers{can_subscribers ...}
        {
            if(!sub)
            {
                // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                // こいつからのコールバックが並列に実行されるようにする
                ros::SubscribeOptions ops;
                ops.template init<can_plugins::Frame>("can_rx", 1000, boost::bind(&CanSubscriberManager::callback, this, _1));
                ops.transport_hints = ros::TransportHints();
                ops.allow_concurrent_callbacks = true;
                ros::Subscriber sub = nh.subscribe(ops);

                // sub = nh.subscribe<can_plugins::Frame>("can_rx", 1000, &CanSubscriberManager::callback, this);

                /// DEBUG:
                ROS_INFO("Stew: can_subscriber is initialized.");
            }
            else
            {
                // コールバックがあるのでCanSubscirberは一度しか初期化されない。
                ROS_ERROR("Stew: can_subscriber has already been initialized.");
            }
        }

        void callback(const can_plugins::Frame::ConstPtr& frame_p) noexcept
        {
            [this, &frame_p]<uint8_t ... indexs>(const std::integer_sequence<std::uint8_t, indexs ...>&)
            {
                (... , std::get<indexs>(can_subscribers).push_and_call(frame_p));
            }(std::make_integer_sequence<std::uint8_t, size>());
        }

        template<class Tag>
        auto& get() noexcept
        {
            static constexpr auto pred = []<class T>()
            {
                return std::same_as<T, Tag>;
            };

            return std::get<Tags::template get_index<pred>()>(can_subscribers);
        }

        template<std::uint8_t index>
        auto& get() noexcept
        {
            return std::get<index>(can_subscribers);
        }
    };
}