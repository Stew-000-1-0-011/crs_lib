#pragma once

#include <cstdint>
#include <tuple>
#include <utility>
#include <string>

#include <ros/ros.h>

#include <can_plugins/Frame.h>

#include "can_subscriber.hpp"

namespace CRSLib
{
    namespace
    {
        namespace Implement::CanSubscriberManager
        {
            template<class T>
            struct Index
            {
                std::uint8_t index;
            };
        }

        template<class ... CanSubscribers>
#ifdef __cpp_concepts
        requires (0 < sizeof...(CanSubscribers)) && (sizeof...(CanSubscribers) < 256) && (... && Concept::CanSubscriber<CanSubscribers>)
#endif
        class CanSubscriberManager
        {
            template<class T>
            struct TypeCalc;

            template<std::uint8_t ... indexs>
            struct TypeCalc<std::integer_sequence<std::uint8_t, indexs ...>>
            {
                static constexpr std::tuple<Implement::CanSubscriberManager::Index<typename CanSubscribers::Tag> ...> tags{indexs ...};
            };

            using TagTuple = TypeCalc<std::make_integer_sequence<std::uint8_t, sizeof...(CanSubscribers)>>;

            static constexpr std::uint8_t size = sizeof...(CanSubscribers);
            inline static ros::Subscriber sub;

            std::tuple<CanSubscribers ...> can_subscribers;

        public:
            CanSubscriberManager(ros::NodeHandle& nh) noexcept:
                can_subscribers{}
            {
                if(!sub)
                {
                    // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                    sub = nh.subscribe<can_plugins::Frame>("can_rx", 1000, &CanSubscriberManager::callback, this);
                    /// DEBUG:
                    ROS_INFO("Stew: can_subscriber is initialized.");
                }
                else
                {
                    // コールバックがあるのでCanSubscirberは一度しか初期化されない。
                    ROS_ERROR("Stew: can_subscriber has already been initialized.");
                }
            }

            CanSubscriberManager(ros::NodeHandle& nh, const CanSubscribers& ... can_subscribers) noexcept:
                can_subscribers{can_subscribers ...}
            {
                if(!sub)
                {
                    // バッファはいい感じに設定してね(実行時に変更するの最初きりしかうまくできんかったので、潔く直値で初期化)
                    sub = nh.subscribe<can_plugins::Frame>("can_rx", 1000, &CanSubscriberManager::callback, this);
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
                }(std::make_integer_sequence<std::uint8_t, sizeof...(CanSubscribers)>());
            }

            template<class Tag>
            auto& get() noexcept
            {
                constexpr std::uint8_t index = std::get<Implement::CanSubscriberManager::Index<Tag>>(TagTuple::tags).index;
                return std::get<index>(can_subscribers);
            }

            template<std::uint8_t index>
            auto& get() noexcept
            {
                return std::get<index>(can_subscribers);
            }
        };
    }
}