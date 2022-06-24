// MPU9250は16bitの整数値で各種データを吐き出すらしい？ならとりあえず変換部は後で書くこととして、とりあえず仮置きで書くか。

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <shared_mutex>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <can_plugins/Frame.h>

#include "can_subscriber.hpp"
#include "can_subscriber_manager.hpp"
#include "rosparam_util.hpp"

#include <StewLib/static_warn.hpp>


namespace CRSLib
{
    namespace Implement::LocalizationWithoutLidarImp
    {
        struct OdomRawData
        {
            std::uint16_t rot_x{};
            std::uint16_t rot_y{};
            std::uint16_t rot_z{};
            std::uint16_t acc_x{};
            std::uint16_t acc_y{};
            std::uint16_t acc_z{};
            std::uint16_t rotary_vel_x{};
            std::uint16_t rotary_vel_y{};
        };

        struct OdomData
        {
            float rot_x{};
            float rot_y{};
            float rot_z{};
            float acc_x{};
            float acc_y{};
            float acc_z{};
            float rotary_vel_x{};
            float rotary_vel_y{};
        };

        struct OdomBuffer
        {
            std::uint16_t buff[8]{};
            OdomRawData current_odom_raw_data{};
            std::int8_t index{};

            bool push(const can_plugins::FrameConstPtr& frame_p) noexcept
            {
                if(const std::uint8_t index_ = std::bit_cast<std::uint8_t>(frame_p->data[0]); index == index_)
                {
                    // pointer_interconvertibleは関係ない(はず)。何の変更も行われないだけのはず。アラインメントもfloatよりか大きいだろうし。
                    std::memcpy(buff + 4 * index, &frame_p->data[1], sizeof(std::uint16_t) * 4);
                    if(index == 1)
                    {
                        index = 0;
                        std::memcpy(&current_odom_raw_data, buff, sizeof(std::uint16_t) * 8);
                        return true;
                    }
                    else
                    {
                        ++index;
                        return false;
                    }
                }
                else  // データが欠落した場合
                {
                    index = 0;
                    return false;
                }
            }

            OdomData get_value() const noexcept
            {
                // 未実装
                Stew_static_warn(false, "I have not written how to calc these data.");
                return OdomData{};
            }

            void clear() noexcept
            {
                index = 0;
            }
        };

        struct LocalizationCallback
        {
            std::shared_mutex mtx{};
            geometry_msgs::Transform odom_transform{};

            LocalizationCallback(const LocalizationCallback&)
            {}

            LocalizationCallback& operator =(const LocalizationCallback&)
            {
                return *this;
            }

            LocalizationCallback() = default;
            LocalizationCallback(LocalizationCallback&&) = delete;
            LocalizationCallback& operator =(LocalizationCallback&&) = delete;
            ~LocalizationCallback() = default;
            
            void callback(const OdomData& data) noexcept
            {
                // 未実装
                Stew_static_warn(false, "I have not written how to calc self position.");
            }

            geometry_msgs::Transform get_odom_transform() noexcept
            {
                std::shared_lock slck{mtx};
                return odom_transform;
            }
        };
    }

    class LocalizationWithoutLidar final
    {
        const struct RosparamData
        {
            std::string odom_frame_id;
            std::string world_frame_id;
            double broadcast_freq;

            RosparamData(ros::NodeHandle& nh)
            {
                 using namespace CRSLib::RosparamUtil;

                std::optional<StewXmlRpc> node_ns = get_param(nh, "localization_without_lidar");

                odom_frame_id = read_param<std::string>(node_ns, "odom_frame_id");
                assert_param(odom_frame_id, odom_frame_id.empty(), "odom", "odom_frame_id");

                world_frame_id = read_param<std::string>(node_ns, "world_frame_id");
                assert_param(world_frame_id, world_frame_id.empty(), "world", "world_frame_id");

                broadcast_freq = read_param<double>(node_ns, "broadcast_freq");
                assert_param(broadcast_freq, broadcast_freq > 0, 1000, "broadcast_freq");
            }
        } rosparam_data;

        tf2_ros::TransformBroadcaster tfb{};

        using OdomCanSub = CanSubscriber<struct ReceiveOdomRawData, Implement::LocalizationWithoutLidarImp::LocalizationCallback, Implement::LocalizationWithoutLidarImp::OdomBuffer>;
        CanSubscriberManager<OdomCanSub> can_subscriber_manager;

        ros::Timer broadcast_timer;

    public:
        LocalizationWithoutLidar(ros::NodeHandle& nh):
            rosparam_data{nh},
            // can_idは適当。
            can_subscriber_manager{nh, OdomCanSub{0x30, {}, {}}},
            broadcast_timer{nh.createTimer(ros::Duration(1 / rosparam_data.broadcast_freq), &LocalizationWithoutLidar::broadcast_callback, this)}
        {}

    private:
        void broadcast_callback(const ros::TimerEvent&)
        {
            geometry_msgs::TransformStamped transform_stamped;

            transform_stamped.header.frame_id = rosparam_data.world_frame_id;
            transform_stamped.child_frame_id = rosparam_data.odom_frame_id;

            // geometry_msgs::Transform odom_transform = can_subscriber_manager.template get<ReceiveOdomRawData>().callback.get_odom_transform();
            transform_stamped.header.stamp = ros::Time::now();
            // transform_stamped.transform = odom_transform;

            tfb.sendTransform(transform_stamped);
        }
    };
}