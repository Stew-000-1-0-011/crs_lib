#pragma once

#include <vector>
#include <array>
#include <queue>
#include <shared_mutex>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "rosparam_util.hpp"
#include "StewLib/circular_buffer.hpp"

namespace CRSLib
{
    namespace
    {
        template<class KeyMap>
        class JoyToKeyButton final
        {
        public:
            using Axes = KeyMap::Axes;
            using Buttons = KeyMap::Buttons;

        private:
            mutable std::shared_mutex shared_mtx{};

            std::array<float, Axes::N> axes{};
            std::array<bool, Buttons::N> buttons{};
            std::array<std::queue<ros::Time>, Buttons::N> buttons_up{};
            std::array<std::queue<ros::Time>, Buttons::N> buttons_down{};

            ros::Subscriber joy_sub{};

        public:
            // ros::Subscriberの重複は一切検知しないよ。
            JoyToKeyButton(ros::NodeHandle& nh, const char *const joy_topic_name, double frame_rate = 60) noexcept
            {
                if(frame_rate <= 0)
                {
                    frame_rate = 60;
                    ROS_ERROR(
                        "Stew: JoyToKeyButton::JoyToKeyButton(ros::NodeHandle&, const double):"
                        "frame_rate must be positive. The value is set by default 60."
                    );
                }

                joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_topic_name, 1, &JoyToKeyButton::sub_update, this);
            }

            bool is_being_pushed(const Buttons::Enum button) const noexcept
            {
                std::shared_lock lock{shared_mtx};

                return buttons[button];
                // 戻り値のbool型の値が構築されてからlockのデストラクタが呼ばれる(はず)
            }

            /// TODO: 最後の状態が残ってしまっている。
            bool is_pushed_up(const Buttons::Enum button, const ros::Duration& interval = ros::Duration()) noexcept
            {
                std::shared_lock lock{shared_mtx};

                while(interval.isZero() || !buttons_up[button].empty())
                {
                    if(ros::Time::now() - buttons_up[button].front() < interval)
                    {
                        buttons_up[button].pop();
                        return true;
                    }
                    buttons_up[button].pop();
                }

                return false;
            }

            bool is_pushed_down(const Buttons::Enum button, const ros::Duration& interval = ros::Duration()) noexcept
            {
                std::shared_lock lock{shared_mtx};

                while(!buttons_down[button].empty())
                {
                    if(interval.isZero() || ros::Time::now() - buttons_down[button].front() < interval)
                    {
                        buttons_down[button].pop();
                        return true;
                    }
                    buttons_down[button].pop();
                }
                
                return false;
            }

            float get_axe(const Axes::Enum axe) const noexcept
            {
                std::shared_lock lock{shared_mtx};

                return axes[axe];
            }

            auto& get_axes() const noexcept
            {
                std::shared_lock lock{shared_mtx};

                return axes;
            }

        private:
            void sub_update(const sensor_msgs::Joy::ConstPtr& joy_p) noexcept
            {
                std::lock_guard lock{shared_mtx};

                {
                    bool flag{};
                    if(joy_p->axes.size() != Axes::N)
                    {
                        ROS_WARN("size of axes is differ from Axes::N.");
                        flag = true;
                    }
                    if(joy_p->buttons.size() != Buttons::N)
                    {
                        ROS_WARN("size of buttons is differ from Buttons::N.");
                        flag = true;
                    }
                    if(flag) return;
                }
                
                for(std::underlying_type_t<decltype(Buttons::N)> i = 0; i < Axes::N; ++i)
                {
                    axes[i] = joy_p->axes[i];
                }

                for(std::underlying_type_t<decltype(Buttons::N)> i = 0; i < Buttons::N; ++i)
                {
                    const bool latest_button = joy_p->buttons[i];

                    if(!latest_button && buttons[i]) buttons_up[i].push(joy_p->header.stamp);
                    if(latest_button && !buttons[i]) buttons_down[i].push(joy_p->header.stamp);
                    buttons[i] = latest_button;
                }
            }
        };
    }
}