#pragma once

#include <cstddef>
#include <vector>
#include <array>
#include <queue>
#include <shared_mutex>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace CRSLib
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
        const std::size_t button_queue_size;

        ros::Subscriber joy_sub{};

    public:
        // ros::Subscriberの重複は一切検知しないよ。
        JoyToKeyButton(ros::NodeHandle& nh, const char *const joy_topic_name, const std::size_t button_queue_size = 1):
            button_queue_size{button_queue_size}
        {
            joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_topic_name, 1, &JoyToKeyButton::sub_update, this);
        }

        bool is_being_pushed(const Buttons::Enum button) const
        {
            std::shared_lock lock{shared_mtx};

            return buttons[button];
            // 戻り値のbool型の値が構築されてからlockのデストラクタが呼ばれる(はず)
        }

        bool is_pushed_up(const Buttons::Enum button, const ros::Duration& interval = ros::Duration())
        {
            std::shared_lock lock{shared_mtx};

            while(!buttons_up[button].empty())
            {
                if(interval.isZero() || ros::Time::now() - buttons_up[button].front() < interval)
                {
                    buttons_up[button].pop();
                    return true;
                }
                buttons_up[button].pop();
            }

            return false;
        }

        bool is_pushed_down(const Buttons::Enum button, const ros::Duration& interval = ros::Duration())
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

        float get_axis(const Axes::Enum axe) const
        {
            std::shared_lock lock{shared_mtx};

            return axes[axe];
        }

        auto& get_axes() const
        {
            std::shared_lock lock{shared_mtx};

            return axes;
        }

    private:
        void sub_update(const sensor_msgs::Joy::ConstPtr& joy_p)
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

                if(!latest_button && buttons[i])
                {
                    if(buttons_up[i].size() > button_queue_size) buttons_up[i].pop();
                    buttons_up[i].push(joy_p->header.stamp);
                }
                if(latest_button && !buttons[i])
                {
                    if(buttons_down[i].size() > button_queue_size) buttons_down[i].pop();
                    buttons_down[i].push(joy_p->header.stamp);
                }
                buttons[i] = latest_button;
            }
        }
    };
}