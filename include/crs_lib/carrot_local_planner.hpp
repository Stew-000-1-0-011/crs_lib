#pragma once

#include <shared_mutex>
#include <atomic>
#include <numbers>
#include <optional>

#include <ros/ros.h>
#include <crs_lib/Twist.h>
#include <crs_lib/Trajectory.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rosparam_util.hpp"

namespace CRSLib
{

    class CarrotLocalPlanner final
    {

        const struct RosparamData
        {
            std::string trajectory_topic_name;
            std::string cmd_vel_topic_name;
            std::string base_frame_id;
            std::string world_frame_id;
            double control_freq;
            double factor;
            double tolerance_radius2;
            double tolerance_angle2;

            RosparamData(ros::NodeHandle& nh)
            {
                 using namespace CRSLib::RosparamUtil;

                std::optional<StewXmlRpc> node_ns = get_param(nh, "carrot_local_planner");

                trajectory_topic_name = read_param<std::string>(node_ns, "trajectory_topic_name");
                assert_param(trajectory_topic_name, !trajectory_topic_name.empty(), "trajectory", "trajectory_topic_name");

                cmd_vel_topic_name = read_param<std::string>(node_ns, "cmd_vel_topic_name");
                assert_param(cmd_vel_topic_name, !cmd_vel_topic_name.empty(), "cmd_vel", "cmd_vel_topic_name");

                base_frame_id = read_param<std::string>(node_ns, "base_frame_id");
                assert_param(base_frame_id, !base_frame_id.empty(), "base", "base_frame_id");

                world_frame_id = read_param<std::string>(node_ns, "world_frame_id");
                assert_param(world_frame_id, !world_frame_id.empty(), "world", "world_frame_id");

                control_freq = read_param<double>(node_ns, "control_freq");
                assert_param(control_freq, control_freq > 0, 1000, "control_freq");

                factor = read_param<double>(node_ns, "factor");
                assert_param(factor, factor > 0, 1, "factor");

                tolerance_radius2 = read_param<double>(node_ns, "tolerance_radius");
                assert_param(tolerance_radius2, tolerance_radius2 >= 0, 0.1, "tolerance_radius");
                tolerance_radius2 *= tolerance_radius2;

                tolerance_angle2 = read_param<double>(node_ns, "tolerance_angle");
                assert_param(tolerance_angle2, tolerance_angle2 >= 0, 0.314, "tolerance_angle");
                tolerance_angle2 *= tolerance_angle2;
            }
        } rosparam_data;

        mutable std::shared_mutex mtx{};
        std::vector<geometry_msgs::Transform> trajectory{};
        std::vector<geometry_msgs::Transform>::const_iterator carrot_c_iter{};
        std::atomic<bool> is_goal{true};

        ros::Subscriber trajectory_sub;
        ros::Publisher cmd_vel_pub;
        tf2_ros::Buffer tf2buff{};
        tf2_ros::TransformListener tf2l{tf2buff};
        ros::Timer pub_tim;
    
    public:
        CarrotLocalPlanner(ros::NodeHandle& nh):
            rosparam_data{nh},
            trajectory_sub{nh.subscribe<crs_lib::Trajectory>(rosparam_data.trajectory_topic_name, 10, &CarrotLocalPlanner::trajectory_sub_callback, this)},
            cmd_vel_pub{nh.advertise<crs_lib::Twist>(rosparam_data.cmd_vel_topic_name, 1)},
            pub_tim{nh.createTimer(ros::Duration(1 / rosparam_data.control_freq), &CarrotLocalPlanner::calc, this)}
        {}

    private:
        void trajectory_sub_callback(const crs_lib::TrajectoryConstPtr& traj)
        {
            std::lock_guard lcd{mtx};
            trajectory = traj->trajectory;
            carrot_c_iter = trajectory.cbegin();
            is_goal = false;
        }

        std::optional<tf2::Transform> calc_delta() const
        {
             tf2::Transform target;
            {
                std::shared_lock slcd{mtx};
                tf2::fromMsg(*carrot_c_iter, target);
            }

            tf2::Transform base;
            try
            {
                tf2::fromMsg(tf2buff.lookupTransform(rosparam_data.base_frame_id, rosparam_data.world_frame_id, ros::Time(0)).transform, base);
            }
            catch(tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                return std::nullopt;
            }

            return target.inverse() * base;
        }

        void calc(const ros::TimerEvent&)
        {
            if(is_goal) return;

            while(true) if(const auto delta = calc_delta(); delta.has_value() && delta->getOrigin().length2() <= rosparam_data.tolerance_radius2 && delta->getRotation().length2() <= rosparam_data.tolerance_angle2)
            {
                std::lock_guard lck{mtx};
                if(carrot_c_iter++ == trajectory.cend())
                {
                    is_goal = true;
                    return;
                }
            }
            else
            {
                crs_lib::Twist twist;

                twist.linear_x = delta->getOrigin().getX();
                twist.linear_y = delta->getOrigin().getY();
                twist.angular_z = delta->getRotation().getAngle() - std::numbers::pi;

                cmd_vel_pub.publish(twist);
                break;
            }
        }
    };
}
