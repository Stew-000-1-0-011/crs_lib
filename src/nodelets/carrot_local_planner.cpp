#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <crs_lib/carrot_local_planner.hpp>

namespace CRSLib
{
    namespace
    {
        class NodeletCarrotLocalPlanner final : public nodelet::Nodelet
        {
            std::unique_ptr<CarrotLocalPlanner> instance_up{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                instance_up = std::make_unique<CarrotLocalPlanner>(nh);
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(CRSLib::NodeletCarrotLocalPlanner, nodelet::Nodelet)
