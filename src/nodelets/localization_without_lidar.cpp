#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <crs_lib/localization_without_lidar.hpp>

namespace CRSLib
{
    namespace
    {
        class NodeletLocalizationWithoutLidar final : public nodelet::Nodelet
        {
            std::unique_ptr<LocalizationWithoutLidar> instance_up{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                instance_up = std::make_unique<LocalizationWithoutLidar>(nh);
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(CRSLib::NodeletLocalizationWithoutLidar, nodelet::Nodelet)
