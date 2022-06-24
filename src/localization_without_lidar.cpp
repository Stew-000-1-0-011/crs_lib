#include <crs_lib/localization_without_lidar.hpp>


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "localization_without_lidar");
    ros::NodeHandle nh{};  // グローバルに似た名前空間であること。

    CRSLib::LocalizationWithoutLidar instance{nh};

    ROS_INFO("Stew: localization_without_lidar node has started.");

    ros::spin();

    ROS_INFO("Stew: localization_without_lidar node has terminated.");

}

