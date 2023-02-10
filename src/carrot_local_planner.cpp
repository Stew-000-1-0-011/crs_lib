#include <crs_lib/carrot_local_planner.hpp>


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "carrot_local_planner");
    ros::NodeHandle nh{};  // グローバルに似た名前空間であること。

    CRSLib::CarrotLocalPlanner instance{nh};

    ROS_INFO("Stew: carrot_local_planner node has started.");

    ros::spin();

    ROS_INFO("Stew: carrot_local_planner node has terminated.");

}

