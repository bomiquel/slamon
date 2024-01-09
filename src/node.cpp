#include <ros/ros.h>

#include "visual_odometry.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamon");
    ros::start();

    // Threads
    VisualOdometry visual_odometer;

    ROS_INFO("May the force be with you");

    // Launch threads
    std::thread visual_odometer_thread(&VisualOdometry::run, &visual_odometer);

    // ROS spin
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}