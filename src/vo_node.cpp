#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "visual_odometry.hpp"


void readParams(int& min_inliers,
                double& lowe_ratio, 
                bool& enable_debugging,
                std::string& working_path,
                std::string& matching_type,
                std::string& robot_frame_id,
                std::string& origin_frame_id,
                std::string& motion_estimation_type)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nhp("~");
    nhp.param("min_inliers",            min_inliers,            10);
    nhp.param("lowe_ratio",             lowe_ratio,             0.8);
    nhp.param("working_path",           working_path,           ros::package::getPath("slamon") + "/output/");
    nhp.param("matching_type",          matching_type,          std::string("ratio_matching"));
    nhp.param("robot_frame_id",         robot_frame_id,         std::string("base_link"));
    nhp.param("origin_frame_id",        origin_frame_id,        std::string("odom"));
    nhp.param("enable_debugging",       enable_debugging,       true);
    nhp.param("motion_estimation_type", motion_estimation_type, std::string("p3p"));

    // Printing parameters values.
    ROS_INFO_STREAM("Parameter setting: " << std::endl <<
                    "Minimum kpts           = " << min_inliers << std::endl << 
                    "Lowe's ratio           = " << lowe_ratio << std::endl <<
                    "Working path           = " << working_path << std::endl <<
                    "Matching type          = " << matching_type << std::endl <<
                    "Robot frame id         = " << robot_frame_id << std::endl <<
                    "Odom frame id          = " << origin_frame_id << std::endl <<
                    "Enable debugging       = " << enable_debugging << std::endl <<
                    "Motion estimation type = " << motion_estimation_type);

    // Checking the validity of the parameters
    bool invalid_parameter = false;
    if (min_inliers < 10)
    {
        min_inliers = 10;
        invalid_parameter = true;
    }
    if (lowe_ratio < 0.01 || lowe_ratio > 0.99)
    {
        lowe_ratio = 0.85;
        invalid_parameter = true;
    }
    if (working_path == "")
    {
        working_path = ros::package::getPath("slamon") + "/output/";
        invalid_parameter = true;
    }
    if (working_path.back() != '/')
    {
        working_path += "/";
        invalid_parameter = true;
    }
    if (robot_frame_id == "")
    {
        robot_frame_id = "base_link";
        invalid_parameter = true;
    }
    if (origin_frame_id == "")
    {
        origin_frame_id = "odom";
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Minimum kpts           = " << min_inliers << std::endl << 
                        "Lowe's ratio           = " << lowe_ratio << std::endl <<
                        "Working path           = " << working_path << std::endl <<
                        "Matching type          = " << matching_type << std::endl <<
                        "Robot frame id         = " << robot_frame_id << std::endl <<
                        "World frame id         = " << origin_frame_id << std::endl <<
                        "Enable debugging       = " << enable_debugging << std::endl <<
                        "Motion estimation type = " << motion_estimation_type);
    }
}
    

int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "vo_slamon");
    ros::start();

    // Read parameters.
    int min_inliers;
    double lowe_ratio;
    bool enable_debugging;
    std::string working_path, matching_type, robot_frame_id, origin_frame_id, motion_estimation_type;
    readParams(min_inliers, lowe_ratio, enable_debugging, working_path, matching_type,
               robot_frame_id, origin_frame_id, motion_estimation_type);

    // Declare threads.
    slam::VisualOdometry visual_odometer(lowe_ratio, 
                                   min_inliers, 
                                   enable_debugging, 
                                   working_path, 
                                   matching_type, 
                                   motion_estimation_type,
                                   robot_frame_id, 
                                   origin_frame_id);

    // Launch threads.
    std::thread visual_odometer_thread(&slam::VisualOdometry::run, &visual_odometer);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}