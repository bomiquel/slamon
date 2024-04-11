#include <vector>
#include <string>

#include <ros/ros.h>

#include "visual_odometry.hpp"


void readParams(int& min_num_kps,
                double& lowe_ratio, 
                bool& enable_debugging,
                std::string& matching_type,
                std::string& odom_frame_id,
                std::string& robot_frame_id)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nhp("~");
    nhp.param("min_num_kps",      min_num_kps,      10);
    nhp.param("lowe_ratio",       lowe_ratio,       0.8);
    nhp.param("matching_type",    matching_type,    std::string("ratio_matching"));
    nhp.param("odom_frame_id",    odom_frame_id,    std::string("odom"));
    nhp.param("robot_frame_id",   robot_frame_id,   std::string("base_link"));
    nhp.param("enable_debugging", enable_debugging, true);

    // Printing parameters values.
    ROS_INFO_STREAM("Parameter setting: " << std::endl <<
                    "Minimum kps      = " << min_num_kps << std::endl << 
                    "Lowe's ratio     = " << lowe_ratio << std::endl <<
                    "Matching type    = " << matching_type << std::endl <<
                    "Odom frame id    = " << odom_frame_id << std::endl <<
                    "Robot frame id   = " << robot_frame_id << std::endl <<
                    "Enable debugging = " << enable_debugging);

    // Checking the validity of the parameters
    bool invalid_parameter = false;
    std::vector<std::string> matching_types = {"ratio_matching", "essential_matching", "homography_matching", "circular_ratio_matching",
                                               "circular_essential_matching", "circular_homography_matching"};
    for (uint i = 0; i < matching_types.size(); i++)
    {
        if (matching_type == matching_types[i])
        {
            invalid_parameter = false;
            break;
        }
        else
        {
            invalid_parameter = true;
        }
    }
    if (invalid_parameter)
    {
        matching_type = "ratio_matching";
    }
    if (min_num_kps < 10)
    {
        min_num_kps = 10;
        invalid_parameter = true;
    }
    if (lowe_ratio < 0.01 || lowe_ratio > 0.99)
    {
        min_num_kps = 0.8;
        invalid_parameter = true;
    }
    if (odom_frame_id == "")
    {
        odom_frame_id = "odom";
        invalid_parameter = true;
    }
    if (robot_frame_id == "")
    {
        robot_frame_id = "base_link";
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Minimum kps      = " << min_num_kps << std::endl << 
                        "Lowe's ratio     = " << lowe_ratio << std::endl <<
                        "Matching type    = " << matching_type << std::endl <<
                        "Odom frame id    = " << odom_frame_id << std::endl <<
                        "Robot frame id   = " << robot_frame_id << std::endl <<
                        "Enable debugging = " << enable_debugging);
    }
}
    

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamon");
    ros::start();

    // Read parameters
    int min_num_kps;
    double lowe_ratio;
    bool enable_debugging;
    std::string matching_type, odom_frame_id, robot_frame_id;
    readParams(min_num_kps, lowe_ratio, enable_debugging, matching_type, odom_frame_id, robot_frame_id);

    // Threads
    VisualOdometry visual_odometer(lowe_ratio, min_num_kps, enable_debugging, matching_type, odom_frame_id, robot_frame_id);

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