#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "graph.hpp"


void readParams(int& min_inliers,
                double& lowe_ratio, 
                bool& enable_debugging,
                std::string& working_path,
                std::string& matching_type,
                std::string& motion_estimation_type)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/working_path",              working_path,           ros::package::getPath("slamon") + "/output/");
    nh.param("slamon/enable_debugging",          enable_debugging,       true);
    nh.param("slamon/vo/min_inliers",            min_inliers,            10);
    nh.param("slamon/vo/lowe_ratio",             lowe_ratio,             0.8);
    nh.param("slamon/vo/matching_type",          matching_type,          std::string("ratio_matching"));
    nh.param("slamon/vo/motion_estimation_type", motion_estimation_type, std::string("p3p"));

      // Printing parameters values.
    ROS_INFO_STREAM("Parameter setting: " << std::endl <<
                    "Minimum kpts           = " << min_inliers << std::endl << 
                    "Lowe's ratio           = " << lowe_ratio << std::endl <<
                    "Working path           = " << working_path << std::endl <<
                    "Matching type          = " << matching_type << std::endl <<
                    "Enable debugging       = " << enable_debugging << std::endl <<
                    "Motion estimation type = " << motion_estimation_type);

    // Checking the validity of the parameters
    bool invalid_parameter = false;
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
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Working path     = " << working_path << std::endl <<
                        "Enable debugging = " << enable_debugging);
    }
}


int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "map_slamon");
    ros::start();

    // Read parameters.
    int min_inliers;
    double lowe_ratio;
    bool enable_debugging;
    std::string working_path, matching_type, motion_estimation_type;
    readParams(min_inliers, lowe_ratio, enable_debugging, working_path, matching_type, motion_estimation_type);

    // Declare threads.
    Graph graph(lowe_ratio, 
                min_inliers, 
                enable_debugging, 
                working_path, 
                matching_type, 
                motion_estimation_type);

    // Launch threads.
    std::thread graph_thread(&Graph::run, &graph);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}