#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "graph.hpp"


void readParams(int& min_lc_inliers,
                double& min_dist_kf,
                bool& enable_debugging,
                std::string& working_path)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/working_path",       working_path,     ros::package::getPath("slamon") + "/output/");
    nh.param("slamon/enable_debugging",   enable_debugging, true);
    nh.param("slamon/map/min_dist_kf",    min_dist_kf,      0.5);
    nh.param("slamon/map/min_lc_inliers", min_lc_inliers,   40);

    // Printing parameters values.
    ROS_INFO_STREAM("[MAP]: Parameter setting: " << std::endl <<
                    "Working path               = " << working_path << std::endl <<
                    "Enable debugging           = " << enable_debugging << std::endl <<
                    "Min dist between keyframes = " << min_dist_kf << std::endl <<
                    "Min loop closing inliers   = " << min_lc_inliers);

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
    if (min_dist_kf < 0.2 || min_dist_kf > 1.0)
    {
        min_dist_kf = 0.5;
        invalid_parameter = true;
    }
    if (min_lc_inliers < 30)
    {
        min_lc_inliers = 30;
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("[MAP]: Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Working path               = " << working_path << std::endl <<
                        "Enable debugging           = " << enable_debugging << std::endl <<
                        "Min dist between keyframes = " << min_dist_kf << std::endl <<
                        "Min loop closing inliers   = " << min_lc_inliers);
    }
}


int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "map_slamon");
    ros::start();

    // Read parameters.
    int min_lc_inliers;
    double min_dist_kf;
    bool enable_debugging;
    std::string working_path;
    readParams(min_lc_inliers, min_dist_kf, enable_debugging, working_path);

    // Declare threads.
    slam::LoopClosing loop_closing(min_lc_inliers, enable_debugging, working_path);
    slam::Graph graph(min_lc_inliers, min_dist_kf, enable_debugging, working_path, &loop_closing);

    // Launch threads.
    std::thread graph_thread(&slam::Graph::run, &graph);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}