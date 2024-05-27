#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "graph.hpp"


void readParams(bool& enable_debugging,
                std::string& working_path)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/working_path",              working_path,           ros::package::getPath("slamon") + "/output/");
    nh.param("slamon/enable_debugging",          enable_debugging,       true);

    // Printing parameters values.
    ROS_INFO_STREAM("Parameter setting: " << std::endl <<
                    "Working path           = " << working_path << std::endl <<
                    "Enable debugging       = " << enable_debugging);

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
    bool enable_debugging;
    std::string working_path;
    readParams(enable_debugging, working_path);

    // Declare threads.
    Graph graph(enable_debugging, 
                working_path);

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