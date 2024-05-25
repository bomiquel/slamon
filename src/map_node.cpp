#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "graph.hpp"


int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "map_slamon");
    ros::start();

    // Read parameters.
    bool enable_debugging = true;

    // Declare threads.
    slam::Graph graph(enable_debugging);

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