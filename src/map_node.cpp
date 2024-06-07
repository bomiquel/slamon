#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "graph.hpp"


void readParams(int& lc_neighbors,
                int& min_lc_inliers,
                int& lc_discard_window,
                bool& enable_debugging,
                double& min_dist_keyframe,
                std::string& working_path,
                std::string& robot_frame_id)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/working_path",          working_path,      ros::package::getPath("slamon") + "/output/");
    nh.param("slamon/robot_frame_id",        robot_frame_id,    std::string("base_link"));
    nh.param("slamon/enable_debugging",      enable_debugging,  true);
    nh.param("slamon/map/lc_neighbors",      lc_neighbors,      4);
    nh.param("slamon/map/min_lc_inliers",    min_lc_inliers,    40);
    nh.param("slamon/map/lc_discard_window", lc_discard_window, 20);
    nh.param("slamon/map/min_dist_keyframe", min_dist_keyframe, 0.5);

    // Printing parameters values.
    ROS_INFO_STREAM("[MAP]: Parameter setting: " << std::endl <<
                    "Working path                    = " << working_path << std::endl <<
                    "Num loop closing neighbors      = " << lc_neighbors << std::endl <<
                    "Min loop closing inliers        = " << min_lc_inliers << std::endl <<
                    "Robot frame id                  = " << robot_frame_id << std::endl <<
                    "Enable debugging                = " << enable_debugging << std::endl <<
                    "Num loop closing discard window = " << lc_discard_window << std::endl <<
                    "Min dist between keyframes      = " << min_dist_keyframe);

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
    if (robot_frame_id == "")
    {
        robot_frame_id = "base_link";
        invalid_parameter = true;
    }
    if (min_dist_keyframe < 0.2 || min_dist_keyframe > 1.0)
    {
        min_dist_keyframe = 0.5;
        invalid_parameter = true;
    }
    if (min_lc_inliers < 30)
    {
        min_lc_inliers = 30;
        invalid_parameter = true;
    }
    if (lc_neighbors <= 0)
    {
        lc_neighbors = 4;
        invalid_parameter = true;
    }
    if (lc_discard_window < 10)
    {
        lc_discard_window = 10;
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("[MAP]: Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Working path                    = " << working_path << std::endl <<
                        "Num loop closing neighbors      = " << lc_neighbors << std::endl <<
                        "Min loop closing inliers        = " << min_lc_inliers << std::endl <<
                        "Robot frame id                  = " << robot_frame_id << std::endl <<
                        "Enable debugging                = " << enable_debugging << std::endl <<
                        "Num loop closing discard window = " << lc_discard_window << std::endl <<
                        "Min dist between keyframes      = " << min_dist_keyframe);
    }
}


int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "map_slamon");
    ros::start();

    // Read parameters.
    double min_dist_keyframe;
    bool enable_debugging;
    std::string working_path, robot_frame_id;
    int lc_neighbors, min_lc_inliers, lc_discard_window;
    readParams(lc_neighbors, 
               min_lc_inliers,
               lc_discard_window,
               enable_debugging,
               min_dist_keyframe,
               working_path,
               robot_frame_id);

    // Declare threads.
    slam::LoopClosing loop_closing(lc_neighbors, 
                                   min_lc_inliers,
                                   lc_discard_window,
                                   enable_debugging,
                                   working_path);
    slam::Graph graph(min_lc_inliers, 
                      enable_debugging, 
                      min_dist_keyframe, 
                      working_path, 
                      &loop_closing, 
                      robot_frame_id);
    loop_closing.setGraph(&graph);

    // Launch threads.
    std::thread graph_thread(&slam::Graph::run, &graph);
    std::thread loop_closing_thread(&slam::LoopClosing::run, &loop_closing);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}