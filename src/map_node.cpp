#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "map.hpp"
#include "queue.hpp"
#include "cluster.hpp"
#include "mapmaker.hpp"
#include "loop_closer.hpp"


void readParams(int& lc_neighbors,
                int& lc_candidates,
                int& min_lc_inliers,
                int& lc_discard_window,
                bool& enable_debugging,
                double& min_dist_keyframe,
                std::string& working_path,
                std::string& matching_type,
                std::string& robot_frame_id,
                std::string& motion_estimation_type)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/lc_neighbors",           lc_neighbors,           4);
    nh.param("slamon/working_path",           working_path,           ros::package::getPath("slamon") + "/output/");
    nh.param("slamon/lc_candidates",          lc_candidates,          3);
    nh.param("slamon/matching_type",          matching_type,          std::string("ratio_matching"));
    nh.param("slamon/min_lc_inliers",         min_lc_inliers,         40);
    nh.param("slamon/robot_frame_id",         robot_frame_id,         std::string("base_link"));
    nh.param("slamon/enable_debugging",       enable_debugging,       true);
    nh.param("slamon/lc_discard_window",      lc_discard_window,      20);
    nh.param("slamon/min_dist_keyframe",      min_dist_keyframe,      0.5);
    nh.param("slamon/motion_estimation_type", motion_estimation_type, std::string("p3p"));

    // Printing parameters values.
    ROS_INFO_STREAM("[MAP]: Parameter setting: " << std::endl <<
                    "Num loop closing neighbors      = " << lc_neighbors << std::endl <<
                    "Working path                    = " << working_path << std::endl <<
                    "Num loop closing candidates     = " << lc_candidates << std::endl <<
                    "Matching type                   = " << matching_type << std::endl <<
                    "Min loop closing inliers        = " << min_lc_inliers << std::endl <<
                    "Robot frame id                  = " << robot_frame_id << std::endl <<
                    "Enable debugging                = " << enable_debugging << std::endl <<
                    "Num loop closing discard window = " << lc_discard_window << std::endl <<
                    "Min dist between keyframes      = " << min_dist_keyframe << std::endl <<
                    "Motion estimation type          = " << motion_estimation_type);

    // Checking the validity of the parameters
    bool invalid_parameter = false;
    if (lc_neighbors <= 0)
    {
        lc_neighbors = 4;
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
    if (lc_candidates < 1 || lc_candidates > 7)
    {
        lc_candidates = 3;
        invalid_parameter = true;
    }
    if (min_lc_inliers < 30)
    {
        min_lc_inliers = 30;
        invalid_parameter = true;
    }
    if (robot_frame_id == "")
    {
        robot_frame_id = "base_link";
        invalid_parameter = true;
    }
    if (lc_discard_window < 10)
    {
        lc_discard_window = 10;
        invalid_parameter = true;
    }
    if (min_dist_keyframe < 0.15 || min_dist_keyframe > 1.0)
    {
        min_dist_keyframe = 0.5;
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("[MAP]: Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Num loop closing neighbors      = " << lc_neighbors << std::endl <<
                        "Working path                    = " << working_path << std::endl <<
                        "Num loop closing candidates     = " << lc_candidates << std::endl <<
                        "Matching type                   = " << matching_type << std::endl <<
                        "Min loop closing inliers        = " << min_lc_inliers << std::endl <<
                        "Robot frame id                  = " << robot_frame_id << std::endl <<
                        "Enable debugging                = " << enable_debugging << std::endl <<
                        "Num loop closing discard window = " << lc_discard_window << std::endl <<
                        "Min dist between keyframes      = " << min_dist_keyframe << std::endl <<
                        "Motion estimation type          = " << motion_estimation_type);
    }
}


int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "slamon_map");
    ros::start();

    // Read parameters.
    bool enable_debugging;
    double min_dist_keyframe;
    int lc_neighbors, min_lc_inliers, lc_discard_window, lc_candidates;
    std::string working_path, matching_type, robot_frame_id, motion_estimation_type;
    readParams(lc_neighbors, 
               lc_candidates,
               min_lc_inliers,
               lc_discard_window,
               enable_debugging,
               min_dist_keyframe,
               working_path,
               matching_type,
               robot_frame_id,
               motion_estimation_type);

    // Declare shared resources.
    slamon::Map map;
    slamon::Queue<Cluster> cluster_queue;

    // Declare threads.
    slamon::Mapmaker mapmaker(&map,
                              min_lc_inliers, 
                              enable_debugging, 
                              min_dist_keyframe, 
                              working_path, 
                              &cluster_queue,
                              robot_frame_id);
    slamon::LoopCloser loop_closer(&map,
                                   lc_neighbors, 
                                   lc_candidates,
                                   min_lc_inliers,
                                   lc_discard_window,
                                   enable_debugging,
                                   working_path,
                                   matching_type,
                                   &cluster_queue,
                                   motion_estimation_type);

    // Launch threads.
    std::thread map_thread(&slamon::Mapmaker::run, &mapmaker);
    std::thread loop_closer_thread(&slamon::LoopCloser::run, &loop_closer);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}