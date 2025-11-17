/**
 *  @file vo_node.hpp
 *  @brief Launch SLAMON's visual odometer as a ROS node.
 *  @package slamon
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @copyright
 *    Copyright (c) 2025 Bo Miquel Nordfeldt-Fiol
 *
 *  @license BSD-3-Clause
 *    This software is released under the BSD 3-Clause License.
 *    See the LICENSE file for full text.
 */

#include <vector>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include "visual_odometer.hpp"


void readParams(int& min_inliers,
                int& min_lc_inliers,
                int& max_kpts_bucket,
                int& max_kpts_retained,
                bool& enable_debugging,
                bool& enable_bucketing,
                bool& enable_enhacement,
                std::string& matching_type,
                std::string& robot_frame_id,
                std::string& feature_detector_name,
                std::string& motion_estimation_type)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/min_inliers",            min_inliers,            20);
    nh.param("slamon/min_lc_inliers",         min_lc_inliers,         40);
    nh.param("slamon/matching_type",          matching_type,          std::string("ratio_matching"));
    nh.param("slamon/robot_frame_id",         robot_frame_id,         std::string("base_link"));
    nh.param("slamon/max_kpts_bucket",        max_kpts_bucket,        4);
    nh.param("slamon/enable_debugging",       enable_debugging,       true);
    nh.param("slamon/enable_bucketing",       enable_bucketing,       true);
    nh.param("slamon/enable_enhacement",      enable_enhacement,      true);
    nh.param("slamon/max_kpts_retained",      max_kpts_retained,      1000);
    nh.param("slamon/feature_detector_name",  feature_detector_name,  std::string("sift"));
    nh.param("slamon/motion_estimation_type", motion_estimation_type, std::string("p3p"));

    // Printing parameters values.
    ROS_INFO_STREAM("[VO]: Parameter setting: " << std::endl <<
                    "Matching type                = " << matching_type << std::endl <<
                    "Robot frame id               = " << robot_frame_id << std::endl <<
                    "Minimum inliers              = " << min_inliers << std::endl << 
                    "Enable debugging             = " << enable_debugging << std::endl <<
                    "Enable bucketing             = " << enable_bucketing << std::endl <<
                    "Enable enhacement            = " << enable_enhacement << std::endl <<
                    "Feature detector name        = " << feature_detector_name << std::endl <<
                    "Motion estimation type       = " << motion_estimation_type << std::endl <<
                    "Min loop closing inliers     = " << min_lc_inliers << std::endl <<
                    "Maximum keypoints per image  = " << max_kpts_retained << std::endl <<
                    "Maximum keypoints per bucket = " << max_kpts_bucket);

    // Checking the validity of the parameters
    bool invalid_parameter = false;
    if (min_inliers < 20)
    {
        min_inliers = 20;
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
    if (feature_detector_name != "sift" && feature_detector_name != "surf")
    {
        feature_detector_name = "sift";
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("[VO]: Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Matching type                = " << matching_type << std::endl <<
                        "Robot frame id               = " << robot_frame_id << std::endl <<
                        "Minimum inliers              = " << min_inliers << std::endl << 
                        "Enable debugging             = " << enable_debugging << std::endl <<
                        "Enable bucketing             = " << enable_bucketing << std::endl <<
                        "Enable enhacement            = " << enable_enhacement << std::endl <<
                        "Feature detector name        = " << feature_detector_name << std::endl <<
                        "Motion estimation type       = " << motion_estimation_type << std::endl <<
                        "Min loop closing inliers     = " << min_lc_inliers << std::endl <<
                        "Maximum keypoints per image  = " << max_kpts_retained << std::endl <<
                        "Maximum keypoints per bucket = " << max_kpts_bucket);
    }
}
    

int main(int argc, char **argv)
{
    // Init ROS.
    ros::init(argc, argv, "slamon_vo");
    ros::start();

    // Read parameters.
    bool enable_debugging, enable_bucketing, enable_enhacement;
    int min_inliers, min_lc_inliers, max_kpts_bucket, max_kpts_retained;
    std::string matching_type, robot_frame_id, feature_detector_name, motion_estimation_type;
    readParams(min_inliers, min_lc_inliers, max_kpts_bucket, max_kpts_retained, enable_debugging, enable_bucketing, 
               enable_enhacement, matching_type, robot_frame_id, feature_detector_name, motion_estimation_type);

    // Declare threads.
    slamon::VisualOdometer visual_odometer(min_inliers,
                                           min_lc_inliers,
                                           max_kpts_bucket,
                                           max_kpts_retained,
                                           enable_debugging, 
                                           enable_bucketing,
                                           enable_enhacement,
                                           matching_type, 
                                           robot_frame_id,
                                           feature_detector_name,
                                           motion_estimation_type);

    // Launch threads.
    std::thread visual_odometer_thread(&slamon::VisualOdometer::run, &visual_odometer);

    // ROS wait
    ros::Rate r(10);
    while (ros::ok())
    {
        r.sleep();
    }

    return 0;
}