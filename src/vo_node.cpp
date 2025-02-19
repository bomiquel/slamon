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
                double& lowe_ratio, 
                bool& enable_debugging,
                bool& enable_bucketing,
                bool& enable_enhacement,
                std::string& working_path,
                std::string& matching_type,
                std::string& robot_frame_id,
                std::string& feature_detector_name,
                std::string& motion_estimation_type)
{
    // Getting parameters from parameter server.
    ros::NodeHandle nh;
    nh.param("slamon/lowe_ratio",             lowe_ratio,             0.8);
    nh.param("slamon/min_inliers",            min_inliers,            20);
    nh.param("slamon/min_lc_inliers",         min_lc_inliers,         40);
    nh.param("slamon/working_path",           working_path,           ros::package::getPath("slamon") + "/output/");
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
                    "Lowe's ratio                 = " << lowe_ratio << std::endl <<
                    "Working path                 = " << working_path << std::endl <<
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
    if (lowe_ratio < 0.40 || lowe_ratio > 0.95)
    {
        lowe_ratio = 0.85;
        invalid_parameter = true;
    }
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
    if (feature_detector_name != "sift" && feature_detector_name != "surf")
    {
        feature_detector_name = "sift";
        invalid_parameter = true;
    }
    if (invalid_parameter)
    {
        ROS_INFO_STREAM("[VO]: Some parameter was invalid and has been assigned its default value." << std::endl <<
                        "Current parameter values are: " << std::endl <<
                        "Lowe's ratio                 = " << lowe_ratio << std::endl <<
                        "Working path                 = " << working_path << std::endl <<
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
    ros::init(argc, argv, "vo_slamon");
    ros::start();

    // Read parameters.
    double lowe_ratio;
    bool enable_debugging, enable_bucketing, enable_enhacement;
    int min_inliers, min_lc_inliers, max_kpts_bucket, max_kpts_retained;
    std::string working_path, matching_type, robot_frame_id, feature_detector_name, motion_estimation_type;
    readParams(min_inliers, min_lc_inliers, max_kpts_bucket, max_kpts_retained, lowe_ratio, enable_debugging, enable_bucketing, 
               enable_enhacement, working_path, matching_type, robot_frame_id, feature_detector_name, motion_estimation_type);

    // Declare threads.
    slamon::VisualOdometer visual_odometer(lowe_ratio, 
                                           min_inliers,
                                           min_lc_inliers,
                                           max_kpts_bucket,
                                           max_kpts_retained,
                                           enable_debugging, 
                                           enable_bucketing,
                                           enable_enhacement,
                                           working_path,
                                           feature_detector_name,
                                           matching_type, 
                                           robot_frame_id,
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