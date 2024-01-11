#include "frame.hpp"


Frame::Frame(const cv::Mat& l_descs,
             const cv::Mat& r_descs,
             const ros::Time& stamp,
             const std::vector<cv::KeyPoint>& l_kps,
             const std::vector<cv::KeyPoint>& r_kps,
             const std::vector<cv::DMatch>& stereo_matches,
             const std::vector<cv::Point3f>& camera_points)
{
    l_kps_ = l_kps;
    r_kps_ = r_kps;
    stamp_ = stamp;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    camera_points_ = camera_points;
    stereo_matches_ = stereo_matches;
}