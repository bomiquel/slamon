#include "frame.hpp"


Frame::Frame(const cv::Mat& l_descs,
             const cv::Mat& r_descs,
             const ros::Time& stamp,
             const std::vector<cv::Point2f>& l_kpts,
             const std::vector<cv::Point2f>& r_kpts,
             const std::vector<cv::Point3f>& obj_pts)
{
    stamp_ = stamp;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    obj_pts_ = obj_pts;
}


Frame::Frame(const cv::Mat& l_img,
             const cv::Mat& r_img,
             const cv::Mat& l_descs,
             const cv::Mat& r_descs,
             const ros::Time& stamp,
             const std::vector<cv::Point2f>& l_kpts,
             const std::vector<cv::Point2f>& r_kpts,
             const std::vector<cv::Point3f>& obj_pts)
{
    l_img_ = l_img;
    r_img_ = r_img;
    stamp_ = stamp;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    obj_pts_ = obj_pts;
}