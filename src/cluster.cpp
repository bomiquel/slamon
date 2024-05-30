#include "cluster.hpp"


Cluster::Cluster()
{
    id_ = -1;
}


Cluster::Cluster(int id,
                 int frame_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform camera_pose,
                 std::vector<cv::Point2f> l_kpts,
                 std::vector<cv::Point2f> r_kpts,
                 std::vector<cv::Point3f> obj_pts)
{
    id_ = id;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    obj_pts_ = obj_pts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    frame_id_ = frame_id_;
    camera_pose_ = camera_pose;
}