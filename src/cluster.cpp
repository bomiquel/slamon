#include "cluster.hpp"


Cluster::Cluster()
{
    id_ = -1;
}


Cluster::Cluster(int id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform camera_pose,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> cam_pts)
{
    id_ = id;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    cam_pts_ = cam_pts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    keyframe_id_ = keyframe_id;
    camera_pose_ = camera_pose;
}


std::vector<cv::Point3f> Cluster::getWorldPoints()
{
    tf::Vector3 tf_cam_pt;
    tf::Vector3 tf_world_pt;
    cv::Point3f cv_world_pt;
    std::vector<cv::Point3f> tf_world_pts;
    for (uint i = 0; i < cam_pts_.size(); i++)
    { 
        tf_cam_pt.setX(cam_pts_[i].x);
        tf_cam_pt.setY(cam_pts_[i].y);
        tf_cam_pt.setZ(cam_pts_[i].z);
        tf_world_pt = camera_pose_ * tf_cam_pt;
        cv_world_pt.x = tf_world_pt.getX();
        cv_world_pt.y = tf_world_pt.getY();
        cv_world_pt.z = tf_world_pt.getZ();
        tf_world_pts.push_back(cv_world_pt);
    }
    return tf_world_pts;
}