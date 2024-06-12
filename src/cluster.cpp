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
    frame_id_ = frame_id;
    camera_pose_ = camera_pose;
}


std::vector<cv::Point3f> Cluster::getWorldPoints()
{
    tf::Vector3 p_tf;
    tf::Vector3 p_tf_world;
    std::vector<cv::Point3f> world_pts;
    for (uint i = 0; i < obj_pts_.size(); i++)
    { 
        p_tf.setX(obj_pts_[i].x);
        p_tf.setY(obj_pts_[i].y);
        p_tf.setZ(obj_pts_[i].z);
        p_tf_world = camera_pose_ * p_tf;
        world_pts.push_back(cv::Point3f(p_tf_world.getX(),
                                        p_tf_world.getY(),
                                        p_tf_world.getZ()));
    }
    return world_pts;
}