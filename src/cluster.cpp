#include "cluster.hpp"


Cluster::Cluster(): global_id_{-1}, local_id_{-1} {}


Cluster::Cluster(int global_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform map_camera_pose,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts): 
                 global_id_{global_id}, local_id_{-1}, keyframe_id_{keyframe_id}, l_descs_{l_descs}, r_descs_{r_descs},
                 map_camera_pose_{map_camera_pose}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


Cluster::Cluster(int global_id,
                 int local_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform map_camera_pose,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts): 
                 global_id_{global_id}, local_id_{local_id}, keyframe_id_{keyframe_id}, l_descs_{l_descs}, r_descs_{r_descs},
                 map_camera_pose_{map_camera_pose}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


std::vector<cv::Point3f> Cluster::getWorldPoints()
{
    tf::Vector3 tf_l_cam_pt;
    tf::Vector3 tf_world_pt;
    cv::Point3f cv_world_pt;
    std::vector<cv::Point3f> tf_world_pts;
    for (unsigned int i = 0; i < l_cam_pts_.size(); i++)
    { 
        tf_l_cam_pt.setX(l_cam_pts_[i].x);
        tf_l_cam_pt.setY(l_cam_pts_[i].y);
        tf_l_cam_pt.setZ(l_cam_pts_[i].z);
        tf_world_pt = map_camera_pose_ * tf_l_cam_pt;
        cv_world_pt.x = tf_world_pt.getX();
        cv_world_pt.y = tf_world_pt.getY();
        cv_world_pt.z = tf_world_pt.getZ();
        tf_world_pts.push_back(cv_world_pt);
    }
    return tf_world_pts;
}