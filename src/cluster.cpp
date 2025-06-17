#include "cluster.hpp"


Cluster::Cluster(): global_id_{-1}, local_id_{-1} 
{
    camera_map_pose_.setIdentity();
}


Cluster::Cluster(int global_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts):
                 global_id_{global_id}, local_id_{-1}, keyframe_id_{keyframe_id}, 
                 l_descs_{l_descs}, r_descs_{r_descs}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} 
{
    camera_map_pose_.setIdentity();
}


Cluster::Cluster(int global_id,
                 int local_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts):
                 global_id_{global_id}, local_id_{local_id}, keyframe_id_{keyframe_id}, 
                 l_descs_{l_descs}, r_descs_{r_descs}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, 
                 l_cam_pts_{l_cam_pts}
{
    camera_map_pose_.setIdentity();
}


Cluster::Cluster(int global_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform camera_map_pose,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts):
                 global_id_{global_id}, local_id_{-1}, keyframe_id_{keyframe_id}, l_descs_{l_descs}, r_descs_{r_descs},
                 camera_map_pose_{camera_map_pose}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


Cluster::Cluster(int global_id,
                 int local_id,
                 int keyframe_id,
                 cv::Mat l_descs,
                 cv::Mat r_descs,
                 tf::Transform camera_map_pose,
                 std::vector<cv::KeyPoint> l_kpts,
                 std::vector<cv::KeyPoint> r_kpts,
                 std::vector<cv::Point3f> l_cam_pts):
                 global_id_{global_id}, local_id_{local_id}, keyframe_id_{keyframe_id}, l_descs_{l_descs}, r_descs_{r_descs},
                 camera_map_pose_{camera_map_pose}, l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


std::vector<cv::Point3f> Cluster::getWorldPoints()
{    
    // Pre-allocates memory.
    const size_t size = l_cam_pts_.size();
    std::vector<cv::Point3f> world_pts;
    world_pts.reserve(size);

    // Translates 3D points to world frame.
    tf::Vector3 l_cam_pt;
    tf::Vector3 world_pt;
    for (size_t i = 0; i < size; ++i)
    { 
        l_cam_pt.setX(l_cam_pts_[i].x);
        l_cam_pt.setY(l_cam_pts_[i].y);
        l_cam_pt.setZ(l_cam_pts_[i].z);
        world_pt = camera_map_pose_ * l_cam_pt;
        world_pts.emplace_back(world_pt.getX(), world_pt.getY(), world_pt.getZ());
    }
    
    return world_pts;
}