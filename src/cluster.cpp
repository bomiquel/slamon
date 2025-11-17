/**
 *  @file cluster.cpp
 *  @brief The Cluster class represents a grouping of keypoints extracted from a keyframe.
 *  @package slamon
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group 
 *  (srv.uib.es), University of the Balearic Islands. The original work is 
 *  licensed under the BSD 4-Clause License (see below).
 *
 *  @author
 *  Original: 
 *    Pep Lluís Negre-Carrasco
 *  Modified by: 
 *    Bo Miquel Nordfeldt-Fiol
 * 
 *  @copyright
 *  Original copyright holder:
 *      Systems, Robotics and Vision Group (srv.uib.es)
 *      University of the Balearic Islands
 *      Copyright (c) 2013-2014
 *  Modified by:
 *      Bo Miquel Nordfeldt-Fiol (2024–2025)
 *      Copyright (c) 2025
 *      Released under the BSD 3-Clause License.
 *
 *  ---------------------------------------------------------------------------
 *  Original BSD 4-Clause License Notice:
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * All advertising materials mentioning features or use of this software
 *        must display the following acknowledgement:
 *          "This product includes software developed by the
 *           Systems, Robotics and Vision Group,
 *           University of the Balearic Islands"
 *      * Neither the name of the Systems, Robotics and Vision Group,
 *        University of the Balearic Islands nor the names of its contributors
 *        may be used to endorse or promote products derived from this software
 *        without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 *  ---------------------------------------------------------------------------
 *
 *  Additional license for modifications:
 *      The modifications made by Bo Miquel Nordfeldt-Fiol are released under
 *      the BSD 3-Clause License. See the top-level LICENSE file for details.
 */

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