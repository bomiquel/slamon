/**
 *  \file cluster.hpp
 *  \brief The Cluster class represents a grouping of keypoints extracted from a keyframe.
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  \author Original: Pep Lluís Negre-Carrasco (2013)
 *  \author Modified: Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon). Both the original 
 *  work and the modifications are released under the BSD 4-Clause License. 
 *  See the top-level LICENSE file for details.
 */

 
#ifndef CLUSTER_HPP
#define CLUSTER_HPP


#include <vector>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>


/** \class Cluster
 *  \brief Represents a spatial grouping (cluster) of stereo keypoints and 3D points 
 *         extracted from a specific keyframe.
 *  
 *  This class encapsulates 2D stereo keypoints, their corresponding feature descriptors,
 *  triangulated 3D points in the left camera frame, and the associated camera pose in the world map.
 */
class Cluster
{
    public:

        /** \brief Default class constructor.
         */
        Cluster();


        /** \brief Parameterized constructor (without local ID or camera pose).
         *  \param[in] global_id Global cluster ID.
         *  \param[in] keyframe_id Corresponding keyframe ID.
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Cluster(int global_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Parameterized constructor (with local ID, without camera pose).
         *  \param[in] global_id Global cluster ID.
         *  \param[in] local_id Local cluster ID within the keyframe.
         *  \param[in] keyframe_id Corresponding keyframe ID.
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Cluster(int global_id,
                int local_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Parameterized constructor (with camera pose, without local ID).
         *  \param[in] global_id Global cluster ID.
         *  \param[in] keyframe_id Corresponding keyframe ID.
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] camera_map_pose World-map camera pose when the cluster was obtained.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Cluster(int global_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf2::Transform camera_map_pose,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Full parameterized constructor (including local ID and camera pose).
         *  \param[in] global_id Global cluster ID.
         *  \param[in] local_id Local cluster ID within the keyframe.
         *  \param[in] keyframe_id Corresponding keyframe ID.
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] camera_map_pose World-map camera pose when the cluster was obtained.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Cluster(int global_id,
                int local_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf2::Transform camera_map_pose,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Gets the global cluster ID.
         *  \return The global cluster ID.
         */
        inline int getGlobalId() const {return global_id_;}


        /** \brief Gets the local cluster ID.
         *  \return The local cluster ID.
         */
        inline int getLocalId() const {return local_id_;}


        /** \brief Gets the corresponding keyframe ID.
         *  \return The corresponding keyframe ID.
         */
        inline int getKeyframeId() const {return keyframe_id_;}


        /** \brief Gets the cluster left descriptors.
         *  \return Matrix containing the cluster left descriptors.
         */
        inline cv::Mat getLeftDescs() const {return l_descs_;}


        /** \brief Gets the cluster right descriptors.
         *  \return Matrix containing the cluster right descriptors.
         */
        inline cv::Mat getRightDescs() const {return r_descs_;}


        /** \brief Gets the cluster left keypoints.
         *  \return Vector of cluster left keypoints.
         */
        inline const std::vector<cv::KeyPoint>& getLeftKpts() const {return l_kpts_;}


        /** \brief Gets the cluster right keypoints.
         *  \return Vector of cluster right keypoints.
         */
        inline const std::vector<cv::KeyPoint>& getRightKpts() const {return r_kpts_;}


        /** \brief Gets the camera world-map pose.
         *  \return The camera pose in the world-map frame.
         */
        inline const tf2::Transform& getCameraPose() const {return camera_map_pose_;}


        /** \brief Gets the cluster 3D points relative to the left camera.
         *  \return Vector of 3D points in the left camera coordinate frame.
         */
        inline const std::vector<cv::Point3f>& getLeftCameraPoints() const {return l_cam_pts_;}


        /** \brief Transforms 3D points from the left camera frame to the world-map coordinate frame.
         *  \return Vector of cluster 3D points relative to the world-map frame.
         */
        std::vector<cv::Point3f> getWorldPoints();


        /** \brief Sets a new world-map camera pose for the cluster.
         *  \param[in] camera_map_pose New world-map camera pose.
         */
        inline void updateCameraPose(const tf2::Transform& camera_map_pose) {camera_map_pose_ = camera_map_pose;}
    

    private:

        /** \brief Global cluster ID. */
        int global_id_;

        /** \brief Local cluster ID within the keyframe. */
        int local_id_;

        /** \brief Corresponding keyframe ID. */
        int keyframe_id_;

        /** \brief Cluster left descriptors. */
        cv::Mat l_descs_;

        /** \brief Cluster right descriptors. */
        cv::Mat r_descs_;

        /** \brief World-map camera pose when the cluster was obtained. */
        tf2::Transform camera_map_pose_;

        /** \brief Cluster left keypoints. */
        std::vector<cv::KeyPoint> l_kpts_;

        /** \brief Cluster right keypoints. */
        std::vector<cv::KeyPoint> r_kpts_;

        /** \brief Cluster 3D points relative to the left camera coordinate frame. */
        std::vector<cv::Point3f> l_cam_pts_;

};

#endif // CLUSTER_HPP