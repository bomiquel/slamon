/**
 *  @file
 *  @brief The Cluster class represents one keypoints clustering of the camera keyframe.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024-2025
 */

#ifndef CLUSTER_HPP
#define CLUSTER_HPP


#include <vector>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf/tf.h>


class Cluster
{
    public:

        /** \brief Empty class constructor
         */
        Cluster();


        /** \brief Class constructor.
         *  \param[in] global_id Global cluster id.
         *  \param[in] keyframe_id Keyframe id.
         *  \param[in] l_descs Cluster left descriptors.
         *  \param[in] r_descs Cluster right descriptors.
         *  \param[in] l_kpts Cluster left keypoints.
         *  \param[in] r_kpts Cluster right keypoints.
         *  \param[in] l_cam_pts Cluster 3D points relative to left camera.
         */
        Cluster(int global_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Class constructor.
         *  \param[in] global_id Global cluster id.
         *  \param[in] local_id Local cluster id.
         *  \param[in] keyframe_id Keyframe id.
         *  \param[in] l_descs Cluster left descriptors.
         *  \param[in] r_descs Cluster right descriptors.
         *  \param[in] l_kpts Cluster left keypoints.
         *  \param[in] r_kpts Cluster right keypoints.
         *  \param[in] l_cam_pts Cluster 3D points relative to left camera.
         */
        Cluster(int global_id,
                int local_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Class constructor.
         *  \param[in] global_id Global cluster id.
         *  \param[in] keyframe_id Keyframe id.
         *  \param[in] l_descs Cluster left descriptors.
         *  \param[in] r_descs Cluster right descriptors.
         *  \param[in] camera_map_pose World-map camera pose when the cluster was obtained.
         *  \param[in] l_kpts Cluster left keypoints.
         *  \param[in] r_kpts Cluster right keypoints.
         *  \param[in] l_cam_pts Cluster 3D points relative to left camera.
         */
        Cluster(int global_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf::Transform camera_map_pose,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Class constructor.
         *  \param[in] global_id Global cluster id.
         *  \param[in] local_id Local cluster id.
         *  \param[in] keyframe_id Keyframe id.
         *  \param[in] l_descs Cluster left descriptors.
         *  \param[in] r_descs Cluster right descriptors.
         *  \param[in] camera_map_pose World-map camera pose when the cluster was obtained.
         *  \param[in] l_kpts Cluster left keypoints.
         *  \param[in] r_kpts Cluster right keypoints.
         *  \param[in] l_cam_pts Cluster 3D points relative to left camera.
         */
        Cluster(int global_id,
                int local_id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf::Transform camera_map_pose,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Get the global cluster id.
         *  @return The global cluster id.
         */
        inline int getGlobalId() const {return global_id_;}


        /** \brief Get the local cluster id.
         *  @return The local cluster id.
         */
        inline int getLocalId() const {return local_id_;}


        /** \brief Get the corresponding keyframe id.
         *  @return The corresponding keyframe id.
         */
        inline int getKeyframeId() const {return keyframe_id_;}


        /** \brief Get cluster left descriptors.
         *  @return The cluster left descriptors.
         */
        inline cv::Mat getLeftDescs() const {return l_descs_;}


        /** \brief Get cluster right descriptors.
         *  @return The cluster right descriptors.
         */
        inline cv::Mat getRightDescs() const {return r_descs_;}


        /** \brief Get cluster left keypoints.
         *  @return The cluster left keypoints.
         */
        inline std::vector<cv::KeyPoint> getLeftKpts() const {return l_kpts_;}


        /** \brief Get camera world-map pose.
         *  @return The camera world-map pose.
         */
        inline tf::Transform getCameraPose() const {return camera_map_pose_;}


        /** \brief Get cluster right keypoints.
         *  @return The cluster right keypoints.
         */
        inline std::vector<cv::KeyPoint> getRightKpts() const {return r_kpts_;}


        /** \brief Get cluster 3D points relative to left camera.
         *  @return The cluster 3D points relative to left camera.
         */
        inline std::vector<cv::Point3f> getLeftCameraPoints() const {return l_cam_pts_;}


        /** \brief Transforms 3D points relative to the camera to relative to the
         *  world-map coordinate axis. 
         *  @return The cluster 3D points relative to world-map.
         */
        std::vector<cv::Point3f> getWorldPoints();


        /** \brief Set a new world-map camera pose.
         *  \param[in] camera_map_pose New world-map camera pose.
         */
        inline void updateCameraPose(const tf::Transform camera_map_pose) {camera_map_pose_ = camera_map_pose;}
    

    private:

        // Attributes:

        int global_id_; //!> Global cluster id.

        int local_id_; //!> Local cluster id.

        int keyframe_id_; //!> Corresponding keyframe id.

        cv::Mat l_descs_; //!> Cluster left descriptors.

        cv::Mat r_descs_; //!> Cluster right descriptors.

        tf::Transform camera_map_pose_; //!> World-map camera pose when the cluster was obtained.

        std::vector<cv::KeyPoint> l_kpts_; //!> Cluster left keypoints.

        std::vector<cv::KeyPoint> r_kpts_; //!> Cluster right keypoints.

        std::vector<cv::Point3f> l_cam_pts_; //!> Cluster 3D points relative to left camera.

};

#endif // CLUSTER_HPP