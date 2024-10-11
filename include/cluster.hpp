/**
 *  @file
 *  @brief The Cluster class represents one keypoints clustering of the camera keyframe.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024
 */

#ifndef CLUSTER_HPP
#define CLUSTER_HPP


#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>


class Cluster
{
    public:

        /** \brief Empty class constructor
         */
        Cluster();


        /** \brief Class constructor
         *  \param id Cluster id.
         *  \param keyframe_id Keyframe id.
         *  \param l_descs Cluster left descriptors.
         *  \param r_descs Cluster right descriptors.
         *  \param camera_pose World camera pose when the cluster was obtained.
         *  \param l_kpts Cluster left keypoints.
         *  \param r_kpts Cluster right keypoints.
         *  \param l_cam_pts Cluster 3D points relative to left camera.
         */
        Cluster(int id,
                int keyframe_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf::Transform camera_pose,
                std::vector<cv::KeyPoint> l_kpts,
                std::vector<cv::KeyPoint> r_kpts,
                std::vector<cv::Point3f> l_cam_pts);


        /** \brief Get the cluster id.
         *  @return The cluster id.
         */
        inline int getId() const {return id_;}


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


        /** \brief Get camera world position.
         *  @return The camera world position.
         */
        inline tf::Transform getCameraPose() const {return camera_pose_;}


        /** \brief Get cluster right keypoints.
         *  @return The cluster right keypoints.
         */
        inline std::vector<cv::KeyPoint> getRightKpts() const {return r_kpts_;}


        /** \brief Get cluster 3D points relative to left camera.
         *  @return The cluster 3D points relative to left camera.
         */
        inline std::vector<cv::Point3f> getLeftCameraPoints() const {return l_cam_pts_;}


        /** \brief Transforms 3D points relative to the camera to relative to the
         *  world coordinate axis. 
         *  @return The cluster 3D points relative to world.
         */
        std::vector<cv::Point3f> getWorldPoints();
    

    private:

        int id_; //!> Cluster id.

        int keyframe_id_; //!> Corresponding keyframe id.

        cv::Mat l_descs_; //!> Cluster left descriptors.

        cv::Mat r_descs_; //!> Cluster right descriptors.

        tf::Transform camera_pose_; //!> World camera pose when the cluster was obtained.

        std::vector<cv::KeyPoint> l_kpts_; //!> Cluster left keypoints.

        std::vector<cv::KeyPoint> r_kpts_; //!> Cluster right keypoints.

        std::vector<cv::Point3f> l_cam_pts_; //!> Cluster 3D points relative to left camera.

};

#endif // CLUSTER_HPP