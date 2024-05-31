/**
 *  @file
 *  @brief The Cluster class represents one keypoints clustering of the camera frame.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
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
         *  \param frame_id Frame id.
         *  \param l_descs Cluster left descriptors.
         *  \param r_descs Cluster right descriptors.
         *  \param camera_pose World camera pose when the cluster was obtained.
         *  \param l_kpts Cluster left keypoints.
         *  \param r_kpts Cluster right keypoints.
         *  \param obj_pts Cluster 3D points.
         */
        Cluster(int id,
                int frame_id,
                cv::Mat l_descs,
                cv::Mat r_descs,
                tf::Transform camera_pose,
                std::vector<cv::Point2f> l_kpts,
                std::vector<cv::Point2f> r_kpts,
                std::vector<cv::Point3f> obj_pts);


        /** \brief Get the cluster id.
         *  @return The cluster id.
         */
        inline int getId() const {return id_;}


        /** \brief Get the corresponding frame id.
         *  @return The corresponding frame id.
         */
        inline int getFrameId() const {return frame_id_;}


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
        inline std::vector<cv::Point2f> getLeftKpts() const {return l_kpts_;}


        /** \brief Get camera world position.
         *  @return The camera world position.
         */
        inline tf::Transform getCameraPose() const {return camera_pose_;}


        /** \brief Get cluster right keypoints.
         *  @return The cluster right keypoints.
         */
        inline std::vector<cv::Point2f> getRightKpts() const {return r_kpts_;}


        /** \brief Get cluster 3D points.
         *  @return The cluster 3D points.
         */
        inline std::vector<cv::Point3f> getObjectPoints() const {return obj_pts_;}
    

    private:

        int id_; //!> Cluster id.

        int frame_id_; //!> Corresponding frame id.

        cv::Mat l_descs_; //!> Cluster left descriptors.

        cv::Mat r_descs_; //!> Cluster right descriptors.

        tf::Transform camera_pose_; //!> World camera pose when the cluster was obtained.

        std::vector<cv::Point2f> l_kpts_; //!> Cluster left keypoints.

        std::vector<cv::Point2f> r_kpts_; //!> Cluster right keypoints.

        std::vector<cv::Point3f> obj_pts_; //!> Cluster 3D points.

};

#endif // CLUSTER_HPP