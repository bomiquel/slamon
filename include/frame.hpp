/**
 *  @file
 *  @brief The Frame class represents each stereo camera frame (presentation).
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef FRAME_H
#define FRAME_H


#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>


class Frame
{
    public:

        /** \brief Empty class constructor
         */
        Frame() {};


        /** \brief Class constructor without images.
         * 
         */
        Frame(const cv::Mat& l_descs,
              const cv::Mat& r_descs,
              const ros::Time& stamp,
              const std::vector<cv::Point2f>& l_kpts,
              const std::vector<cv::Point2f>& r_kpts,
              const std::vector<cv::Point3f>& obj_pts);


        /** \brief Full class constructor
         */
        Frame(const cv::Mat& l_img,
              const cv::Mat& r_img,
              const cv::Mat& l_descs,
              const cv::Mat& r_descs,
              const ros::Time& stamp,
              const std::vector<cv::Point2f>& l_kpts,
              const std::vector<cv::Point2f>& r_kpts,
              const std::vector<cv::Point3f>& obj_pts);


        /** \brief Set keyframe id.
         *  \param id of the keyframe.
         */
        inline void setId(const int id){id_ = id;}


        /** \brief Get keyframe id.
         *  @return keyframe id.
         */
        inline int getId() const {return id_;}


        /** \brief Get keyframe timestamp
         *  @return The time stamp when taking the image.
         */
        inline ros::Time getTimestamp() const {return stamp_;}


        /** \brief Get left image
         *  @return The left image of the stereo pair
         */
        inline cv::Mat getLeftImg() const {return l_img_;}


        /** \brief Get right image
         *  @return The right image of the stereo pair
         */
        inline cv::Mat getRightImg() const {return r_img_;}


        /** \brief Get left descriptors.
         *  @return Left descriptors.
         */
        inline cv::Mat getLeftDescs() const {return l_descs_;}


        /** \brief Get right descriptors.
         *  @return Right descriptors.
         */
        inline cv::Mat getRightDescs() const {return r_descs_;}


        /** \brief Get left keypoints.
         *  @return Left keypoints.
         */
        inline std::vector<cv::Point2f> getLeftKpts() const {return l_kpts_;}


        /** \brief Get right keypoints.
         *  @return Right keypoints.
         */
        inline std::vector<cv::Point2f> getRightKpts() const {return r_kpts_;}
        

        /** \brief Get 3D in camera frame.
         */
        inline std::vector<cv::Point3f> getObjectPoints() const {return obj_pts_;}


        /** \brief Set camera pose.
         *  \param camera_pose Camera pose when taking the image.
         */
        inline void setCameraPose(const tf::Transform& camera_pose){camera_pose_ = camera_pose;}


        /** \brief Get camera pose
         *  @return The camera pose when taking the image.
         */
        inline tf::Transform getCameraPose() const {return camera_pose_;}


    private:

        int id_; //!> Frame id.

        ros::Time stamp_; //!> Store the frame timestamp.

        cv::Mat l_img_; //!> Left image.
        cv::Mat r_img_; //!> Right image.

        cv::Mat l_descs_; //!> Left descriptors.
        cv::Mat r_descs_; //!> Right descriptors.

        std::vector<cv::Point2f> l_kpts_; //!> Left keypoints.
        std::vector<cv::Point2f> r_kpts_; //!> Right keypoints.

        std::vector<cv::Point3f> obj_pts_; //!> Stereo 3D points in camera frame.

        tf::Transform camera_pose_; //!> Camera world position for this frame.

        // std::vector<std::vector<int>> clusters_; //!> Keypoints clustering.

        // std::vector<Eigen::Vector4f> cluster_centroids_; //!> Central point for every cluster.
};


#endif // FRAME_H