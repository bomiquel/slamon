/**
 *  @file
 *  @brief The Frame class represents each stereo camera frame (presentation).
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef FRAME_HPP
#define FRAME_HPP


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
         *  \param l_descs Left descriptors.
         *  \param r_descs Right descriptors.
         *  \param stamp Time stamp in which the frame was obtained.
         *  \param l_kpts Left keypoints.
         *  \param r_kpts Right keypoints.
         *  \param cam_pts 3D points relative to camera.
         */
        Frame(cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> cam_pts);


        /** \brief Full class constructor
         *  \param l_img Left image.
         *  \param r_img Right image.
         *  \param l_descs Left descriptors.
         *  \param r_descs Right descriptors.
         *  \param stamp Time stamp in which the frame was obtained.
         *  \param l_kpts Left keypoints.
         *  \param r_kpts Right keypoints.
         *  \param cam_pts 3D points relative to camera.
         */
        Frame(cv::Mat l_img,
              cv::Mat r_img,
              cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> cam_pts);


        /** \brief Set left image.
         *  \param l_img Left image.
         */
        inline void setLeftImg(const cv::Mat l_img){l_img_ = l_img;}


        /** \brief Set right image.
         *  \param r_img Right image.
         */
        inline void setRightImg(const cv::Mat r_img){r_img_ = r_img;}


        /** \brief Set keyframe timestamp.
         *  \param stamp Keyframe timestamp.
         */
        inline void setTimestamp(const ros::Time stamp){stamp_ = stamp;}


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
        inline std::vector<cv::KeyPoint> getLeftKpts() const {return l_kpts_;}


        /** \brief Get right keypoints.
         *  @return Right keypoints.
         */
        inline std::vector<cv::KeyPoint> getRightKpts() const {return r_kpts_;}
        

        /** \brief Get 3D points relative to camera.
         */
        inline std::vector<cv::Point3f> getCameraPoints() const {return cam_pts_;}


        /** \brief Set camera pose.
         *  \param camera_pose Camera pose when taking the image.
         */
        inline void setCameraPose(const tf::Transform& camera_pose){camera_pose_ = camera_pose;}


        /** \brief Get camera pose
         *  @return The camera pose when taking the image.
         */
        inline tf::Transform getCameraPose() const {return camera_pose_;}


        /** \brief Get the clustering for the current frame.
         *  @return The keypoints clustering.
         */
        inline std::vector<std::vector<int>> getClusters() const {return clusters_;}


        /** \brief Get the clustering centroids.
         *  @return The centroids.
         */
        inline std::vector<cv::Point3f> getClustersCentroids() const {return cluster_centroids_;}


        /** \brief Cluster keypoints
         */
        void regionClustering();


    protected:


        /** \brief Search keypoints into region
         *  \param eps maximum distance to considerate a keypoint into the region.
         *  \param keypoint query keypoint.
         *  \param keypoints list of keypoints.
         *  @return the list of keypoints into a certain region
         */
        std::vector<int> regionQuery(float eps, 
                                     cv::KeyPoint *keypoint, 
                                     std::vector<cv::KeyPoint> *keypoints);


        /** \brief Given a 3D point vector, calculate its centroid.
         *  \param pts The 3D points.
         *  @return The 3D centroid.
         */
        cv::Point3f compute3DCentroid(const std::vector<cv::Point3f>& pts);


    private:

        ros::Time stamp_; //!> Store the frame timestamp.

        cv::Mat l_img_; //!> Left image.
        cv::Mat r_img_; //!> Right image.

        cv::Mat l_descs_; //!> Left descriptors.
        cv::Mat r_descs_; //!> Right descriptors.

        std::vector<cv::KeyPoint> l_kpts_; //!> Left keypoints.
        std::vector<cv::KeyPoint> r_kpts_; //!> Right keypoints.

        std::vector<cv::Point3f> cam_pts_; //!> Stereo 3D points relative to camera.

        tf::Transform camera_pose_; //!> Camera world position for this frame.

        std::vector<std::vector<int>> clusters_; //!> Keypoints clustering.

        std::vector<cv::Point3f> cluster_centroids_; //!> Central point for every cluster.
};


#endif // FRAME_HPP