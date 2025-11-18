/**
 *  @file frame.hpp
 *  @brief Frame class representing a stereo camera frame.
 *  @package slamon
 *  @project SLAMON
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  @author
 *    Original:  Pep Lluís Negre-Carrasco (2013)
 *    Modified:  Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    Both the original work and the modifications are released under the
 *    BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#ifndef FRAME_HPP
#define FRAME_HPP


#include <vector>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf/tf.h>


class Frame
{
    public:

        /** \brief Empty class constructor
         */
        Frame() {};


        /** \brief Class constructor without images.
         *  \param[in] l_descs Left descriptors.
         *  \param[in] r_descs Right descriptors.
         *  \param[in] stamp Time stamp in which the frame was obtained.
         *  \param[in] l_kpts Left keypoints.
         *  \param[in] r_kpts Right keypoints.
         *  \param[in] l_cam_pts 3D points relative to left camera.
         */
        Frame(cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> l_cam_pts);


        /** \brief Full class constructor
         *  \param[in] l_img Left image.
         *  \param[in] r_img Right image.
         *  \param[in] l_descs Left descriptors.
         *  \param[in] r_descs Right descriptors.
         *  \param[in] stamp Time stamp in which the frame was obtained.
         *  \param[in] l_kpts Left keypoints.
         *  \param[in] r_kpts Right keypoints.
         *  \param[in] l_cam_pts 3D points relative to left camera.
         */
        Frame(cv::Mat l_img,
              cv::Mat r_img,
              cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> l_cam_pts);


        /** \brief Set keyframe timestamp.
         *  \param[in] stamp Keyframe timestamp.
         */
        inline void setTimestamp(const ros::Time stamp) {stamp_ = stamp;}


        /** \brief Set left image.
         *  \param[in] l_img Left image.
         */
        inline void setLeftImg(const cv::Mat l_img) {l_img_ = l_img;}


        /** \brief Set right image.
         *  \param[in] r_img Right image.
         */
        inline void setRightImg(const cv::Mat r_img) {r_img_ = r_img;}


        /** \brief Set world-odom camera pose.
         *  \param[in] odom_camera_pose World-odom camera pose when taking the image.
         */
        inline void setCameraPose(const tf::Transform& odom_camera_pose){odom_camera_pose_ = odom_camera_pose;}


        /** \brief Gets the number of clusters.
         *  @return The clusters number.
         */
        inline int getClustersNumber() const {return clusters_num_;}


        /** \brief Get keyframe timestamp.
         *  @return The time stamp when taking the image.
         */
        inline ros::Time getTimestamp() const {return stamp_;}


        /** \brief Get left image.
         *  @return The left image of the stereo pair
         */
        inline cv::Mat getLeftImg() const {return l_img_;}


        /** \brief Get right image.
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
        

        /** \brief Get 3D points relative to left camera.
         */
        inline std::vector<cv::Point3f> getLeftCameraPoints() const {return l_cam_pts_;}


        /** \brief Get world-odom camera pose.
         *  @return The world-odom camera pose when taking the image.
         */
        inline tf::Transform getCameraPose() const {return odom_camera_pose_;}


        /** \brief Get the cluster centroids.
         *  @return The centroids.
         */
        inline std::vector<cv::Point3f> getClustersCentroids() const {return cluster_centroids_;}


        /** \brief Does the frame have clusters?
         *  @return if it contains clusters.
         */
        inline bool hasClusters() const {return !cluster_centroids_.empty();}


        /** \brief Get the relationship between the keypoint index and the cluster to which it belongs.
         *  @return Cluster ids.
         */
        inline std::vector<int> getClusterIds() const {return cluster_ids_;}


        /** \brief Clustering keypoints.
         *  \param[in] c_global_cluster_id
         */
        void regionClustering(int c_global_cluster_id);


        /** \brief Increases all cluster identifiers by a given value.
         *  \param[in] increase The value to add to each cluster identifier.
         */
        void increaseClustersIds(int increase);


    protected:


        /** \brief Search keypoints into region
         *  \param[in] eps maximum distance to considerate a keypoint into the region.
         *  \param[in] keypoint query keypoint.
         *  \param[in] keypoints list of keypoints.
         *  @return the list of keypoints into a certain region
         */
        std::vector<int> regionQuery(float eps, 
                                     cv::KeyPoint *keypoint, 
                                     std::vector<cv::KeyPoint> *keypoints);


        /** \brief Given a 3D point vector, calculate its centroid.
         *  \param[in] pts The 3D points.
         *  @return The 3D centroid.
         */
        cv::Point3f compute3DCentroid(const std::vector<cv::Point3f>& pts);


    private:

        // Attributes:

        int clusters_num_; //!> Stores the number of clusters.

        ros::Time stamp_; //!> Store the frame timestamp.

        cv::Mat l_img_; //!> Left image.
        cv::Mat r_img_; //!> Right image.

        cv::Mat l_descs_; //!> Left descriptors.
        cv::Mat r_descs_; //!> Right descriptors.

        tf::Transform odom_camera_pose_; //!> World-odom camera pose for this frame.

        std::vector<int> cluster_ids_; //!> Relationship between the keypoint index and the cluster to which it belongs.

        std::vector<cv::KeyPoint> l_kpts_; //!> Left keypoints.
        std::vector<cv::KeyPoint> r_kpts_; //!> Right keypoints.

        std::vector<cv::Point3f> l_cam_pts_; //!> Stereo 3D points relative to left camera.

        std::vector<cv::Point3f> cluster_centroids_; //!> Central point for every cluster.
};


#endif // FRAME_HPP