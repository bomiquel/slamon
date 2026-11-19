/**
 *  \file frame.hpp
 *  \brief Frame class representing a stereo camera frame.
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

 
#ifndef FRAME_HPP
#define FRAME_HPP


#include <vector>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>


/** \class Frame
 *  \brief Represents a stereo camera frame along with its extracted visual features and 3D geometry.
 *  
 *  This class stores the stereo images, 2D keypoints, feature descriptors, triangulated 3D points,
 *  and odometry pose. It also provides spatial clustering routines to group keypoints.
 */
class Frame
{
    public:

        /** \brief Default class constructor.
         */
        Frame() {}


        /** \brief Parameterized constructor (without stereo images).
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] stamp Timestamp at which the frame was captured.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Frame(cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> l_cam_pts);


        /** \brief Full parameterized constructor (including stereo images).
         *  \param[in] l_img Left stereo image.
         *  \param[in] r_img Right stereo image.
         *  \param[in] l_descs Matrix of left image feature descriptors.
         *  \param[in] r_descs Matrix of right image feature descriptors.
         *  \param[in] stamp Timestamp at which the frame was captured.
         *  \param[in] l_kpts Vector of left image 2D keypoints.
         *  \param[in] r_kpts Vector of right image 2D keypoints.
         *  \param[in] l_cam_pts Triangulated 3D points relative to the left camera frame.
         */
        Frame(cv::Mat l_img,
              cv::Mat r_img,
              cv::Mat l_descs,
              cv::Mat r_descs,
              ros::Time stamp,
              std::vector<cv::KeyPoint> l_kpts,
              std::vector<cv::KeyPoint> r_kpts,
              std::vector<cv::Point3f> l_cam_pts);


        /** \brief Sets the frame timestamp.
         *  \param[in] stamp Frame capture timestamp.
         */
        inline void setTimestamp(const ros::Time& stamp) {stamp_ = stamp;}


        /** \brief Sets the left image of the stereo pair.
         *  \param[in] l_img Left image.
         */
        inline void setLeftImg(const cv::Mat& l_img) {l_img_ = l_img;}


        /** \brief Sets the right image of the stereo pair.
         *  \param[in] r_img Right image.
         */
        inline void setRightImg(const cv::Mat& r_img) {r_img_ = r_img;}


        /** \brief Sets the world-odometry camera pose.
         *  \param[in] odom_camera_pose World-odometry camera pose when capturing the frame.
         */
        inline void setCameraPose(const tf2::Transform& odom_camera_pose) {odom_camera_pose_ = odom_camera_pose;}


        /** \brief Gets the number of clusters in the frame.
         *  \return The total number of clusters.
         */
        inline int getClustersNumber() const {return clusters_num_;}


        /** \brief Gets the frame timestamp.
         *  \return The timestamp when the frame was captured.
         */
        inline const ros::Time& getTimestamp() const {return stamp_;}


        /** \brief Gets the left image.
         *  \return The left image of the stereo pair.
         */
        inline cv::Mat getLeftImg() const {return l_img_;}


        /** \brief Gets the right image.
         *  \return The right image of the stereo pair.
         */
        inline cv::Mat getRightImg() const {return r_img_;}


        /** \brief Gets the left feature descriptors.
         *  \return Matrix containing the left descriptors.
         */
        inline cv::Mat getLeftDescs() const {return l_descs_;}


        /** \brief Gets the right feature descriptors.
         *  \return Matrix containing the right descriptors.
         */
        inline cv::Mat getRightDescs() const {return r_descs_;}


        /** \brief Gets the left 2D keypoints.
         *  \return Vector of left keypoints.
         */
        inline const std::vector<cv::KeyPoint>& getLeftKpts() const {return l_kpts_;}


        /** \brief Gets the right 2D keypoints.
         *  \return Vector of right keypoints.
         */
        inline const std::vector<cv::KeyPoint>& getRightKpts() const {return r_kpts_;}
        

        /** \brief Gets the 3D points relative to the left camera.
         *  \return Vector of 3D points in the left camera coordinate frame.
         */
        inline const std::vector<cv::Point3f>& getLeftCameraPoints() const {return l_cam_pts_;}


        /** \brief Gets the world-odometry camera pose.
         *  \return The world-odometry camera pose when the frame was captured.
         */
        inline const tf2::Transform& getCameraPose() const {return odom_camera_pose_;}


        /** \brief Gets the 3D centroids of all clusters in the frame.
         *  \return Vector containing the 3D centroids of the clusters.
         */
        inline const std::vector<cv::Point3f>& getClustersCentroids() const {return cluster_centroids_;}


        /** \brief Checks whether the frame contains any clusters.
         *  \return `true` if the frame contains clusters, `false` otherwise.
         */
        inline bool hasClusters() const {return !cluster_centroids_.empty();}


        /** \brief Gets the mapping between each keypoint index and its assigned cluster ID.
         *  \return Vector of cluster IDs corresponding to each keypoint.
         */
        inline const std::vector<int>& getClusterIds() const {return cluster_ids_;}


        /** \brief Groups the frame keypoints into spatial clusters.
         *  \param[in] c_global_cluster_id Starting global cluster ID to assign to new clusters.
         */
        void regionClustering(int c_global_cluster_id);


        /** \brief Increments all cluster identifiers in the frame by a given offset.
         *  \param[in] increase The offset value to add to each cluster ID.
         */
        void increaseClustersIds(int increase);


    protected:


        /** \brief Searches for neighboring keypoints within a given radius around a query keypoint.
         *  \param[in] eps Maximum Euclidean distance to consider a keypoint within the region [px].
         *  \param[in] keypoint Pointer to the query keypoint.
         *  \param[in] keypoints Pointer to the complete list of candidate keypoints.
         *  \return Vector of indices of the keypoints found within the region.
         */
        std::vector<int> regionQuery(float eps, 
                                     cv::KeyPoint *keypoint, 
                                     std::vector<cv::KeyPoint> *keypoints);


        /** \brief Computes the 3D geometric centroid of a given set of 3D points.
         *  \param[in] pts Vector of 3D points.
         *  \return The computed 3D centroid point.
         */
        cv::Point3f compute3DCentroid(const std::vector<cv::Point3f>& pts);


    private:

        /** \brief Total number of clusters in the frame. */
        int clusters_num_;

        /** \brief Frame capture timestamp. */
        ros::Time stamp_;

        /** \brief Left stereo image. */
        cv::Mat l_img_;

        /** \brief Right stereo image. */
        cv::Mat r_img_;

        /** \brief Matrix of left image feature descriptors. */
        cv::Mat l_descs_;

        /** \brief Matrix of right image feature descriptors. */
        cv::Mat r_descs_;

        /** \brief World-odometry camera pose for this frame. */
        tf2::Transform odom_camera_pose_;

        /** \brief Mapping between each keypoint index and the cluster ID to which it belongs. */
        std::vector<int> cluster_ids_;

        /** \brief Vector of left image 2D keypoints. */
        std::vector<cv::KeyPoint> l_kpts_;

        /** \brief Vector of right image 2D keypoints. */
        std::vector<cv::KeyPoint> r_kpts_;

        /** \brief Triangulated stereo 3D points relative to the left camera coordinate frame. */
        std::vector<cv::Point3f> l_cam_pts_;

        /** \brief 3D centroid point for each cluster in the frame. */
        std::vector<cv::Point3f> cluster_centroids_;
};


#endif // FRAME_HPP