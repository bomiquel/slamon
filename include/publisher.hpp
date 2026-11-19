/**
 *  \file publisher.hpp
 *  \brief The Publisher class is responsible for publishing ROS messages and saving 
 *         image visualizations and trajectory results for debugging and evaluation.
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

 
#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "slamon/Keypoint.h"
#include "slamon/Keyframe.h"
#include "slamon/Performance.h"


/** \class Publisher
 *  \brief Provides utilities for publishing ROS messages and saving debug visualizations and results to disk.
 *  
 *  This base class is inherited by the main SLAMON modules (Visual Odometry, Mapmaker, and Loop Closer)
 *  to publish odometry, poses, keyframes, execution runtimes, and debug images, as well as to write 
 *  trajectory poses, loop closure edges, and feature-matching visualizations to disk.
 */
class Publisher
{
    public:

        /** \brief Default class constructor.
         */
        Publisher() {}


        /** \struct RunTimes
         *  \brief Structure for storing per-stage execution runtimes across SLAMON modules.
         *  
         *  To avoid defining a separate timing structure for each subsystem, this structure is 
         *  shared and reused across all three main threads: Visual Odometry (VO), Mapmaker (MM), 
         *  and Loop Closer (LC).
         */
        struct RunTimes
        {
            /** \brief Execution time for stage 0 (VO: Feature extraction | MM: Pose update | LC: Process current cluster) [s]. */
            double run_time_0 = 0.0;

            /** \brief Execution time for stage 1 (VO: Stereo matching | MM: Add vertex to graph | LC: Proximity candidate search) [s]. */
            double run_time_1 = 0.0;

            /** \brief Execution time for stage 2 (VO: Triangulation | MM: Add intra-keyframe edges | LC: Hash candidate search) [s]. */
            double run_time_2 = 0.0;

            /** \brief Execution time for stage 3 (VO: Clustering | MM: Add inter-frame edges | LC: Read candidate clusters) [s]. */
            double run_time_3 = 0.0;

            /** \brief Execution time for stage 4 (VO: Temporal matching | MM: Publish messages | LC: Loop detection) [s]. */
            double run_time_4 = 0.0;

            /** \brief Execution time for stage 5 (VO: Keypoint bucketing | MM: Write map data to disk | LC: Loop correction) [s]. */
            double run_time_5 = 0.0;

            /** \brief Execution time for stage 6 (VO: Motion estimation | MM: Unused | LC: Unused) [s]. */
            double run_time_6 = 0.0;

            /** \brief Execution time for stage 7 (VO: Keyframe insertion decision | MM: Unused | LC: Unused) [s]. */
            double run_time_7 = 0.0;

            /** \brief Total execution time required by the thread to complete one full iteration [s]. */
            double total_run_time = 0.0;

            /** \brief Resets all recorded execution times to zero.
             */
            void reset()
            {
                run_time_0 = 0.0;
                run_time_1 = 0.0;
                run_time_2 = 0.0;
                run_time_3 = 0.0;
                run_time_4 = 0.0;
                run_time_5 = 0.0;
                run_time_6 = 0.0;
                run_time_7 = 0.0;
                total_run_time = 0.0;
            }
        };


        /** \brief Draws keypoints on an image and saves the resulting visualization to disk.
         *  \param[in] img Input image.
         *  \param[in] color Color used to draw the keypoints.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] kpts Vector of 2D keypoints to draw.
         */
        void writeImgKeypoints(const cv::Mat& img,
                               const cv::Scalar& color,
                               const std::string& o_path,
                               const std::vector<cv::KeyPoint>& kpts);


        /** \brief Draws keypoints on a stereo image pair and publishes the concatenated visualization via ROS.
         *  \param[in] l_img Left stereo image.
         *  \param[in] r_img Right stereo image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] l_kpts Left image 2D keypoints.
         *  \param[in] r_kpts Right image 2D keypoints.
         */
        void publishImgStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const ros::Time& stamp,
                                       const ros::Publisher& publisher,
                                       const std::vector<cv::KeyPoint>& l_kpts,
                                       const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Draws keypoints on a stereo image pair and saves the concatenated visualization to disk.
         *  \param[in] l_img Left stereo image.
         *  \param[in] r_img Right stereo image.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] l_kpts Left image 2D keypoints.
         *  \param[in] r_kpts Right image 2D keypoints.
         */
        void writeImgStereoKeypoints(const cv::Mat& l_img,
                                     const cv::Mat& r_img,
                                     const std::string& o_path,
                                     const std::vector<cv::KeyPoint>& l_kpts,
                                     const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Draws vertical (temporal) or horizontal (stereo) keypoint matches between two images and publishes the visualization via ROS.
         *  \param[in] vertical If `true`, concatenates the images vertically (temporal matching); if `false`, concatenates them horizontally (stereo matching).
         *  \param[in] img_0 First image (current image in temporal matching; left image in stereo matching).
         *  \param[in] img_1 Second image (previous image in temporal matching; right image in stereo matching).
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] m_kpts_0 Matched and aligned keypoints from the first image.
         *  \param[in] m_kpts_1 Matched and aligned keypoints from the second image.
         */
        void publishImgMatches(const bool vertical, 
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const ros::Time& stamp,
                               const ros::Publisher& publisher,
                               const std::vector<cv::KeyPoint>& m_kpts_0,
                               const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Draws vertical (temporal) or horizontal (stereo) keypoint matches between two images and saves the visualization to disk.
         *  \param[in] vertical If `true`, concatenates the images vertically (temporal matching); if `false`, concatenates them horizontally (stereo matching).
         *  \param[in] img_0 First image (current image in temporal matching; left image in stereo matching).
         *  \param[in] img_1 Second image (previous image in temporal matching; right image in stereo matching).
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] m_kpts_0 Matched and aligned keypoints from the first image.
         *  \param[in] m_kpts_1 Matched and aligned keypoints from the second image.
         */
        void writeImgMatches(const bool vertical, 
                             const cv::Mat& img_0,
                             const cv::Mat& img_1,
                             const std::string& o_path,
                             const std::vector<cv::KeyPoint>& m_kpts_0,
                             const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Draws circular stereo-temporal keypoint matches across four images and publishes the visualization via ROS.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] m_c_l_kpts Matched and aligned current left keypoints.
         *  \param[in] m_c_r_kpts Matched and aligned current right keypoints.
         *  \param[in] m_p_l_kpts Matched and aligned previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and aligned previous right keypoints.
         */
        void publishImgCircularMatches(const cv::Mat& c_l_img,                                    
                                       const cv::Mat& c_r_img,
                                       const cv::Mat& p_l_img,
                                       const cv::Mat& p_r_img,
                                       const ros::Time& stamp,
                                       const ros::Publisher& publisher,
                                       const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                       const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                       const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                       const std::vector<cv::KeyPoint>& m_p_r_kpts);


        /** \brief Draws circular stereo-temporal keypoint matches across four images and saves the visualization to disk.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] m_c_l_kpts Matched and aligned current left keypoints.
         *  \param[in] m_c_r_kpts Matched and aligned current right keypoints.
         *  \param[in] m_p_l_kpts Matched and aligned previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and aligned previous right keypoints.
         */
        void writeImgCircularMatches(const cv::Mat& c_l_img,
                                     const cv::Mat& c_r_img,
                                     const cv::Mat& p_l_img,
                                     const cv::Mat& p_r_img,
                                     const std::string& o_path,
                                     const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                     const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                     const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                     const std::vector<cv::KeyPoint>& m_p_r_kpts);


        /** \brief Draws spatially clustered keypoints (from grouped keypoint vectors) and publishes the visualization via ROS.
         *  \param[in] l_img Left stereo image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] l_kpts Vector of clusters, where each element is a vector of left keypoints belonging to that cluster.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const ros::Publisher& publisher,
                                           const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws spatially clustered keypoints (from cluster index lists) and publishes the visualization via ROS.
         *  \param[in] l_img Left stereo image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] l_kpts Vector of left image keypoints.
         *  \param[in] clusters Vector of clusters, where each element contains the keypoint indices belonging to that cluster.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const ros::Publisher& publisher,
                                           const std::vector<cv::KeyPoint>& l_kpts,
                                           const std::vector<std::vector<int>>& clusters);


        /** \brief Draws spatially clustered keypoints (from per-keypoint cluster IDs) and publishes the visualization via ROS.
         *  \param[in] l_img Left stereo image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] cluster_num Total number of clusters.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] cluster_ids Vector mapping each keypoint index to its assigned cluster ID.
         *  \param[in] l_kpts Vector of left image keypoints.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const size_t cluster_num,
                                           const ros::Publisher& publisher,
                                           const std::vector<int>& cluster_ids,
                                           const std::vector<cv::KeyPoint>& l_kpts);


        /** \brief Draws spatially clustered keypoints (from grouped keypoint vectors) and saves the visualization to disk.
         *  \param[in] l_img Left stereo image.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] l_kpts Vector of clusters, where each element is a vector of left keypoints belonging to that cluster.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const std::string& o_path,
                                         const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws spatially clustered keypoints (from cluster index lists) and saves the visualization to disk.
         *  \param[in] l_img Left stereo image.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] l_kpts Vector of left image keypoints.
         *  \param[in] clusters Vector of clusters, where each element contains the keypoint indices belonging to that cluster.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const std::string& o_path,
                                         const std::vector<cv::KeyPoint>& l_kpts,
                                         const std::vector<std::vector<int>>& clusters);


        /** \brief Draws spatially clustered keypoints (from per-keypoint cluster IDs) and saves the visualization to disk.
         *  \param[in] l_img Left stereo image.
         *  \param[in] cluster_num Total number of clusters.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] cluster_ids Vector mapping each keypoint index to its assigned cluster ID.
         *  \param[in] l_kpts Vector of left image keypoints.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const size_t cluster_num,
                                         const std::string& o_path,
                                         const std::vector<int>& cluster_ids,
                                         const std::vector<cv::KeyPoint>& l_kpts);


        /** \brief Draws and publishes a loop closure visualization image showing all feature correspondences
         *         between the query keyframe and all matched loop-closing candidate keyframes.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         *  \param[in] q_keyframe_id ID of the keyframe to which the query cluster belongs.
         *  \param[in] i_path Directory path where keyframe images are stored on disk.
         *  \param[in] q_cluster_ids IDs of the query clusters involved in the loop closure.
         *  \param[in] cand_cluster_ids IDs of the candidate clusters involved in the loop closure.
         *  \param[in] cand_keyframe_ids IDs of the keyframes containing the candidate clusters that closed the loop.
         *  \param[in] f_q_l_kpts Filtered (inlier) query left keypoints.
         *  \param[in] f_q_r_kpts Filtered (inlier) query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered (inlier) candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered (inlier) candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair Number of geometrical inliers for each loop-closing cluster pair.
         *  \param[in] definitive_cluster_pairs Vector of matched cluster ID pairs (`q_cluster_id`, `cand_cluster_id`).
         *  \param[in] cand_cluster_keyframe_relations Vector of pairs mapping candidate cluster IDs to their keyframe IDs (`cand_cluster_id`, `cand_keyframe_id`).
         */
        void publishImgLoopClosure(const ros::Time& stamp,
                                   const ros::Publisher& publisher,
                                   const int q_keyframe_id,
                                   const std::string& i_path,
                                   const std::vector<int>& q_cluster_ids,
                                   const std::vector<int>& cand_cluster_ids,
                                   const std::vector<int>& cand_keyframe_ids,
                                   const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                   const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                   const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                   const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                   const std::vector<int>& definitive_inliers_per_pair,
                                   const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                   const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations);


        /** \brief Draws and saves to disk a loop closure visualization image showing all feature correspondences
         *         between the query keyframe and all matched loop-closing candidate keyframes.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] q_keyframe_id ID of the keyframe to which the query cluster belongs.
         *  \param[in] i_path Directory path where keyframe images are stored on disk.
         *  \param[in] q_cluster_ids IDs of the query clusters involved in the loop closure.
         *  \param[in] cand_cluster_ids IDs of the candidate clusters involved in the loop closure.
         *  \param[in] cand_keyframe_ids IDs of the keyframes containing the candidate clusters that closed the loop.
         *  \param[in] f_q_l_kpts Filtered (inlier) query left keypoints.
         *  \param[in] f_q_r_kpts Filtered (inlier) query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered (inlier) candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered (inlier) candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair Number of geometrical inliers for each loop-closing cluster pair.
         *  \param[in] definitive_cluster_pairs Vector of matched cluster ID pairs (`q_cluster_id`, `cand_cluster_id`).
         *  \param[in] cand_cluster_keyframe_relations Vector of pairs mapping candidate cluster IDs to their keyframe IDs (`cand_cluster_id`, `cand_keyframe_id`).
         */
        void writeImgLoopClosure(const std::string& o_path,
                                 const int q_keyframe_id,
                                 const std::string& i_path,
                                 const std::vector<int>& q_cluster_ids,
                                 const std::vector<int>& cand_cluster_ids,
                                 const std::vector<int>& cand_keyframe_ids,
                                 const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                 const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                 const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                 const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                 const std::vector<int>& definitive_inliers_per_pair,
                                 const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                 const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations);


        /** \brief Publishes a 6-DoF pose as a `nav_msgs::Odometry` message.
         *  \param[in] c_stamp Current pose timestamp.
         *  \param[in] c_pose Current 6-DoF pose to be published.
         *  \param[in] bad_pose_covariance Flag indicating whether pose estimation failed (assigns high covariance).
         *  \param[in] publisher Target ROS odometry publisher.
         *  \param[in] reference_frame_id ID of the coordinate frame to which the pose is relative.
         */
        void publishPose(const ros::Time& c_stamp,
                         const tf2::Transform& c_pose,
                         const bool bad_pose_covariance,
                         const ros::Publisher& publisher,
                         const std::string& reference_frame_id);


        /** \brief Publishes a 6-DoF pose and linear/angular velocity twist as a `nav_msgs::Odometry` message.
         *  \param[in] c_stamp Current frame ROS timestamp.
         *  \param[in] p_stamp Previous frame ROS timestamp.
         *  \param[in] c_pose Current 6-DoF pose to be published.
         *  \param[in] c_motion Relative 6-DoF motion between the previous and current frames (used to compute velocity).
         *  \param[in] bad_pose_covariance Flag indicating whether pose estimation failed (assigns high pose covariance).
         *  \param[in] bad_twist_covariance Flag indicating whether velocity estimation failed (assigns high twist covariance).
         *  \param[in] publisher Target ROS odometry publisher.
         *  \param[in] reference_frame_id ID of the coordinate frame to which the pose and velocity are relative.
         */
        void publishPoseAndVelocity(const ros::Time& c_stamp,
                                    const ros::Time& p_stamp,
                                    const tf2::Transform& c_pose,
                                    const tf2::Transform& c_motion,
                                    const bool bad_pose_covariance,
                                    const bool bad_twist_covariance,
                                    const ros::Publisher& publisher,
                                    const std::string& reference_frame_id);


        /** \brief Publishes per-iteration execution performance metrics via ROS.
         *  \param[in] stamp Message timestamp (VO: current frame timestamp | MM: current time | LC: current time).
         *  \param[in] successes_num Number of successful operations in the last iteration of the thread.
         *  \param[in] run_times Structure containing all per-stage execution times of the thread.
         *  \param[in] publisher Target ROS performance publisher.
         */
        void publishPerformance(const ros::Time& stamp,
                                const int successes_num,
                                const RunTimes& run_times,
                                const ros::Publisher& publisher);


        /** \brief Publishes pointcloud.
         *  \param[in] publisher Target ROS performance publisher.
         *  \param[in] cloud_msg ROS message containing the pointcloud.
         */
        void publishCloud(const ros::Publisher& publisher,
                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


        /** \brief Builds and publishes a custom `slamon::Keyframe` message (without stereo images) via ROS.
         *  \param[in] keyframe_id Unique keyframe ID.
         *  \param[in] l_descs Matrix of keyframe left feature descriptors.
         *  \param[in] r_descs Matrix of keyframe right feature descriptors.
         *  \param[in] stamp Keyframe capture ROS timestamp.
         *  \param[in] publisher Target ROS keyframe publisher.
         *  \param[in] inliers Number of matched inliers per cluster pair.
         *  \param[in] cluster_ids Vector mapping each keypoint to its cluster ID.
         *  \param[in] l_kpts Vector of keyframe left 2D keypoints.
         *  \param[in] r_kpts Vector of keyframe right 2D keypoints.
         *  \param[in] l_cam_pts Vector of keyframe 3D points in the left camera frame.
         *  \param[in] cluster_centroids Vector of 3D centroids for each cluster in the keyframe.
         *  \param[in] cluster_pairs Vector of matched cluster ID pairs.
         */
        void publishKeyframe(const int keyframe_id,
                             const cv::Mat& l_descs,
                             const cv::Mat& r_descs,
                             const ros::Time& stamp,
                             const ros::Publisher& publisher,
                             const std::vector<int>& inliers,
                             const std::vector<int>& cluster_ids,
                             const std::vector<cv::KeyPoint>& l_kpts,
                             const std::vector<cv::KeyPoint>& r_kpts,
                             const std::vector<cv::Point3f>& l_cam_pts,
                             const std::vector<cv::Point3f>& cluster_centroids,
                             const std::vector<std::pair<int, int>>& cluster_pairs);


        /** \brief Builds and publishes a custom `slamon::Keyframe` message (including stereo images) via ROS.
         *  \param[in] keyframe_id Unique keyframe ID.
         *  \param[in] l_img Keyframe left stereo image.
         *  \param[in] r_img Keyframe right stereo image.
         *  \param[in] l_descs Matrix of keyframe left feature descriptors.
         *  \param[in] r_descs Matrix of keyframe right feature descriptors.
         *  \param[in] stamp Keyframe capture ROS timestamp.
         *  \param[in] publisher Target ROS keyframe publisher.
         *  \param[in] inliers Number of matched inliers per cluster pair.
         *  \param[in] cluster_ids Vector mapping each keypoint to its cluster ID.
         *  \param[in] l_kpts Vector of keyframe left 2D keypoints.
         *  \param[in] r_kpts Vector of keyframe right 2D keypoints.
         *  \param[in] l_cam_pts Vector of keyframe 3D points in the left camera frame.
         *  \param[in] cluster_centroids Vector of 3D centroids for each cluster in the keyframe.
         *  \param[in] cluster_pairs Vector of matched cluster ID pairs.
         */
        void publishKeyframe(const int keyframe_id,
                             const cv::Mat& l_img,
                             const cv::Mat& r_img,
                             const cv::Mat& l_descs,
                             const cv::Mat& r_descs,
                             const ros::Time& stamp,
                             const ros::Publisher& publisher,
                             const std::vector<int>& inliers,
                             const std::vector<int>& cluster_ids,
                             const std::vector<cv::KeyPoint>& l_kpts,
                             const std::vector<cv::KeyPoint>& r_kpts,
                             const std::vector<cv::Point3f>& l_cam_pts,
                             const std::vector<cv::Point3f>& cluster_centroids,
                             const std::vector<std::pair<int, int>>& cluster_pairs);


        /** \brief Publishes an integer counter value as a `std_msgs::Int32` message via ROS.
         *  \param[in] counter Integer value to be published.
         *  \param[in] publisher Target ROS publisher.
         */
        void publishCounter(const int counter,
                            const ros::Publisher& publisher);


        /** \brief Writes a list of stamped 6-DoF poses to a text file on disk.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] stamps Vector of pose timestamps.
         *  \param[in] poses Vector of 6-DoF poses to be written to the file.
         */
        void writePoses(const std::string& o_path,
                        const std::vector<double>& stamps,
                        const std::vector<tf2::Transform>& poses);


        /** \brief Writes a list of loop closure graph edges and their poses to a text file on disk.
         *  \param[in] o_path Output file path (including filename and extension).
         *  \param[in] weights Scalar list reflecting the total confidence of each loop closure edge.
         *  \param[in] edges_ids List of keyframe ID pairs connected by a loop closure.
         *  \param[in] edges_poses List of 6-DoF pose pairs for the keyframes connected by a loop closure.
         */
        void writeLoopClosureEdges(const std::string& o_path,
                                   const std::vector<double>& weights,
                                   const std::vector<std::pair<int, int>>& edges_ids,
                                   const std::vector<std::pair<tf2::Transform, tf2::Transform>>& edges_poses);


        /** \brief Formats an integer as a zero-padded five-digit string.
         *  \param[in] in Input integer.
         *  \return Zero-padded five-digit string representation.
         */
        std::string convertTo5Digits(int in);


    private:


        /** \brief Draws 2D keypoints on an image.
         *  \param[in] img Input image.
         *  \param[in] color Desired color used to draw the keypoints.
         *  \param[in] kpts Vector of 2D keypoints to draw.
         *  \return Output image with the keypoints drawn.
         */
        cv::Mat drawImgKeypoints(const cv::Mat& img,
                                 const cv::Scalar& color,
                                 const std::vector<cv::KeyPoint>& kpts);


        /** \brief Renders a text banner at the bottom of an image.
         *  \param[in] caption If `true`, formats the text as a caption; otherwise, formats it as a title.
         *  \param[in] img Input image.
         *  \param[in] text String stream containing the text to render.
         *  \param[in] extra_vertical_space Additional vertical padding to prevent text from touching margins [px].
         *  \return Output image with the text banner appended at the bottom.
         */
        cv::Mat drawTxtInImg(const bool caption,
                             const cv::Mat& img,
                             const std::stringstream& text,
                             const int extra_vertical_space);


        /** \brief Concatenates a stereo image pair horizontally and draws their respective keypoints.
         *  \param[in] l_img Left stereo image.
         *  \param[in] r_img Right stereo image.
         *  \param[in] color Color used to draw the keypoints.
         *  \param[in] l_kpts Left image 2D keypoints.
         *  \param[in] r_kpts Right image 2D keypoints.
         *  \return Concatenated stereo image showing the drawn keypoints and a text caption with the keypoint count.
         */
        cv::Mat drawImgStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const cv::Scalar& color,
                                       const std::vector<cv::KeyPoint>& l_kpts,
                                       const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Concatenates two images vertically or horizontally and draws the keypoint matches between them.
         *  \param[in] vertical If `true`, concatenates the images vertically (temporal matching); if `false`, concatenates them horizontally (stereo matching).
         *  \param[in] img_0 First image (current image in temporal matching; left image in stereo matching).
         *  \param[in] img_1 Second image (previous image in temporal matching; right image in stereo matching).
         *  \param[in] color Color used to draw the keypoints and match lines.
         *  \param[in] m_kpts_0 Matched and aligned keypoints from the first image.
         *  \param[in] m_kpts_1 Matched and aligned keypoints from the second image.
         *  \return Concatenated image with the drawn keypoint correspondences.
         */
        cv::Mat drawImgMatches(const bool vertical,
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const cv::Scalar& color,
                               const std::vector<cv::KeyPoint>& m_kpts_0,
                               const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Concatenates current and previous stereo pairs and draws circular keypoint matches across all four images.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] color Color used to draw the keypoints and match lines.
         *  \param[in] m_c_l_kpts Matched and aligned current left keypoints.
         *  \param[in] m_c_r_kpts Matched and aligned current right keypoints.
         *  \param[in] m_p_l_kpts Matched and aligned previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and aligned previous right keypoints.
         *  \return Composite 2x2 image showing both stereo pairs and their circular matches.
         */
        cv::Mat drawImgCircularMatches(const cv::Mat& c_l_img,                                    
                                       const cv::Mat& c_r_img,
                                       const cv::Mat& p_l_img,
                                       const cv::Mat& p_r_img,
                                       const cv::Scalar& color,
                                       const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                       const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                       const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                       const std::vector<cv::KeyPoint>& m_p_r_kpts);


        /** \brief Draws spatially clustered keypoints on an image (from grouped keypoint vectors).
         *  \param[in] img Input image on which to draw the clusters.
         *  \param[in] l_kpts Vector of clusters, where each element is a vector of keypoints belonging to that cluster.
         *  \return Output image with color-coded keypoint clusters drawn.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws spatially clustered keypoints on an image (from cluster index lists).
         *  \param[in] img Input image on which to draw the clusters.
         *  \param[in] kpts Vector of 2D keypoints.
         *  \param[in] clusters Vector of clusters, where each element contains the keypoint indices belonging to that cluster.
         *  \return Output image with color-coded keypoint clusters drawn.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const std::vector<cv::KeyPoint>& kpts,
                                           const std::vector<std::vector<int>>& clusters);


        /** \brief Draws spatially clustered keypoints on an image (from per-keypoint cluster IDs).
         *  \param[in] img Input image on which to draw the clusters.
         *  \param[in] cluster_num Total number of clusters.
         *  \param[in] cluster_ids Vector mapping each keypoint index to its assigned cluster ID.
         *  \param[in] kpts Vector of 2D keypoints.
         *  \return Output image with color-coded keypoint clusters drawn.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const size_t cluster_num,
                                           const std::vector<int>& cluster_ids,
                                           const std::vector<cv::KeyPoint>& kpts);


        /** \brief Renders a composite loop closure visualization image showing all correspondences
         *         between the query keyframe and all matched loop-closing candidate keyframes.
         *  \param[in] q_keyframe_id ID of the keyframe to which the query cluster belongs.
         *  \param[in] i_path Directory path where keyframe images are stored on disk.
         *  \param[in] q_cluster_ids IDs of the query clusters involved in the loop closure.
         *  \param[in] cand_cluster_ids IDs of the candidate clusters involved in the loop closure.
         *  \param[in] cand_keyframe_ids IDs of the keyframes containing the candidate clusters that closed the loop.
         *  \param[in] f_q_l_kpts Filtered (inlier) query left keypoints.
         *  \param[in] f_q_r_kpts Filtered (inlier) query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered (inlier) candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered (inlier) candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair Number of geometrical inliers for each loop-closing cluster pair.
         *  \param[in] definitive_cluster_pairs Vector of matched cluster ID pairs (`q_cluster_id`, `cand_cluster_id`).
         *  \param[in] cand_cluster_keyframe_relations Vector of pairs mapping candidate cluster IDs to their keyframe IDs (`cand_cluster_id`, `cand_keyframe_id`).
         *  \return Composite visualization image of the loop closure.
         */
        cv::Mat drawImgLoopClosure(const int q_keyframe_id,
                                   const std::string& i_path,
                                   const std::vector<int>& q_cluster_ids,
                                   const std::vector<int>& cand_cluster_ids,
                                   const std::vector<int>& cand_keyframe_ids,
                                   const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                   const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                   const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                   const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                   const std::vector<int>& definitive_inliers_per_pair,
                                   const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                   const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations);


        /** \brief Constructs a `nav_msgs::Odometry` message from a 6-DoF pose.
         *  \param[in] c_stamp Current pose timestamp.
         *  \param[in] c_pose Current 6-DoF pose.
         *  \param[in] bad_pose_covariance Flag indicating whether pose estimation failed (assigns high covariance).
         *  \param[in] reference_frame_id ID of the coordinate frame to which the current pose is relative.
         *  \return Constructed `nav_msgs::Odometry` message.
         */
        nav_msgs::Odometry buildPoseMsg(const ros::Time& c_stamp,
                                        const tf2::Transform& c_pose,
                                        const bool bad_pose_covariance,
                                        const std::string& reference_frame_id);


        /** \brief Constructs a custom `slamon::Keyframe` ROS message from keyframe data and clusters.
         *  \param[in] keyframe_id Unique keyframe ID.
         *  \param[in] l_descs Matrix of keyframe left feature descriptors.
         *  \param[in] r_descs Matrix of keyframe right feature descriptors.
         *  \param[in] stamp Keyframe capture ROS timestamp.
         *  \param[in] inliers Number of matched inliers per cluster pair.
         *  \param[in] cluster_ids Vector mapping each keypoint to its cluster ID.
         *  \param[in] l_kpts Vector of keyframe left 2D keypoints.
         *  \param[in] r_kpts Vector of keyframe right 2D keypoints.
         *  \param[in] l_cam_pts Vector of keyframe 3D points in the left camera frame.
         *  \param[in] cluster_centroids Vector of 3D centroids for each cluster in the keyframe.
         *  \param[in] cluster_pairs Vector of matched cluster ID pairs.
         *  \return Constructed `slamon::Keyframe` message.
         */
        slamon::Keyframe buildKeyframe(const int keyframe_id,
                                       const cv::Mat& l_descs,
                                       const cv::Mat& r_descs,
                                       const ros::Time& stamp,
                                       const std::vector<int>& inliers,
                                       const std::vector<int>& cluster_ids,
                                       const std::vector<cv::KeyPoint>& l_kpts,
                                       const std::vector<cv::KeyPoint>& r_kpts,
                                       const std::vector<cv::Point3f>& l_cam_pts,
                                       const std::vector<cv::Point3f>& cluster_centroids,
                                       const std::vector<std::pair<int, int>>& cluster_pairs);


        /** \brief Converts an OpenCV image into a ROS image message and publishes it.
         *  \param[in] img Input OpenCV image.
         *  \param[in] stamp ROS timestamp for the published image message.
         *  \param[in] publisher Target ROS image publisher.
         */
        void publishImg(const cv::Mat& img,
                        const ros::Time& stamp,
                        const ros::Publisher& publisher);


        /** \brief Default 6x6 pose covariance matrix (row-major) for valid pose estimates. */
        const boost::array<double, 36> GOOD_POSE_COVARIANCE_ = 
        { { 0.1,     0,     0,     0,     0,     0,
              0,   0.1,     0,     0,     0,     0,
              0,     0,   0.1,     0,     0,     0,
              0,     0,     0,  0.17,     0,     0,
              0,     0,     0,     0,  0.17,     0,
              0,     0,     0,     0,     0,  0.17 } };

        /** \brief Default 6x6 twist covariance matrix (row-major) for valid velocity estimates. */
        const boost::array<double, 36> GOOD_TWIST_COVARIANCE_ = 
        { { 0.002,     0,     0,     0,     0,     0,
                0, 0.002,     0,     0,     0,     0,
                0,     0,  0.05,     0,     0,     0,
                0,     0,     0,  0.09,     0,     0,
                0,     0,     0,     0,  0.09,     0,
                0,     0,     0,     0,     0,  0.09 } };

        /** \brief High-uncertainty 6x6 covariance matrix (row-major) assigned when tracking or motion estimation fails. */
        const boost::array<double, 36> BAD_COVARIANCE_ = 
        { { 9999,      0,     0,     0,     0,     0,
               0,   9999,     0,     0,     0,     0,
               0,      0,  9999,     0,     0,     0,
               0,      0,     0,  9999,     0,     0,
               0,      0,     0,     0,  9999,     0,
               0,      0,     0,     0,     0,  9999 } };
};


#endif // PUBLISHER_HPP