/**
 *  @file publisher.hpp
 *  @brief The Publisher class is responsive to publish the image visualizations 
 *         for debugging purposes.
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

 
#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "slamon/Keypoint.h"
#include "slamon/Keyframe.h"
#include "slamon/Performance.h"


class Publisher
{
    public:

        /** \brief Empty class constructor.
         */
        Publisher() {};


        /** \brief Structure for storing SLAMON runtimes. To avoid creating a structure
         *  for each part of the system, this structure is reused in all three threads.
         */
        struct RunTimes
        {
            double run_time_0 = 0.0;     //!> Time needed for VO: Feature extraction, MM: Pose update or LC: Process current cluster.
            double run_time_1 = 0.0;     //!> Time needed for VO: Stereo matching, MM: Add vertex (cluster) to graph or LC: Search for candidates to close loops by proximity.
            double run_time_2 = 0.0;     //!> Time needed for VO: Triangulation, MM: Add co-framed edges to graph or LC: Search for candidates to close loops by hash.
            double run_time_3 = 0.0;     //!> Time needed for VO: Clustering, MM: Add inter-frames edges to graph or LC: Read cluster candidates.
            double run_time_4 = 0.0;     //!> Time needed for VO: Temporal matching, MM: Publish or LC: Loop detection.
            double run_time_5 = 0.0;     //!> Time needed for VO: Bucketing, MM: Data write or LC: Loop correction.
            double run_time_6 = 0.0;     //!> Time needed for VO: Motion estimation, MM: Nothing or LC: Nothing.
            double run_time_7 = 0.0;     //!> Time needed for VO: Keyframe insertion decision, MM: Nothing or LC: Nothing.
            double total_run_time = 0.0; //!> Time needed for each thread to perform one iteration.

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


        /** \brief Writes on disk a image with its keypoints drawn.
         *  \param[in] img The image.
         *  \param[in] color The keypoints color.
         *  \param[in] o_path Image output path. The name of the final 
         *  file must be included.
         *  \param[in] kpts The keypoints.
         */
        void writeImgKeypoints(const cv::Mat& img,
                               const cv::Scalar& color,
                               const std::string& o_path,
                               const std::vector<cv::KeyPoint>& kpts);


        /** \brief Publishes stereo images with its keypoints drawn.
         *  \param[in] l_img The left image. 
         *  \param[in] r_img The right image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] l_kpts The left image keypoints.
         *  \param[in] r_kpts The left image keypoints.
         */
        void publishImgStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const ros::Time& stamp,
                                       const ros::Publisher& publisher,
                                       const std::vector<cv::KeyPoint>& l_kpts,
                                       const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Writes on disk the stereoscopic images with their
         *  keypoints drawn.
         *  \param[in] l_img The left image. 
         *  \param[in] r_img The right image.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] l_kpts The left image keypoints.
         *  \param[in] r_kpts The left image keypoints.
         */
        void writeImgStereoKeypoints(const cv::Mat& l_img,
                                     const cv::Mat& r_img,
                                     const std::string& o_path,
                                     const std::vector<cv::KeyPoint>& l_kpts,
                                     const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Draws and publishes vertical (temporal) or horizontal (stereo)
         *  matches between two images.
         *  \param[in] vertical If true, draws and publishes vertical matches 
         *  (case for the temporal images). If false, draws and publishes horizontal
         *  matches (case for stereo pair).
         *  \param[in] img_0 In the case of temporal images, it is the current image.
         *  In the stereo case, it is the left image.
         *  \param[in] img_1 In the case of temporal images, it is the previous image.
         *  In the stereo case, it is the right image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] m_kpts_0 Matched and sorted current/left keypoints.
         *  \param[in] m_kpts_1 Matched and sorted previous/right keypoints.
         */
        void publishImgMatches(const bool vertical, 
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const ros::Time& stamp,
                               const ros::Publisher& publisher,
                               const std::vector<cv::KeyPoint>& m_kpts_0,
                               const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Draws and writes on disk vertical (temporal) or horizontal (stereo)
         *  matches between two images.
         *  \param[in] vertical If true, draws and publishes vertical matches 
         *  (case for the temporal images). If false, draws and publishes horizontal
         *  matches (case for stereo pair).
         *  \param[in] img_0 In the case of temporal images, it is the current image.
         *  In the stereo case, it is the left image.
         *  \param[in] img_1 In the case of temporal images, it is the previous image.
         *  In the stereo case, it is the right image.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] m_kpts_0 Matched and sorted current/left keypoints.
         *  \param[in] m_kpts_1 Matched and sorted previous/right keypoints.
         */
        void writeImgMatches(const bool vertical, 
                             const cv::Mat& img_0,
                             const cv::Mat& img_1,
                             const std::string& o_path,
                             const std::vector<cv::KeyPoint>& m_kpts_0,
                             const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Draws and publishes circular matches.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] m_c_l_kpts Matched and sorted current left keypoints.
         *  \param[in] m_c_r_kpts Matched and sorted current right keypoints.
         *  \param[in] m_p_l_kpts Matched and sorted previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and sorted previous right keypoints.
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


        /** \brief Draws and writes on disk circular matches.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] m_c_l_kpts Matched and sorted current left keypoints.
         *  \param[in] m_c_r_kpts Matched and sorted current right keypoints.
         *  \param[in] m_p_l_kpts Matched and sorted previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and sorted previous right keypoints.
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


        /** \brief Draws and publishes keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] l_kpts Clustered left keypoints.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const ros::Publisher& publisher,
                                           const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws and publishes keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] l_kpts Left keypoints.
         *  \param[in] clusters Keypoints clustering.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const ros::Publisher& publisher,
                                           const std::vector<cv::KeyPoint>& l_kpts,
                                           const std::vector<std::vector<int>>& clusters);


        /** \brief Draws and publishes keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] cluster_num Number of clusters.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] cluster_ids Cluster identifiers.
         *  \param[in] l_kpts Left keypoints.
         */
        void publishImgKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const size_t cluster_num,
                                           const ros::Publisher& publisher,
                                           const std::vector<int>& cluster_ids,
                                           const std::vector<cv::KeyPoint>& l_kpts);


        /** \brief Draws and writes on disk keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] l_kpts Clustered left keypoints.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const std::string& o_path,
                                         const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws and writes on disk keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] l_kpts Left keypoints.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] clusters Keypoints clustering.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const std::string& o_path,
                                         const std::vector<cv::KeyPoint>& l_kpts,
                                         const std::vector<std::vector<int>>& clusters);


        /** \brief Draws and writes on disk keypoints clustering.
         *  \param[in] l_img Left image.
         *  \param[in] cluster_num Number of clusters.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] cluster_ids Cluster identifiers.
         *  \param[in] l_kpts Left keypoints.
         */
        void writeImgKeypointsClustering(const cv::Mat& l_img,
                                         const size_t cluster_num,
                                         const std::string& o_path,
                                         const std::vector<int>& cluster_ids,
                                         const std::vector<cv::KeyPoint>& l_kpts);


        /** \brief Draws and publishes a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] q_keyframe_id Keyframe identifier to which the query cluster belongs.
         *  \param[in] i_path Path to keyframes saved on disk.
         *  \param[in] q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop.
         *  \param[in] cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop.
         *  \param[in] cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops.
         *  \param[in] f_q_l_kpts Filtered query left keypoints.
         *  \param[in] f_q_r_kpts Filtered query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair.
         *  \param[in] definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id).
         *  \param[in] cand_cluster_keyframe_relations Stores the candidate cluster-keyframe
         *  relations (cand_cluster_id, cand_keyframe_id).
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


        /** \brief Draws and writes on disk a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param[in] o_path Image output path. The name of the final
         *  file must be included.
         *  \param[in] q_keyframe_id Keyframe identifier to which the query cluster belongs.
         *  \param[in] i_path Path to keyframes saved on disk.
         *  \param[in] q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop.
         *  \param[in] cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop.
         *  \param[in] cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops.
         *  \param[in] f_q_l_kpts Filtered query left keypoints.
         *  \param[in] f_q_r_kpts Filtered query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair.
         *  \param[in] definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id).
         *  \param[in] cand_cluster_keyframe_relations Stores the candidate cluster-keyframe
         *  relations (cand_cluster_id, cand_keyframe_id).
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


        /** \brief Publishes current robot pose.
         *  \param[in] c_stamp Current pose timestamp.
         *  \param[in] c_pose The current pose to be published.
         *  \param[in] bad_pose_covariance Has the module been lost?
         *  \param[in] publisher The ROS publisher.
         *  \param[in] reference_frame_id The ID of the frame to which the pose
         *  is relative.
         */
        void publishPose(const ros::Time& c_stamp,
                         const tf::Transform& c_pose,
                         const bool bad_pose_covariance,
                         const ros::Publisher& publisher,
                         const std::string& reference_frame_id);


        /** \brief Publishes current pose and velocity.
         *  \param[in] c_stamp Current ROS timestamp.
         *  \param[in] p_stamp Previous ROS timestamp.
         *  \param[in] c_pose The current pose to be published.
         *  \param[in] c_motion The movement between current and previous frames.
         *  Used to calculate velocity.
         *  \param[in] bad_pose_covariance Has the module been lost?
         *  \param[in] bad_twist_covariance Has the module been lost?
         *  \param[in] publisher The ROS publisher.
         *  \param[in] reference_frame_id The ID of the frame to which the pose and 
         *  velocity are relative.
         */
        void publishPoseAndVelocity(const ros::Time& c_stamp,
                                    const ros::Time& p_stamp,
                                    const tf::Transform& c_pose,
                                    const tf::Transform& c_motion,
                                    const bool bad_pose_covariance,
                                    const bool bad_twist_covariance,
                                    const ros::Publisher& publisher,
                                    const std::string& reference_frame_id);


        /** \brief Publishes SLAMON performance.
         *  \param[in] stamp Message timestamp. VO: current frame timestamp,
         *   MM: current time or LC: current time.
         *  \param[in] successes_num Successes in the last iteration of the
         *  thread in question..
         *  \param[in] run_times Structure with all the execution times of
         *  the thread in question..
         *  \param[in] publisher The ROS publisher.
         */
        void publishPerformance(const ros::Time& stamp,
                                const int successes_num,
                                const RunTimes& run_times,
                                const ros::Publisher& publisher);


        /** \brief Publishes a keyframe in ROS.
         *  \param[in] keyframe_id Keyframe identifier.
         *  \param[in] l_descs Keyframe left descriptors.
         *  \param[in] r_descs Keyframe right descriptors.
         *  \param[in] stamp Current image ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] inliers Number of inliers per cluster pair.
         *  \param[in] cluster_ids Keyframe cluster ids.
         *  \param[in] l_kpts Keyframe left keypoints.
         *  \param[in] r_kpts Keyframe right keypoints.
         *  \param[in] l_cam_pts Keyframe left camera points.
         *  \param[in] cluster_centroids Cluster centroids.
         *  \param[in] cluster_pairs Cluster pairs.
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


        /** \brief Publishes a keyframe in ROS.
         *  \param[in] keyframe_id Keyframe identifier.
         *  \param[in] l_img Keyframe left image.
         *  \param[in] r_img Keyframe right image.
         *  \param[in] l_descs Keyframe left descriptors.
         *  \param[in] r_descs Keyframe right descriptors.
         *  \param[in] stamp Current image ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         *  \param[in] inliers Number of inliers per cluster pair.
         *  \param[in] cluster_ids Keyframe cluster ids.
         *  \param[in] l_kpts Keyframe left keypoints.
         *  \param[in] r_kpts Keyframe right keypoints.
         *  \param[in] l_cam_pts Keyframe left camera points.
         *  \param[in] cluster_centroids Cluster centroids.
         *  \param[in] cluster_pairs Cluster pairs.
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


        /** \brief Publishes counters to ROS.
         *  \param[in] counter The data to be published.
         *  \param[in] publisher The ROS publisher. 
         */
        void publishCounter(const int counter,
                            const ros::Publisher& publisher);


        /** \brief Writes a list of poses.
         *  \param[in] o_path File output path. The name of the final 
         *  file must be included.
         *  \param[in] stamps Poses timestamps.
         *  \param[in] poses The list of poses to be written in the file.
         */
        void writePoses(const std::string& o_path,
                        const std::vector<double>& stamps,
                        const std::vector<tf::Transform>& poses);


        /** \brief Writes a list of edges.
         *  \param[in] o_path File output path. The name of the final 
         *  file must be included.
         *  \param[in] weights A simple scalar list reflecting the total 
         *  confidence in all dimensions of each graph edge.
         *  \param[in] edges_ids A list of the identifiers of the keyframe
         *  pairs that close a loop.
         *  \param[in] edges_poses A list of the poses of the keyframe pairs
         *  that close a loop.
         */
        void writeLoopClosureEdges(const std::string& o_path,
                                   const std::vector<double>& weights,
                                   const std::vector<std::pair<int, int>> edges_ids,
                                   const std::vector<std::pair<tf::Transform, tf::Transform>>& edges_poses);


        /** \brief Converts an integer to a string containing a
         *  five-digit number.
         *  \param[in] in Input integer.
         *  @return The string containing the five-digit number.
         */
        std::string convertTo5Digits(int in);


    private:


        /** \brief Draws points to image.
         *  \param[in] img Original image.
         *  \param[in] color The colour desired to be used to paint the points.
         *  \param[in] kpts The points.
         *  @return The image with the points drawn.
         */
        cv::Mat drawImgKeypoints(const cv::Mat& img,
                                 const cv::Scalar& color,
                                 const std::vector<cv::KeyPoint>& kpts);


        /** \brief Writes text at the bottom of the image. 
         *  \param[in] caption If true, the text is written in the caption.
         *  Otherwise it is written as a title.
         *  \param[in] img The image.
         *  \param[in] extra_vertical_space Extra vertical space to keep
         *  the text from touching the margins.
         *  \param[in] text The text.
         *  @return An image with the txt at the bottom.
         */
        cv::Mat drawTxtInImg(const bool caption,
                             const cv::Mat& img,
                             const std::stringstream& text,
                             const int extra_vertical_space);


        /** \brief Concatenates the stereo images and draw their keypoints. 
         *  \param[in] l_img The left image. 
         *  \param[in] r_img The right image.
         *  \param[in] l_kpts The left image keypoints.
         *  \param[in] r_kpts The left image keypoints.
         *  @return An image showing the stereo pair with its key points
         *  drawn and a text indicating the number of points.
         */
        cv::Mat drawImgStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const cv::Scalar& color,
                                       const std::vector<cv::KeyPoint>& l_kpts,
                                       const std::vector<cv::KeyPoint>& r_kpts);


        /** \brief Draws vertical or horizontal matches between two images.
         *  \param[in] vertical If true, the function concatenates the two images
         *  and paints the matchings vertically (case for the temporal images). 
         *  In false, it does it horizontally (case for stereo pair).
         *  \param[in] img_0 In the case of temporal images, it is the current image.
         *  In the stereo case, it is the left image.
         *  \param[in] img_1 In the case of temporal images, it is the previous image.
         *  In the stereo case, it is the right image.
         *  \param[in] color Color of the keypoints and matches.
         *  \param[in] m_kpts_0 Matched and sorted current/left keypoints.
         *  \param[in] m_kpts_1 Matched and sorted previous/right keypoints.
         *  @return a concatened and drawn image. 
         */
        cv::Mat drawImgMatches(const bool vertical,
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const cv::Scalar& color,
                               const std::vector<cv::KeyPoint>& m_kpts_0,
                               const std::vector<cv::KeyPoint>& m_kpts_1);


        /** \brief Draws circular matches.
         *  \param[in] c_l_img Current left image.
         *  \param[in] c_r_img Current right image.
         *  \param[in] p_l_img Previous left image.
         *  \param[in] p_r_img Previous right image.
         *  \param[in] color Color of the keypoints and matches.
         *  \param[in] m_c_l_kpts Matched and sorted current left keypoints.
         *  \param[in] m_c_r_kpts Matched and sorted current right keypoints.
         *  \param[in] m_p_l_kpts Matched and sorted previous left keypoints.
         *  \param[in] m_p_r_kpts Matched and sorted previous right keypoints.
         *  @return An image where the current stereo pair and the previous stereo
         *  pair are concatenated with the drawn matches.
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


        /** \brief Draws keypoints clustering.
         *  \param[in] img The image where draw the keypoints clusters.
         *  \param[in] l_kpts Clustered left keypoints.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const std::vector<std::vector<cv::KeyPoint>>& l_kpts);


        /** \brief Draws keypoints clustering.
         *  \param[in] img The image where draw the keypoints clusters.
         *  \param[in] kpts The keypoints.
         *  \param[in] clusters Keypoints clustering.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const std::vector<cv::KeyPoint>& kpts,
                                           const std::vector<std::vector<int>>& clusters);


        /** \brief Draws keypoints clustering.
         *  \param[in] img The image where draw the keypoints clusters.
         *  \param[in] cluster_num Number of clusters.
         *  \param[in] cluster_ids Cluster identifiers.
         *  \param[in] kpts The keypoints.
         */
        cv::Mat drawImgKeypointsClustering(const cv::Mat& img,
                                           const size_t cluster_num,
                                           const std::vector<int>& cluster_ids,
                                           const std::vector<cv::KeyPoint>& kpts);


        /** \brief Draws a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param[in] q_keyframe_id Keyframe identifier to which the query cluster belongs.
         *  \param[in] i_path Path to keyframes saved on disk.
         *  \param[in] q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop.
         *  \param[in] cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop.
         *  \param[in] cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops.
         *  \param[in] f_q_l_kpts Filtered query left keypoints.
         *  \param[in] f_q_r_kpts Filtered query right keypoints.
         *  \param[in] f_cand_l_kpts Filtered candidate left keypoints.
         *  \param[in] f_cand_r_kpts Filtered candidate right keypoints.
         *  \param[in] definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair.
         *  \param[in] definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id).
         *  \param[in] cand_cluster_keyframe_relations Stores the candidate cluster-keyframe
         *  relations (cand_cluster_id, cand_keyframe_id).
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


        /** \brief Builds nav_msgs::Odometry msg with current pose.
         *  \param[in] c_stamp Current pose timestamp.
         *  \param[in] c_pose The current pose to be published.
         *  \param[in] bad_pose_covariance Has the module been lost?
         *  \param[in] reference_frame_id The ID of the frame to which 
         *  the current pose is relative.
         */
        nav_msgs::Odometry buildPoseMsg(const ros::Time& c_stamp,
                                        const tf::Transform& c_pose,
                                        const bool bad_pose_covariance,
                                        const std::string& reference_frame_id);


        /** \brief Builds a slamon::Keyframe.
         *  \param[in] keyframe_id Keyframe identifier.
         *  \param[in] l_descs Keyframe left descriptors.
         *  \param[in] r_descs Keyframe right descriptors.
         *  \param[in] stamp Current image ROS timestamp.
         *  \param[in] inliers Number of inliers per cluster pair.
         *  \param[in] cluster_ids Keyframe cluster ids.
         *  \param[in] l_kpts Keyframe left keypoints.
         *  \param[in] r_kpts Keyframe right keypoints.
         *  \param[in] l_cam_pts Keyframe left camera points.
         *  \param[in] cluster_centroids Cluster centroids.
         *  \param[in] cluster_pairs Cluster pairs.
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


        /** \brief Publishes images.
         *  \param[in] img The image.
         *  \param[in] stamp The ROS timestamp.
         *  \param[in] publisher The ROS publisher.
         */
        void publishImg(const cv::Mat& img,
                        const ros::Time& stamp,
                        const ros::Publisher& publisher);


        // Attributes:

        // Some arbitrary values.
        const boost::array<double, 36> GOOD_POSE_COVARIANCE_ = 
        { { 0.1,     0,     0,     0,     0,     0,
              0,   0.1,     0,     0,     0,     0,
              0,     0,   0.1,     0,     0,     0,
              0,     0,     0,  0.17,     0,     0,
              0,     0,     0,     0,  0.17,     0,
              0,     0,     0,     0,     0,  0.17 } };

        const boost::array<double, 36> GOOD_TWIST_COVARIANCE_ = 
        { { 0.002,     0,     0,     0,     0,     0,
                0, 0.002,     0,     0,     0,     0,
                0,     0,  0.05,     0,     0,     0,
                0,     0,     0,  0.09,     0,     0,
                0,     0,     0,     0,  0.09,     0,
                0,     0,     0,     0,     0,  0.09 } };

        const boost::array<double, 36> BAD_COVARIANCE_ = 
        { { 9999,      0,     0,     0,     0,     0,
               0,   9999,     0,     0,     0,     0,
               0,      0,  9999,     0,     0,     0,
               0,      0,     0,  9999,     0,     0,
               0,      0,     0,     0,  9999,     0,
               0,      0,     0,     0,     0,  9999 } };
};


#endif // PUBLISHER_HPP