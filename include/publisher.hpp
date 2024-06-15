/**
 *  @file
 *  @brief The Publisher class is responsive to publish the image visualizations for debugging purposes.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <string>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "slamon/Frame.h"
#include "slamon/Keypoint.h"
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
            double run_time_0 = 0.0;     //!> Time needed for VO: Feature extraction, MAP: From Frame msg to Frame object or LC: Process current cluster.
            double run_time_1 = 0.0;     //!> Time needed for VO: Stereo matching, MAP: Need new keyframe? or LC: Search for candidates to close loops by proximity.
            double run_time_2 = 0.0;     //!> Time needed for VO: Triangulation, MAP: Add vertex (cluster) to graph or LC: Search for candidates to close loops by hash.
            double run_time_3 = 0.0;     //!> Time needed for VO: Successive matching, MAP: Add edges to graph or LC: Read cluster candidates.
            double run_time_4 = 0.0;     //!> Time needed for VO: Motion estimation, MAP: Write graph to disk and publish updated robot pose or LC: Loop detection.
            double run_time_5 = 0.0;     //!> Time needed for VO: Nothing, MAP: Nothing or LC: Close loop.
            double total_run_time = 0.0; //!> Time needed for each thread to perform one iteration.
        };


        /** \brief Writes on disk a image with its keypoints drawn.
         *  \param img The image (input).
         *  \param color The keypoints color (input).
         *  \param o_path Image output path. The name of the final 
         *  file must be included (input).
         *  \param kpts The keypoints (input).
         */
        void writeKeypoints(const cv::Mat& img,
                            const cv::Scalar& color,
                            const std::string& o_path,
                            const std::vector<cv::Point2f> kpts);


        /** \brief Publishes stereo images with its keypoints drawn.
         *  \param l_img The left image (input). 
         *  \param r_img The right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param l_kpts The left image keypoints (input).
         *  \param r_kpts The left image keypoints (input).
         */
        void publishStereoKeypoints(const cv::Mat& l_img,
                                    const cv::Mat& r_img,
                                    const ros::Time& stamp,
                                    const ros::Publisher& publisher,
                                    const std::vector<cv::Point2f>& l_kpts,
                                    const std::vector<cv::Point2f>& r_kpts);


        /** \brief Writes on disk the stereoscopic images with their
         *  keypoints drawn.
         *  \param l_img The left image (input). 
         *  \param r_img The right image (input).
         *  \param o_path Image output path. The name of the final
         *  file must be included (input).
         *  \param l_kpts The left image keypoints (input).
         *  \param r_kpts The left image keypoints (input).
         */
        void writeStereoKeypoints(const cv::Mat& l_img,
                                  const cv::Mat& r_img,
                                  const std::string& o_path,
                                  const std::vector<cv::Point2f>& l_kpts,
                                  const std::vector<cv::Point2f>& r_kpts);


        /** \brief Draws and publishes vertical (successive) or horizontal (stereo)
         *  matches between two images.
         *  \param vertical If true, draws and publishes vertical matches 
         *  (case for the successive images). If false, draws and publishes horizontal
         *  matches (case for stereo pair) (input).
         *  \param img_0 In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param img_1 In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param f_kpts_0 Filtered and sorted current/left keypoints (input).
         *  \param f_kpts_1 Filtered and sorted previous/right keypoints (input).
         */
        void publishMatches(const bool vertical, 
                            const cv::Mat& img_0,
                            const cv::Mat& img_1,
                            const ros::Time& stamp,
                            const ros::Publisher& publisher,
                            const std::vector<cv::Point2f>& f_kpts_0,
                            const std::vector<cv::Point2f>& f_kpts_1);


        /** \brief Draws and writes on disk vertical (successive) or horizontal (stereo)
         *  matches between two images.
         *  \param vertical If true, draws and publishes vertical matches 
         *  (case for the successive images). If false, draws and publishes horizontal
         *  matches (case for stereo pair) (input).
         *  \param img_0 In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param img_1 In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param o_path Image output path. The name of the final
         *  file must be included (input).
         *  \param f_kpts_0 Filtered and sorted current/left keypoints (input).
         *  \param f_kpts_1 Filtered and sorted previous/right keypoints (input).
         */
        void writeMatches(const bool vertical, 
                          const cv::Mat& img_0,
                          const cv::Mat& img_1,
                          const std::string& o_path,
                          const std::vector<cv::Point2f>& f_kpts_0,
                          const std::vector<cv::Point2f>& f_kpts_1);


        /** \brief Draws and publishes circular matches.
         *  \param c_l_img Current left image (input).
         *  \param c_r_img Current right image (input).
         *  \param p_l_img Previous left image (input).
         *  \param p_r_img Previous right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param f_c_l_kpts Filtered and sorted current left keypoints (input).
         *  \param f_c_r_kpts Filtered and sorted current right keypoints (input).
         *  \param f_p_l_kpts Filtered and sorted previous left keypoints (input).
         *  \param f_p_r_kpts Filtered and sorted previous right keypoints (input).
         */
        void publishCircularMatches(const cv::Mat& c_l_img,                                    
                                    const cv::Mat& c_r_img,
                                    const cv::Mat& p_l_img,
                                    const cv::Mat& p_r_img,
                                    const ros::Time& stamp,
                                    const ros::Publisher& publisher,
                                    const std::vector<cv::Point2f>& f_c_l_kpts,
                                    const std::vector<cv::Point2f>& f_c_r_kpts,
                                    const std::vector<cv::Point2f>& f_p_l_kpts,
                                    const std::vector<cv::Point2f>& f_p_r_kpts);


        /** \brief Draws and writes on disk circular matches.
         *  \param c_l_img Current left image (input).
         *  \param c_r_img Current right image (input).
         *  \param p_l_img Previous left image (input).
         *  \param p_r_img Previous right image (input).
         *  \param o_path Image output path. The name of the final
         *  file must be included (input).
         *  \param f_c_l_kpts Filtered and sorted current left keypoints (input).
         *  \param f_c_r_kpts Filtered and sorted current right keypoints (input).
         *  \param f_p_l_kpts Filtered and sorted previous left keypoints (input).
         *  \param f_p_r_kpts Filtered and sorted previous right keypoints (input).
         */
        void writeCircularMatches(const cv::Mat& c_l_img,
                                  const cv::Mat& c_r_img,
                                  const cv::Mat& p_l_img,
                                  const cv::Mat& p_r_img,
                                  const std::string& o_path,
                                  const std::vector<cv::Point2f>& f_c_l_kpts,
                                  const std::vector<cv::Point2f>& f_c_r_kpts,
                                  const std::vector<cv::Point2f>& f_p_l_kpts,
                                  const std::vector<cv::Point2f>& f_p_r_kpts);


        /** \brief Draws and publishes keypoints clustering.
         *  \param l_img Left image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param l_kpts Left keypoints (input).
         *  \param clusters Keypoints clustering (input).
         */
        void publishKeypointsClustering(const cv::Mat& l_img,
                                        const ros::Time& stamp,
                                        const ros::Publisher& publisher,
                                        const std::vector<cv::Point2f>& l_kpts,
                                        const std::vector<std::vector<int>>& clusters);


        /** \brief Draws and writes on disk keypoints clustering.
         *  \param l_img Left image (input).
         *  \param l_kpts Left keypoints (input).
         *  \param o_path Image output path. The name of the final
         *  file must be included (input).
         *  \param clusters Keypoints clustering (input).
         */
        void writeKeypointsClustering(const cv::Mat& l_img,
                                      const std::string& o_path,
                                      const std::vector<cv::Point2f>& l_kpts,
                                      const std::vector<std::vector<int>>& clusters);


        /** \brief Draws and publishes a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param q_keyframe_id Keyframe identifier to which the query cluster belongs
         *  (input).
         *  \param i_path Path to keyframes saved on disk.
         *  \param q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop (input).
         *  \param cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop (input).
         *  \param cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops (input).
         *  \param f_q_l_kpts Filtered query left keypoints (input).
         *  \param f_q_r_kpts Filtered query right keypoints (input).
         *  \param f_cand_l_kpts Filtered candidate left keypoints (input).
         *  \param f_cand_r_kpts Filtered candidate right keypoints (input).
         *  \param definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair (input).
         *  \param definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id)
         *  (input).
         *  \param cand_cluster_keyframe_relation Stores the candidate cluster-keyframe
         *  relation (cand_cluster_id, cand_keyframe_id) (input).
         */
        void publishLoopClosure(const ros::Time& stamp,
                                const ros::Publisher& publisher,
                                const int q_keyframe_id,
                                const std::string& i_path,
                                const std::vector<int>& q_cluster_ids,
                                const std::vector<int>& cand_cluster_ids,
                                const std::vector<int>& cand_keyframe_ids,
                                const std::vector<cv::Point2f>& f_q_l_kpts,
                                const std::vector<cv::Point2f>& f_q_r_kpts,
                                const std::vector<cv::Point2f>& f_cand_l_kpts,
                                const std::vector<cv::Point2f>& f_cand_r_kpts,
                                const std::vector<int>& definitive_inliers_per_pair,
                                const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation);


        /** \brief Draws and writes on disk a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param o_path Image output path. The name of the final
         *  file must be included (input).
         *  \param q_keyframe_id Keyframe identifier to which the query cluster belongs
         *  (input).
         *  \param i_path Path to keyframes saved on disk.
         *  \param q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop (input).
         *  \param cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop (input).
         *  \param cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops (input).
         *  \param f_q_l_kpts Filtered query left keypoints (input).
         *  \param f_q_r_kpts Filtered query right keypoints (input).
         *  \param f_cand_l_kpts Filtered candidate left keypoints (input).
         *  \param f_cand_r_kpts Filtered candidate right keypoints (input).
         *  \param definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair (input).
         *  \param definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id)
         *  (input).
         *  \param cand_cluster_keyframe_relation Stores the candidate cluster-keyframe
         *  relation (cand_cluster_id, cand_keyframe_id) (input).
         */
        void writeLoopClosure(const std::string& o_path,
                              const int q_keyframe_id,
                              const std::string& i_path,
                              const std::vector<int>& q_cluster_ids,
                              const std::vector<int>& cand_cluster_ids,
                              const std::vector<int>& cand_keyframe_ids,
                              const std::vector<cv::Point2f>& f_q_l_kpts,
                              const std::vector<cv::Point2f>& f_q_r_kpts,
                              const std::vector<cv::Point2f>& f_cand_l_kpts,
                              const std::vector<cv::Point2f>& f_cand_r_kpts,
                              const std::vector<int>& definitive_inliers_per_pair,
                              const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                              const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation);


        /** \brief Publishes current robot pose.
         *  \param c_stamp Current pose timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param world_frame_id World coordinate axis id (input).
         *  \param robot_frame_id Robot coordinate axis id (input).
         *  \param c_robot_pose The current robot pose to be published (input).
         */
        void publishPose(const ros::Time& c_stamp,
                         const ros::Publisher& publisher,
                         const std::string& world_frame_id,
                         const std::string& robot_frame_id,
                         const tf::Transform& c_robot_pose);


        /** \brief Publishes current robot pose and velocity.
         *  \param c_stamp Current frame ROS timestamp (input).
         *  \param p_stamp Previous ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         *  \param world_frame_id World coordinate axis id (input).
         *  \param robot_frame_id Robot coordinate axis id (input).
         *  \param c_robot_pose The current robot pose to be published (input).
         *  \param c_robot_motion The robot movement between current and previous frames.
         *  Used to calculate robot velocity (input).
         */
        void publishPoseAndVelocity(const ros::Time& c_stamp,
                                    const ros::Time& p_stamp,
                                    const ros::Publisher& publisher,
                                    const std::string& world_frame_id,
                                    const std::string& robot_frame_id,
                                    const tf::Transform& c_robot_pose,
                                    const tf::Transform& c_robot_motion);


        /** \brief Publishes SLAMON performance.
         *  \param stamp Message timestamp. VO: current frame timestamp,
         *   MAP: current time or LC: current time (input).
         *  \param successes_num Successes in the last iteration of the
         *  thread in question. (input).
         *  \param run_times Structure with all the execution times of
         *  the thread in question. (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishPerformance(const ros::Time& stamp,
                                const int successes_num,
                                const RunTimes& run_times,
                                const ros::Publisher& publisher);              


        /** \brief Publishes relevant information from a stereo pair of images.
         *  \param stamp Current frame ROS timestamp (input).
         *  \param l_descs The left image keypoints descriptors (input).
         *  \param r_descs The right image keypoints descriptors (input).
         *  \param l_kpts The left image keypoints (input).
         *  \param r_kpts The right image keypoints (input).
         *  \param cam_pts 3D points of the scene obtained from the stereo data
         *  and relative to camera (input).
         *  \param publisher The ROS publisher (input).
        */
        void publishFrame(const cv::Mat& l_descs,
                          const cv::Mat& r_descs,
                          const ros::Time& stamp,
                          const ros::Publisher& publisher,
                          const std::vector<cv::Point2f>& l_kpts,
                          const std::vector<cv::Point2f>& r_kpts,
                          const std::vector<cv::Point3f>& cam_pts);


        /** \brief Publishes counters to ROS.
         *  \param counter The data to be published (input).
         *  \param publisher The ROS publisher (input). 
         */
        void publishCounter(const int counter,
                            const ros::Publisher& publisher);


        /** \brief Writes a list of poses.
         *  \param o_path File output path. The name of the final 
         *  file must be included (input).
         *  \param stamps Poses timestamps (input).
         *  \param poses The list of poses to be written in the file (input).
         */
        void writePoses(const std::string& o_path,
                        const std::vector<double>& stamps,
                        const std::vector<tf::Transform>& poses);


        /** \brief Converts an integer to a string containing a
         *  five-digit number.
         *  \param in Input integer (input).
         *  @return The string containing the five-digit number.
         */
        std::string convertTo5Digits(int in);


    private:


        /** \brief Draws points to image.
         *  \param img Original image (input).
         *  \param color The colour desired to be used to paint the points (input).
         *  \param pts The points (input).
         *  @return The image with the points drawn.
         */
        cv::Mat drawPoints(const cv::Mat& img,
                           const cv::Scalar& color,
                           const std::vector<cv::Point2f>& pts);


        /** \brief Writes text at the bottom of the image. 
         *  \param caption If true, the text is written in the caption.
         *  Otherwise it is written as a caption (input).
         *  \param img The image (input).
         *  \param extra_vertical_space Extra vertical space to keep
         *  the text from touching the margins (input).
         *  \param text The text (input).
         *  @return An image with the txt at the bottom.
         */
        cv::Mat writeTxtInImage(const bool caption,
                                const cv::Mat& img,
                                const std::stringstream& text,
                                const int extra_vertical_space);


        /** \brief Concatenates the stereo images and draw their keypoints. 
         *  \param l_img The left image (input). 
         *  \param r_img The right image (input).
         *  \param l_kpts The left image keypoints (input).
         *  \param r_kpts The left image keypoints (input).
         *  @return An image showing the stereo pair with its key points
         *  drawn and a text indicating the number of points.
         */
        cv::Mat drawStereoKeypoints(const cv::Mat& l_img,
                                    const cv::Mat& r_img,
                                    const cv::Scalar& color,
                                    const std::vector<cv::Point2f>& l_kpts,
                                    const std::vector<cv::Point2f>& r_kpts);


        /** \brief Draws vertical or horizontal matches between two images.
         *  \param vertical If true, the function concatenates the two images
         *  and paints the matchings vertically (case for the successive images). 
         *  In false, it does it horizontally (case for stereo pair) (input).
         *  \param img_0 In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param img_1 In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param color Color of the keypoints and matches (input).
         *  \param f_kpts_0 Filtered and sorted current/left keypoints (input).
         *  \param f_kpts_1 Filtered and sorted previous/right keypoints (input).
         *  @return a concatened and drawn image. 
         */
        cv::Mat drawMatches(const bool vertical,
                            const cv::Mat& img_0,
                            const cv::Mat& img_1,
                            const cv::Scalar& color,
                            const std::vector<cv::Point2f>& f_kpts_0,
                            const std::vector<cv::Point2f>& f_kpts_1);


        /** \brief Draws circular matches.
         *  \param c_l_img Current left image (input).
         *  \param c_r_img Current right image (input).
         *  \param p_l_img Previous left image (input).
         *  \param p_r_img Previous right image (input).
         *  \param color Color of the keypoints and matches (input).
         *  \param f_c_l_kpts Filtered and sorted current left keypoints (input).
         *  \param f_c_r_kpts Filtered and sorted current right keypoints (input).
         *  \param f_p_l_kpts Filtered and sorted previous left keypoints (input).
         *  \param f_p_r_kpts Filtered and sorted previous right keypoints (input).
         *  @return An image where the current stereo pair and the previous stereo
         *  pair are concatenated with the drawn matches.
         */
        cv::Mat drawCircularMatches(const cv::Mat& c_l_img,                                    
                                    const cv::Mat& c_r_img,
                                    const cv::Mat& p_l_img,
                                    const cv::Mat& p_r_img,
                                    const cv::Scalar& color,
                                    const std::vector<cv::Point2f>& f_c_l_kpts,
                                    const std::vector<cv::Point2f>& f_c_r_kpts,
                                    const std::vector<cv::Point2f>& f_p_l_kpts,
                                    const std::vector<cv::Point2f>& f_p_r_kpts);


        /** \brief Draws keypoints clustering.
         *  \param img The image where draw the keypoints clusters (input).
         *  \param kpts The keypoints (input).
         *  \param clusters Keypoints clustering (input).
         */
        cv::Mat drawKeypointsClustering(const cv::Mat& img,
                                        const std::vector<cv::Point2f>& kpts,
                                        const std::vector<std::vector<int>>& clusters);


        /** \brief Draws a loop closure image with all the correspondences
         *  between query keyframe and all the loop closing keyframes.
         *  \param q_keyframe_id Keyframe identifier to which the query cluster belongs
         *  (input).
         *  \param i_path Path to keyframes saved on disk.
         *  \param q_cluster_ids Identifiers of the query clusters that have 
         *  closed the loop (input).
         *  \param cand_cluster_ids Identifiers of the candidate clusters that have 
         *  closed the loop (input).
         *  \param cand_keyframe_ids Identifiers of keyframes containing candidate 
         *  clusters that have closed loops (input).
         *  \param f_q_l_kpts Filtered query left keypoints (input).
         *  \param f_q_r_kpts Filtered query right keypoints (input).
         *  \param f_cand_l_kpts Filtered candidate left keypoints (input).
         *  \param f_cand_r_kpts Filtered candidate right keypoints (input).
         *  \param definitive_inliers_per_pair The number of inliers for every loop
         *  closure cluster pair (input).
         *  \param definitive_cluster_pairs The cluster pairs vector (q_cluster_id - cand_cluster_id)
         *  (input).
         *  \param cand_cluster_keyframe_relation Stores the candidate cluster-keyframe
         *  relation (cand_cluster_id, cand_keyframe_id) (input).
         */
        cv::Mat drawLoopClosure(const int q_keyframe_id,
                                const std::string& i_path,
                                const std::vector<int>& q_cluster_ids,
                                const std::vector<int>& cand_cluster_ids,
                                const std::vector<int>& cand_keyframe_ids,
                                const std::vector<cv::Point2f>& f_q_l_kpts,
                                const std::vector<cv::Point2f>& f_q_r_kpts,
                                const std::vector<cv::Point2f>& f_cand_l_kpts,
                                const std::vector<cv::Point2f>& f_cand_r_kpts,
                                const std::vector<int>& definitive_inliers_per_pair,
                                const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation);


        /** \brief Builds nav_msgs::Odometry msg with current robot pose.
         *  \param c_stamp Current pose timestamp (input).
         *  \param world_frame_id World coordinate axis id (input).
         *  \param robot_frame_id Robot coordinate axis id (input).
         *  \param c_robot_pose The current robot pose to be published (input).
         */
        nav_msgs::Odometry buildPoseMsg(const ros::Time& c_stamp,
                                        const std::string& world_frame_id,
                                        const std::string& robot_frame_id,
                                        const tf::Transform& c_robot_pose);


        /** \brief Publishes images.
         *  \param img The image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishImage(const cv::Mat& img,
                          const ros::Time& stamp,
                          const ros::Publisher& publisher);
};


#endif // PUBLISHER_HPP