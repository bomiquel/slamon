/**
 *  @file visual_odometer.hpp
 *  @brief The Visual Odometer class is responsive to track consecutive 
 *         image frames and compute their motion.
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @copyright
 *    Copyright (c) 2025 Bo Miquel Nordfeldt-Fiol
 *
 *  @license BSD-3-Clause
 *    This software is released under the BSD 3-Clause License.
 *    See the LICENSE file for full text.
 */


#ifndef VISUAL_ODOMETER_HPP
#define VISUAL_ODOMETER_HPP

#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "frame.hpp"
#include "tools.hpp"
#include "publisher.hpp"
#include "motion_estimator.hpp"
#include "keypoint_matcher.hpp"
#include "keypoint_detector.hpp"

#include "slamon/GetTransform.h"
#include "slamon/GetProjectionMatrices.h"


namespace slamon
{


    class VisualOdometer: protected KeypointDetector, protected KeypointMatcher, protected MotionEstimator, protected Publisher
    {
        public:

            enum VOStatus
            {
                UNINITIALISED    = 0,
                BOOTSTRAPPING    = 1,
                LOST             = 2,
                RUNNING          = 3
            };


            /** \brief Class constructor.
             *  \param[in] min_inliers Minimum number of keypoints to estimate the movement.
             *  \param[in] min_lc_inliers Minimum number of inliers for loop closure.
             *  \param[in] max_kpts_bucket Maximum number of keypoints per bucket.
             *  \param[in] max_kpts_retained Maximum number of keypoints to be retained.
             *  \param[in] enable_debugging Enabling or disabling debugging messages and topics.
             *  \param[in] enable_bucketing Enabling or disabling keypoints bucketing.
             *  \param[in] enable_enhacement Enabling or disabling image enhacement.
             *  \param[in] matching_type Matching type.
             *  \param[in] robot_frame_id Robot odometry coordinate axis identficator.
             *  \param[in] feature_detector_name String containing the name of the desired detector for detecting
             *  image keypoints.
             *  \param[in] motion_estimation_type Motion estimation type.
             */
            VisualOdometer(const int min_inliers,
                           const int min_lc_inliers,
                           const int max_kpts_bucket,
                           const int max_kpts_retained,
                           const bool enable_debugging,
                           const bool enable_bucketing,
                           const bool enable_enhacement,
                           const std::string& matching_type,
                           const std::string& robot_frame_id,
                           const std::string& feature_detector_name,
                           const std::string& motion_estimation_type);


            /** \brief Starts tracking
             */
            void run();


        private:


            /** \brief Service callback. This function is called when another part of the
             *  system requests the transform between the robot centre (base_link) and the
             *  sensor (camera).
             *  \param[in] req The service request.
             *  \param[out] res The service response.
             */
            bool sendRobot2Camera(slamon::GetTransform::Request& req,
                                  slamon::GetTransform::Response& res);


            /** \brief Service callback. This function is called when another part of the
             *  system requests the stereo camera projection matrices.
             *  \param[in] req The service request.
             *  \param[out] res The service response.
             */
            bool sendProjectionMatrices(slamon::GetProjectionMatrices::Request& req,
                                        slamon::GetProjectionMatrices::Response& res);


            /** \brief Image callback. This function is called when synchronized stereo image
             *  messages are received.
             *  \param[in] l_img_msg ROS message containing the left image.
             *  \param[in] r_img_msg ROS message containing the right image.
             *  \param[in] l_info_msg ROS message containing the info of the left image.
             *  \param[in] r_info_msg ROS message containing the info of the right image.
             */  
            void callback(const sensor_msgs::ImageConstPtr& l_img_msg,
                          const sensor_msgs::ImageConstPtr& r_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                          const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Initialise the visual odometer by setting the transform between the camera
             *  and the centre of the robot and the projection matrices.
             *  \param[in] l_info_msg ROS message containing the info of the left image.
             *  \param[in] r_info_msg ROS message containing the info of the right image.
             *  @return Whether the SLAM could be initialised.
             */
            bool initialisation(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Detects (using SIFT), matches and filters the keypoints of the current
             *  stereo image pair. It then retrieves the 3D points from the scene and removes the
             *  "far" ones. A 3D keypoint is considered "far away" when its z is more than 40 times
             *  the stereo baseline. "Far" 3D keypoints are removed because they do not provide
             *  accurate scale and translation information (Paz et al. 2008 and Mur-Artal et al.
             *  2017).
             *  \param[in] l_img_msg ROS message containing the left image.
             *  \param[in] r_img_msg ROS message containing the right image.
             *  \param[out] c_frame Will contain all the corresponfing data of the current frame.
             *  @return Whether the current frame could be processed.
             */
            bool processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                     const sensor_msgs::ImageConstPtr& r_img_msg,
                                     Frame& c_frame);


            /** \brief First, it matches and filters the keypoints of successive frames. 
             *  These actions can be carried out in a "plain" or "circular" manner, the user chooses.
             *  Filtering is performed using Lowe's ratio, computing the best homography (with RANSAC)
             *  or computing the best essential matrix (with RANSAC). Finally, the movement between
             *  frames is estimated using the 3D to 2D method (PnP + RANSAC).
             *  \param[in] c_frame Current stereo frame.
             *  \param[out] camera_delta_odom Estimated movement between the two images, referenced to
             *  camera coordinate axis.
             *  @return Whether it was possible to match keypoints, filter outliers and estimate movement. 
             */ 
            bool computeOdometry(const Frame& c_frame,
                                 tf::Transform& camera_delta_odom);


            /** \brief Initialises the robot odometry relative to the robot coordinate axis. To do this,
             *  it calls a service and waits for another part of the system to respond. If there is a
             *  response, the received odometry is assigned, otherwise the odometry is initialised to 0
             *  (identity).
             */
            void getInitialOdom();


            /** \brief Publish a keyframe. Depending on enable_debugging_ the keyframe contains the images.
             *  \param[in] inliers_per_pair Number of inliers per cluster pair.
             *  \param[in] cluster_pairs Cluster pairs.
             */
            void sendKeyframe(const std::vector<int>& inliers_per_pair,
                              const std::vector<std::pair<int, int>>& cluster_pairs);


            /** \brief Integrates and publishes odometry.
             *  \param[in] c_frame Current frame.
             *  \param[in] camera_delta_odom Estimated movement between the two images, referenced to
             *  camera coordinate axis.
             */
            void integrateAndPublish(const Frame& c_frame,
                                     tf::Transform& camera_delta_odom);


            /** \brief Sends a keyframe if the map needs it
             *  \param[in] c_frame Current frame.
             */
            void sendKeyframeIfNecessary(const Frame& c_frame);


            /** \brief Detects keypoints in ROS images. To do this, it transforms images of 
             *  sensor_msgs::ImageConstPtr type to cv::Mat type, and then detects the
             *  keypoints and calculates the descriptors.
             *  \param[in] max_kpts Maximum number of keypoints to be retained.
             *  \param[in] enable_enhacement If true, the image is enhanced using CLAHE.
             *  \param[in] feature_detector_type String containing the name of the desired detector for detecting
             *  image keypoints.
             *  \param[in] img_msg ROS message containing an image.
             *  \param[out] img cv::Mat image.
             *  \param[out] descs Descriptors.
             *  \param[out] kpts Keypoints.
             *  @return If the conversion has been possible.
             */
            bool detectROSKeypoints(const int max_kpts,
                                    const bool enable_enhacement,
                                    const std::string feature_detector_type,
                                    const sensor_msgs::ImageConstPtr& img_msg,
                                    cv::Mat& img,
                                    cv::Mat& descs,
                                    std::vector<cv::KeyPoint>& kpts);


            // Atributes:

            int c_keyframe_id_; //!> Current keyframe id.

            int c_cluster_id_; //!> Current cluster id.
            
            int img_height_; //!> Number of image rows.
            
            int img_width_; //!> Number of image cols.
            
            int min_inliers_; //!> Minimum number of inliers to estimate the movement.

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            int max_kpts_bucket_; //!> Maximum number of keypoints per bucket.

            int max_kpts_retained_; //!> Maximum number of keypoints retained at the detection stage.

            cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Left projection matrix.

            cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Right projection matrix.
            
            Frame p_frame_; //!> Previous frame.

            Frame keyframe_; //!> Current keyframe.
            
            VOStatus status_; //!> Visual odometer status.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            bool enable_bucketing_; //!> Enabling or disabling keypoint bucketing before motion estimation.

            bool enable_enhacement_; //!> Enabling or disabling image enhacement.

            bool initialised_odom_; //!> Flag that indicates if the robot's odom has been initialised.

            std::string robot_frame_id_; //!> Robot odometry coordinate axis identficator.

            std::string camera_frame_id_; //!> Camera odometry coordinate axis identficator.

            tf::Transform c_robot_odom_; //!> Current robot odometry.

            tf::Transform c_camera_odom_; //!> Current camera odometry.

            std::string feature_detector_name_; //!> The name of the feature detector.

            tf::StampedTransform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

            ros::Publisher keyframe_pub_; //!> Publisher for the current keyframe.
            
            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.
            
            ros::Publisher robot_odom_pub_; //!> Publishes current velocity and accumulated movement (odometry) relative to robot.
            
            ros::Publisher camera_odom_pub_; //!> Publishes current velocity and accumulated movement (odometry) relative to camera.
            
            ros::Publisher inliers_num_pub_; //!> Publisher for the number of inliers.
            
            ros::Publisher inliers_img_pub_; //!> Publisher for the image with drawn inliers.
            
            ros::Publisher stereo_kpts_img_pub_; //!> Publisher for the image with drawn stereo keypoints.
            
            ros::Publisher stereo_matches_num_pub_; //!> Publisher for the number of stereo matches.
            
            ros::Publisher stereo_matches_img_pub_; //!> Publisher for the image with drawn stereo matches.
            
            ros::Publisher temporal_matches_num_pub_; //!> Publisher for the number of temporal matches.
            
            ros::Publisher temporal_matches_img_pub_; //!> Publisher for the image with drawn temporal matches.
            
            ros::Publisher keyframe_inliers_img_pub_; //!> Publisher for the inliers between current and key frames.

            ros::ServiceClient initial_odom_client_; //!> Service for requesting the initial position to the SLAM.
            
            ros::ServiceServer robot2camera_server_; //!> Service for sending robot2camera transform.

            ros::ServiceServer proj_mat_server_; //!> Service for sending projection matrices.

            RunTimes run_times_; //!> Internal visual odometer execution times.

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                    sensor_msgs::Image,
                                                                    sensor_msgs::CameraInfo,
                                                                    sensor_msgs::CameraInfo> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
    };


} // End namespace.


#endif // VISUAL_ODOMETER_HPP