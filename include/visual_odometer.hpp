/**
 *  \file visual_odometer.hpp
 *  \brief The VisualOdometer class is responsible for tracking consecutive stereo 
 *         image frames and computing the 6-DoF camera and robot odometry.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */


#ifndef VISUAL_ODOMETER_HPP
#define VISUAL_ODOMETER_HPP

#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{


    /** \class VisualOdometer
     *  \brief Stereo visual odometry front-end that tracks consecutive frames, estimates 6-DoF motion, 
     *         and selects keyframes for the pose-graph map.
     *  
     *  This class synchronizes incoming stereo image pairs and camera calibration messages, performs 
     *  keypoint detection, stereo triangulation, spatial clustering, temporal feature matching, 
     *  keypoint bucketing, and RANSAC PnP motion estimation. It also manages keyframe selection 
     *  and publishes robot and camera odometry.
     */
    class VisualOdometer: protected KeypointDetector, protected KeypointMatcher, protected MotionEstimator, protected Publisher
    {
        public:

            /** \enum VOStatus
             *  \brief Operational states of the stereo visual odometer.
             */
            enum VOStatus
            {
                /** \brief Initial state: retrieves the robot-to-camera transform, builds stereo projection matrices, and processes the first frame. */
                UNINITIALISED    = 0,

                /** \brief Calibration is complete, waiting to successfully extract features and 3D points from a reference stereo frame. */
                BOOTSTRAPPING    = 1,

                /** \brief A valid previous frame is available, attempting to establish inter-frame motion estimation and insert a reference keyframe. */
                LOST             = 2,

                /** \brief Nominal tracking state: continuously estimating frame-to-frame motion, integrating odometry, and selecting keyframes. */
                RUNNING          = 3
            };


            /** \brief Parameterized class constructor.
             *  \param[in] min_inliers Minimum number of geometrical inliers required to accept a motion estimate.
             *  \param[in] min_lc_inliers Minimum number of inliers required for loop closure.
             *  \param[in] max_kpts_bucket Maximum number of keypoints to retain per grid bucket.
             *  \param[in] max_kpts_retained Maximum number of keypoints to retain during feature detection.
             *  \param[in] enable_debugging If `true`, enables debugging messages and visualization topics.
             *  \param[in] enable_bucketing If `true`, enables spatial keypoint bucketing prior to motion estimation.
             *  \param[in] enable_enhacement If `true`, enables CLAHE image contrast enhancement.
             *  \param[in] matching_type String identifier of the desired keypoint matching strategy.
             *  \param[in] robot_frame_id Robot base coordinate frame ID (e.g., `"base_link"`).
             *  \param[in] feature_detector_name Name of the feature detector/descriptor algorithm (e.g., `"ORB"`, `"SIFT"`).
             *  \param[in] motion_estimation_type String identifier of the desired PnP motion estimation algorithm.
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


            /** \brief Starts the visual odometer execution loop.
             */
            void run();


        private:


            /** \brief ROS service callback that provides the static transformation between the robot base frame 
             *         (`base_link`) and the camera sensor frame.
             *  \param[in] req Service request.
             *  \param[out] res Service response populated with the robot-to-camera transform.
             *  \return `true` if the transform was successfully populated, `false` otherwise.
             */
            bool sendRobot2Camera(slamon::GetTransform::Request& req,
                                  slamon::GetTransform::Response& res);


            /** \brief ROS service callback that provides the left and right stereo camera projection matrices.
             *  \param[in] req Service request.
             *  \param[out] res Service response populated with the stereo projection matrices.
             *  \return `true` if the projection matrices were successfully populated, `false` otherwise.
             */
            bool sendProjectionMatrices(slamon::GetProjectionMatrices::Request& req,
                                        slamon::GetProjectionMatrices::Response& res);


            /** \brief Synchronized stereo callback triggered when left/right images and camera info messages arrive.
             *  \param[in] l_img_msg Incoming ROS message containing the left stereo image.
             *  \param[in] r_img_msg Incoming ROS message containing the right stereo image.
             *  \param[in] l_info_msg Incoming ROS message containing the left camera calibration info.
             *  \param[in] r_info_msg Incoming ROS message containing the right camera calibration info.
             */
            void callback(const sensor_msgs::ImageConstPtr& l_img_msg,
                          const sensor_msgs::ImageConstPtr& r_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                          const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Initializes the visual odometer by looking up the robot-to-camera transform 
             *         and building the stereo projection matrices.
             *  \param[in] l_info_msg ROS message containing the left camera calibration info.
             *  \param[in] r_info_msg ROS message containing the right camera calibration info.
             *  \return `true` if the visual odometer was successfully initialized, `false` otherwise.
             */
            bool initialisation(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Detects, matches, and filters keypoints from the current stereo image pair, 
             *         triangulates 3D points, and performs spatial clustering.
             *  
             *  After stereo triangulation, 3D points categorized as "far" (depth exceeding 40 times 
             *  the stereo baseline) are removed because they do not provide accurate scale and 
             *  translation information (Paz et al., 2008; Mur-Artal et al., 2017).
             *  
             *  \param[in] l_img_msg ROS message containing the left stereo image.
             *  \param[in] r_img_msg ROS message containing the right stereo image.
             *  \param[out] c_frame Output `Frame` object populated with all extracted data from the current stereo pair.
             *  \return `true` if the current frame was successfully processed, `false` otherwise.
             */
            bool processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                     const sensor_msgs::ImageConstPtr& r_img_msg,
                                     Frame& c_frame);


            /** \brief Matches keypoints between consecutive stereo frames and estimates the relative 6-DoF camera motion.
             *  
             *  Feature matching and outlier rejection are carried out using plain or circular stereo consistency 
             *  combined with Lowe's ratio test or RANSAC homography. Optional keypoint bucketing is then applied, 
             *  followed by refined 3D-to-2D motion estimation (PnP + RANSAC).
             *  
             *  \param[in] c_frame Current processed stereo frame.
             *  \param[out] camera_delta_odom Estimated relative 6-DoF motion between the previous and current frames in the camera coordinate frame.
             *  \return `true` if feature matching and motion estimation succeeded with enough inliers, `false` otherwise.
             */
            bool computeOdometry(const Frame& c_frame,
                                 tf2::Transform& camera_delta_odom);


            /** \brief Initializes the global robot odometry pose.
             *  
             *  Calls the initial pose service and waits for a response. If the service responds, 
             *  the received pose is assigned; otherwise, the odometry is initialized to the identity transform.
             */
            void getInitialOdom();


            /** \brief Publishes the current keyframe message to the mapmaker node.
             *  
             *  Depending on `enable_debugging_`, the published keyframe message may also include the raw stereo images.
             *  
             *  \param[in] inliers_per_pair Number of matched inliers per cluster pair.
             *  \param[in] cluster_pairs Vector of matched cluster ID pairs.
             */
            void sendKeyframe(const std::vector<int>& inliers_per_pair,
                              const std::vector<std::pair<int, int>>& cluster_pairs);


            /** \brief Integrates the incremental camera motion into the global odometry and publishes the updated poses and twist.
             *  \param[in] c_frame Current stereo frame.
             *  \param[in] camera_delta_odom Estimated relative 6-DoF camera motion between the previous and current frames.
             */
            void integrateAndPublish(const Frame& c_frame,
                                     tf2::Transform& camera_delta_odom);


            /** \brief Evaluates keyframe selection criteria and publishes a new keyframe if required by the map.
             *  \param[in] c_frame Current stereo frame.
             */
            void sendKeyframeIfNecessary(const Frame& c_frame);


            /** \brief Converts a ROS image message to an OpenCV matrix and extracts 2D keypoints and feature descriptors.
             *  \param[in] max_kpts Maximum number of keypoints to retain.
             *  \param[in] enable_enhacement If `true`, pre-processes the image using CLAHE.
             *  \param[in] feature_detector_type Name of the feature detector/descriptor algorithm to use.
             *  \param[in] img_msg Incoming ROS message containing the image.
             *  \param[out] img Converted OpenCV `cv::Mat` image.
             *  \param[out] descs Output matrix of extracted feature descriptors.
             *  \param[out] kpts Output vector of detected 2D keypoints.
             *  \return `true` if image conversion and feature extraction succeeded, `false` otherwise.
             */
            bool detectROSKeypoints(const int max_kpts,
                                    const bool enable_enhacement,
                                    const std::string feature_detector_type,
                                    const sensor_msgs::ImageConstPtr& img_msg,
                                    cv::Mat& img,
                                    cv::Mat& descs,
                                    std::vector<cv::KeyPoint>& kpts);


            /** \brief Current keyframe ID counter. */
            int c_keyframe_id_;

            /** \brief Current global cluster ID counter. */
            int c_cluster_id_;
            
            /** \brief Image height (number of rows) [px]. */
            int img_height_;
            
            /** \brief Image width (number of columns) [px]. */
            int img_width_;
            
            /** \brief Minimum number of geometrical inliers required to accept a motion estimate. */
            int min_inliers_;

            /** \brief Minimum number of inliers required to close a loop. */
            int min_lc_inliers_;

            /** \brief Maximum number of keypoints retained per grid bucket. */
            int max_kpts_bucket_;

            /** \brief Maximum number of keypoints retained at the feature detection stage. */
            int max_kpts_retained_;

            /** \brief Left stereo camera 3x4 projection matrix. */
            cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);

            /** \brief Right stereo camera 3x4 projection matrix. */
            cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);
            
            /** \brief Previously processed stereo frame. */
            Frame p_frame_;

            /** \brief Active reference keyframe. */
            Frame keyframe_;
            
            /** \brief Current operational state of the visual odometer. */
            VOStatus status_;

            /** \brief Flag to enable or disable debugging messages and visualization topics. */
            bool enable_debugging_;

            /** \brief Flag to enable or disable spatial keypoint bucketing prior to motion estimation. */
            bool enable_bucketing_;

            /** \brief Flag to enable or disable CLAHE image contrast enhancement. */
            bool enable_enhacement_;

            /** \brief Flag indicating whether the initial robot odometry pose has been set. */
            bool initialised_odom_;

            /** \brief Robot base coordinate frame ID (e.g., `"base_link"`). */
            std::string robot_frame_id_;

            /** \brief Camera sensor coordinate frame ID. */
            std::string camera_frame_id_;

            /** \brief Name of the active feature detector/descriptor algorithm. */
            std::string feature_detector_name_;

            /** \brief Public ROS node handle. */
            ros::NodeHandle nh_;
            
            /** \brief Private ROS node handle (`"~"`). */
            ros::NodeHandle nhp_;

            /** \brief ImageTransport instance for subscribing to image streams. */
            image_transport::ImageTransport it_;

            /** \brief Current accumulated robot odometry pose. */
            tf2::Transform c_robot_odom_;

            /** \brief Current accumulated camera odometry pose. */
            tf2::Transform c_camera_odom_;

            /** \brief Static transformation between the robot base frame (`base_link`) and the camera frame. */
            tf2::Transform robot2camera_;

            /** \brief TF2 buffer used to store and look up coordinate frame transforms. */
            tf2_ros::Buffer tf_buffer_;

            /** \brief TF2 transform listener that populates the transform buffer. */
            tf2_ros::TransformListener tf_listener_;

            /** \brief ROS publisher for newly selected keyframes. */
            ros::Publisher keyframe_pub_;
            
            /** \brief ROS publisher for per-iteration visual odometry execution times. */
            ros::Publisher run_times_pub_;
            
            /** \brief ROS publisher for the accumulated robot odometry pose and velocity twist. */
            ros::Publisher robot_odom_pub_;
            
            /** \brief ROS publisher for the accumulated camera odometry pose and velocity twist. */
            ros::Publisher camera_odom_pub_;
            
            /** \brief ROS publisher for the number of motion estimation inliers. */
            ros::Publisher inliers_num_pub_;
            
            /** \brief ROS publisher for the visualization image of motion estimation inliers. */
            ros::Publisher inliers_img_pub_;
            
            /** \brief ROS publisher for the visualization image of detected stereo keypoints. */
            ros::Publisher stereo_kpts_img_pub_;
            
            /** \brief ROS publisher for the number of intra-frame stereo matches. */
            ros::Publisher stereo_matches_num_pub_;
            
            /** \brief ROS publisher for the visualization image of intra-frame stereo matches. */
            ros::Publisher stereo_matches_img_pub_;
            
            /** \brief ROS publisher for the number of inter-frame temporal matches. */
            ros::Publisher temporal_matches_num_pub_;
            
            /** \brief ROS publisher for the visualization image of inter-frame temporal matches. */
            ros::Publisher temporal_matches_img_pub_;
            
            /** \brief ROS publisher for the visualization image of inliers between the current frame and the reference keyframe. */
            ros::Publisher keyframe_inliers_img_pub_;

            /** \brief ROS service client for requesting the initial robot pose. */
            ros::ServiceClient initial_odom_client_;
            
            /** \brief ROS service server for providing the static robot-to-camera transformation. */
            ros::ServiceServer robot2camera_server_;

            /** \brief ROS service server for providing the stereo camera projection matrices. */
            ros::ServiceServer proj_mat_server_;

            /** \brief ImageTransport subscriber filter for the left stereo image stream. */
            image_transport::SubscriberFilter l_img_sub_;

            /** \brief ImageTransport subscriber filter for the right stereo image stream. */
            image_transport::SubscriberFilter r_img_sub_;

            /** \brief Message filter subscriber for the left camera calibration info stream. */
            message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_;

            /** \brief Message filter subscriber for the right camera calibration info stream. */
            message_filters::Subscriber<sensor_msgs::CameraInfo> r_info_sub_;

            /** \brief Execution time profiler for internal visual odometry stages. */
            RunTimes run_times_;

            /** \brief Approximate-time synchronization policy for stereo images and camera info messages. */
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                    sensor_msgs::Image,
                                                                    sensor_msgs::CameraInfo,
                                                                    sensor_msgs::CameraInfo> SyncPolicy_;

            /** \brief Message filter synchronizer type using `SyncPolicy_`. */
            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
            
            /** \brief Shared pointer to the active message filter synchronizer instance. */
            std::shared_ptr<Sync_> sync_;
    };


} // namespace slamon


#endif // VISUAL_ODOMETER_HPP