/**
 *  @file
 *  @brief The Visual Odometer class is responsive to track consecutive image frames and compute their motion.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter-Spring 2024
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
#include "vision_motion_modules.hpp"

#include "slamon/GetTransform.h"
#include "slamon/GetProjectionMatrices.h"


namespace slamon
{


    class VisualOdometer: protected Publisher, protected VisionMotionModules 
    {
        public:

            enum VOStatus
            {
                UNINITIALISED    = 0,
                SEMI_INITIALISED = 1,
                RUNNING          = 2
            };


            /** \brief Class constructor.
             *  \param ratio The ratio for Lowe's ratio test.
             *  \param min_inliers The minimum number of keypoints to estimate the movement.
             *  \param enable_debugging Enabling or disabling debugging.
             *  \param w_path Working path for debugging purpose.
             *  \param matching_type Matching type.
             *  \param robot_frame_id Robot odometry coordinate axis identficator.
             *  \param motion_estimation_type Motion estimation type.
             */
            VisualOdometer(double ratio,
                           int min_inliers,
                           bool enable_debugging,
                           bool enable_enhacement,
                           const std::string& w_path,
                           std::string& matching_type,
                           std::string& robot_frame_id,
                           std::string& motion_estimation_type);


            /** \brief Starts tracking
             */
            void run();


        private:


            /** \brief Image callback. This function is called when synchronized stereo image
             *  messages are received.
             *  \param l_img_msg ROS message containing the left image (input).
             *  \param r_img_msg ROS message containing the right image (input).
             *  \param l_info_msg ROS message containing the info of the left image (input).
             *  \param r_info_msg ROS message containing the info of the right image (input).
             */  
            void callback(const sensor_msgs::ImageConstPtr& l_img_msg,
                          const sensor_msgs::ImageConstPtr& r_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                          const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Service callback. This function is called when another part of the
             *  system requests the transform between the robot centre (base_link) and the
             *  sensor (camera).
             *  \param req The service request (input).
             *  \param res The service response (output).
             */
            bool sendRobot2Camera(slamon::GetTransform::Request& req,
                                  slamon::GetTransform::Response& res);


            /** \brief Service callback. This function is called when another part of the
             *  system requests the stereo camera projection matrices.
             *  \param req The service request (input).
             *  \param res The service response (output).
             */
            bool sendProjectionMatrices(slamon::GetProjectionMatrices::Request& req,
                                        slamon::GetProjectionMatrices::Response& res);


            /** \brief Initialise the visual odometer by setting the transform between the camera
             *  and the centre of the robot and the projection matrices.
             *  \param l_info_msg ROS message containing the info of the left image (input).
             *  \param r_info_msg ROS message containing the info of the right image (input).
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
             *  \param l_img_msg ROS message containing the left image (input).
             *  \param r_img_msg ROS message containing the right image (input).
             *  \param c_frame Will contain all the corresponfing data of the current frame (output).
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
             *  \param c_frame Current stereo frame (input).
             *  \param robot_motion Estimated movement between the two images, referenced to
             *  robot coordinate axis.
             *  @return Whether it was possible to match keypoints, filter outliers and estimate movement. 
             */ 
            bool computeMotionSuccessiveFrames(const Frame& c_frame,
                                               tf::Transform& robot_motion);


            /** \brief Initialises the robot odometry relative to the robot coordinate axis. To do this,
             *  it calls a service and waits for another part of the system to respond. If there is a
             *  response, the received odometry is assigned, otherwise the odometry is initialised to 0
             *  (identity).
             */                        
            void getInitialOdom();


            /** \brief Detects keypoints in ROS images. To do this, it transforms images of 
             *  sensor_msgs::ImageConstPtr type to cv::Mat type, and then detects the
             *  keypoints and calculates the descriptors.
             *  \param enable_enhacement If true, the image is enhanced using CLAHE (input).
             *  \param feature_detector_type String containing the name of the desired detector for detecting
             *  image keypoints (input).
             *  \param img_msg ROS message containing an image (input).
             *  \param img cv::Mat image (output).
             *  \param descs Descriptors (output).
             *  \param kpts Keypoints (output).
             *  @return If the conversion has been possible.
             */
            bool detectROSKeypoints(const bool enable_enhacement,
                                    const std::string& feature_detector_type,
                                    const sensor_msgs::ImageConstPtr& img_msg,
                                    cv::Mat& img,
                                    cv::Mat& descs,
                                    std::vector<cv::Point2f>& kpts);


            /** \brief Convert ROS image message to cv::Mat
             *  \param msg ROS message containing an img (input).
             *  \param img Will contain the image in cv::Mat type (output).
             *  @return If the conversion has been possible.
             */
            bool fromImgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                                 cv::Mat& img);
            
            // Atributes
            int c_frame_id_; //!> Current frame id.

            int min_inliers_; //!> The minimum number of inliers to estimate the movement.

            Frame p_frame_; //!> Previous frame.
            
            VOStatus status_; //!> Visual odometer state.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            bool enable_enhacement_; //!> Enabling or disabling image enhacement.

            bool initialised_odom_; //!> Flag that indicates if the robot's odom has been initialised.

            std::string w_path_; //!> Path to working directory.

            std::string frame_path_; //!> Path to store frames with debugging purpose.

            std::string robot_frame_id_; //!> Robot odometry coordinate axis identficator.

            std::string camera_frame_id_; //!> Camera odometry coordinate axis identficator.

            tf::Transform c_robot_odom_; //!> Current robot odometry.

            tf::Transform c_camera_odom_; //!> Current camera odometry.

            tf::StampedTransform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

            ros::Publisher robot_odom_pub_; //!> Publishes current velocity and accumulated movement (odometry) relative to robot.

            ros::Publisher camera_odom_pub_; //!> Publishes current velocity and accumulated movement (odometry) relative to camera.

            ros::Publisher stereo_data_pub_; //!> Publishes current relevant information from a stereo pair of images.

            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher inliers_img_pub_; // !> Publisher for the image with drawn inliers.

            ros::Publisher stereo_kps_img_pub_; // !> Publisher for the image with drawn stereo keypoints.

            ros::Publisher stereo_matches_img_pub_; // !> Publisher for the image with drawn stereo matches.

            ros::Publisher successive_matches_img_pub_; // !> Publisher for the image with drawn successive matches.

            ros::ServiceClient initial_odom_client_; //!> Service for requesting the initial position to the SLAM.
            
            ros::ServiceServer robot2camera_server_; // !> Service for sending robot2camera transform.

            ros::ServiceServer proj_mat_server_; // !> Service for sending projection matrices.

            RunTimes run_times_; // !> Internal visual odometer execution times.

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                    sensor_msgs::Image,
                                                                    sensor_msgs::CameraInfo,
                                                                    sensor_msgs::CameraInfo> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
    };


} // End namespace.


#endif // VISUAL_ODOMETER_HPP