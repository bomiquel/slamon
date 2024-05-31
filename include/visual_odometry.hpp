/**
 *  @file
 *  @brief The Visual Odometry class is responsive to track consecutive image frames and compute their motion.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter-Spring 2024
 */

#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

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
#include "caster.hpp"
#include "publisher.hpp"
#include "stereo_modules.hpp"

#include "slamon/GetTransform.h"


namespace slam
{


    class VisualOdometry: protected Caster, protected Publisher, protected StereoModules 
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
             *  \param motion_estimation_type Motion estimation type.
             *  \param robot_frame_id Robot coordinate axis identficator (base link).
             *  \param origin_frame_id Origin coordinate axis identficator.
             */
            VisualOdometry(double ratio,
                           int min_inliers,
                           bool enable_debugging,
                           const std::string& w_path,
                           std::string& matching_type,
                           std::string& motion_estimation_type,
                           const std::string& robot_frame_id,
                           const std::string& origin_frame_id);


            /** \brief Starts tracking
             */
            void run();


        protected:


            /** \brief Messages callback. This function is called when synchronized image
             *  messages are received.
             *  \param l_img_msg ROS message containing the left image (input).
             *  \param r_img_msg ROS message containing the right image (input).
             *  \param l_info_msg ROS message containing the info of the left image (input).
             *  \param r_info_msg ROS message containing the info of the right image (input).
             */  
            void imageCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                               const sensor_msgs::ImageConstPtr& r_img_msg,
                               const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                               const sensor_msgs::CameraInfoConstPtr& r_info_msg);


            /** \brief Service callback. This function is called when another part of the
             *  system requests the transform between the robot centre (base_link) and the
             *  sensor (camera).
             *  \param req The service request (input).
             *  \param res The service response (output).
             */
            bool getRobot2CameraCallback(slamon::GetTransform::Request& req,
                                         slamon::GetTransform::Response& res);


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


            /** \brief Initialises the robot pose (robot coordinate axis). To do this,
             *  it calls a service and waits for another part of the system to respond.
             *  If there is a response, the received pose is assigned, otherwise the pose
             *  is initialised to 0 (identity).
             */                        
            void getInitialPose();


            /** \brief Decodes the enum type StereoResults into a boolean and a message
             *  printed to the console (optional).
             *  \param title Extra info to know which module sends the result (input).
             *  \param result The stereo module result (input).
             *  @return Whether keypoint matching and filtering has been performed correctly
             *  and if there are enough filtered points.
             */
            bool decodeStereoResult(const std::string& title,
                                    const StereoResult& result);


        private:
            
            // Atributes
            int c_frame_id_; //!> Current frame id.

            Frame p_frame_; //!> Previous frame.
            
            VOStatus status_; //!> Visual odometer state.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            bool initialised_pose_; //!> Flag that indicates if the robot's pose has been initialised.

            std::string w_path_; //!> Path to working directory.

            std::string f_path_; //!> Path to store frames with debugging purpose.

            std::string robot_frame_id_; //!> Robot coordinate axis identficator (base link).

            std::string origin_frame_id_; //!> Odometry coordinate axis identficator.

            tf::Transform c_robot_pose_; //!> Current robot pose.

            tf::StampedTransform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

            ros::Publisher info_pub_; //!> Publishes last visual odometer last iteration performance.

            ros::Publisher odom_pub_; //!> Publishes current velocity and accumulated movement (odometry).

            ros::Publisher frame_pub_; //!> Publishes current relevant information from a stereo pair of images.

            ros::Publisher inliers_img_pub_; // !> Publisher for the image with drawn inliers.

            ros::Publisher stereo_kps_img_pub_; // !> Publisher for the image with drawn stereo keypoints.

            ros::Publisher stereo_matches_img_pub_; // !> Publisher for the image with drawn stereo matches.

            ros::Publisher successive_matches_img_pub_; // !> Publisher for the image with drawn successive matches.

            ros::ServiceClient initial_pose_client_; //!> Service client to assign the initial position to the SLAM.
            
            ros::ServiceServer robot2camera_server_; // !> Service server to send robot2camera transform to service client.

            VORunTimes run_times_; // !> Internal times of the visual odometer.

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                    sensor_msgs::Image,
                                                                    sensor_msgs::CameraInfo,
                                                                    sensor_msgs::CameraInfo> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
    };


} // End namespace.


#endif // VISUAL_ODOMETRY_HPP