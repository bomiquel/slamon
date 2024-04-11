/**
 *  @file
 *  @brief The Visual Odometry class is responsive to track consecutive image frames and compute their motion.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter 2024
 */

#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>

#include "frame.hpp"
#include "tools.hpp"
#include "publisher.hpp"
#include "stereo_motion_estimation_modules.hpp"

#include "slamon/GetPose.h"

class VisualOdometry: protected Tools, protected Publisher, protected StereoMotionEstimationModules 
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
         *  \param min_num_kps The minimum number of keypoints to estimate the movement.
         *  \param enable_debugging Enabling or disabling debugging messages.
         *  \param matching_type Matching type.
         *  \param odom_frame_id Odometry coordinate axis identficator.
         *  \param robot_frame_id Robot coordinate axis identficator (base link).
         */
        VisualOdometry(double ratio,
                       int min_num_kps,
                       bool enable_debugging,
                       const std::string& matching_type,
                       const std::string& odom_frame_id, 
                       const std::string& robot_frame_id);


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


        /** \brief
         *  \param l_info_msg ROS message containing the info of the left image (input).
         *  \param r_info_msg ROS message containing the info of the right image (input).
         *  @return Whether the SLAM could be initialised.
         */
        bool initialisation(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                            const sensor_msgs::CameraInfoConstPtr& r_info_msg);


        /** \brief Detects (using SIFT), matches and filters (using Lowe's ratio, homography,
         *  essential matrix or circular version of the above) the keypoints of the current
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


        /**
         * 
         */ 
        bool computeMotionSuccessiveFrames(const Frame& c_frame,
                                           const Frame& p_frame);


        /** \brief Convert ROS image messages to cv::Mat
         *  \param l_img_msg ROS message containing the left image (input).
         *  \param r_img_msg ROS message containing the right image (input).
         *  \param l_img Will contain the left image in cv::Mat type (output).
         *  \param r_img Will contain the right image in cv::Mat type (output).
         *  @return If the conversion has been possible.
         */
        bool imgMsgToMat(const sensor_msgs::ImageConstPtr& l_img_msg,
                         const sensor_msgs::ImageConstPtr& r_img_msg,
                         cv::Mat& l_img, 
                         cv::Mat& r_img);


        /** \brief Initialises the robot pose (robot coordinate axis). To do this,
         *  it calls a service and waits for another part of the system to respond.
         *  If there is a response, the received pose is assigned, otherwise the pose
         *  is initialised to 0 (identity).
         */                        
        void getInitialPose();


    private:
        
        // Atributes
        Frame p_frame_; //!> Previous frame
        
        VOStatus status_; //!> Visual odometer state

        ros::Time start_time_; //>! Time at which the program enters into the callback.

        std::string odom_frame_id_; //!> Odometry coordinate axis identficator.
        
        std::string robot_frame_id_; //!> Robot coordinate axis identficator (base link).

        tf::Transform c_robot_pose_; //!> Current robot pose.

        tf::StampedTransform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

        tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

        ros::ServiceClient initial_pose_client_; //!> Service client to assign the initial position to the SLAM.

        image_transport::Publisher stereo_kps_pub_; // !> Stereo keypoints publisher

        image_transport::Publisher stereo_matches_pub_; // !> Stereo matches publisher

        image_transport::Publisher successive_matches_pub_; // !> Successive frame matching publisher 

        image_geometry::StereoCameraModel stereo_camera_model_; //!> Stereo camera model

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                sensor_msgs::CameraInfo> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

};

#endif // VISUAL_ODOMETRY_HPP