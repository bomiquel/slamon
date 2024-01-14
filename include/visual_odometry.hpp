/**
 *  @file
 *  @brief The Visual Odometry class is responsive to track consecutive image frames and compute their motion.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter 2024
 */

#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP


#include "frame.hpp"
#include "tools.hpp"
#include "publisher.hpp"
#include "stereo_motion_estimation_modules.hpp"

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>


class VisualOdometry: protected Tools, Publisher, StereoMotionEstimationModules 
{
    public:

        enum VOStatus
        {
            UNINITIALIZED   = 0,
            INITIALIZING    = 1,
            WORKING         = 2,
            LOST            = 3
        };


        /** \brief Class constructor.
         *  \param pub Frame publisher object pointer.
         *  \param graph Graph object pointer.
         */
        VisualOdometry() {}


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


        void initialization(const sensor_msgs::ImageConstPtr& l_img_msg,
                            const sensor_msgs::ImageConstPtr& r_img_msg,
                            const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                            const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                            Frame& c_frame);


        /** \brief Detects, matches, filters and calculates the 3D points of the keypoints
         *  of the current stereo image pair.
         *  \param l_img_msg ROS message containing the left image (input).
         *  \param r_img_msg ROS message containing the right image (input).
         *  \param c_frame Will contain all the corresponfing data of the current frame (output).
         *  @return Whether the current frame could be processed.
         */
        bool processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                 const sensor_msgs::ImageConstPtr& r_img_msg,
                                 Frame& c_frame);



        bool computeMotionSuccessiveFrames(const Frame& c_frame,
                                           const Frame& p_frame);


        /** \brief Get the static transform between robot frame (base link) and camera frame.
         *  \param robot_frame_id Name of the robot frame (input).
         *  \param sensor_frame_id Name of the sensor frame (input).
         *  \param transform Will contain the transform between robot frame and camera frame (output).
         *  @return True if valid transform, false otherwise.
         */
        bool getRobot2CameraTf(const std::string& robot_frame_id,
                               const std::string& sensor_frame_id,
                               tf::StampedTransform& transform);


        /** \brief Gets the stereo camera model and the camera matrix from the ROS 
         *  messages containing the image information.
         *  \param l_info_msg ROS message containing the info of the left image (input).
         *  \param r_info_msg ROS message containing the info of the right image (input).
         *  \param camera_matrix Will contain the stereo camera matrix (output).
         *  \param stereo_camera_model Will contain the stereo camera model (output).
         *  @return If it has been possible to obtain the model of the camera.
         */
        void getCameraModelMatrix(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                  cv::Mat& camera_matrix,
                                  image_geometry::StereoCameraModel& stereo_camera_model);


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


        /** \brief Calculates the 3D points of the keypoints of the image stereo pair.
         *  The points are related to the camera frame. 
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kps Left image keypoints (input).
         *  \param r_kps Right image keypoints (input).
         *  \param camera_model Stereo camera model matrix (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kps Filtered left image keypoints (output).
         *  \param r_l_kps Filtered right image keypoints (output).
         *  @return The 3D camera points.
         */
        std::vector<cv::Point3f> compute3DPoints(const cv::Mat& l_descs,
                                                 const cv::Mat& r_descs,
                                                 const std::vector<cv::KeyPoint>& l_kps,
                                                 const std::vector<cv::KeyPoint>& r_kps,
                                                 const image_geometry::StereoCameraModel& camera_model,
                                                 cv::Mat& f_l_descs,
                                                 cv::Mat& f_r_descs,
                                                 std::vector<cv::KeyPoint>& f_l_kps,
                                                 std::vector<cv::KeyPoint>& f_r_kps);

    private:
        
        Frame p_frame_; //!> Previous frame

        VOStatus status_; //!> Visual odometer state

        ros::Time start_time_; //>! Time at which the program enters into the callback.

        cv::Mat camera_matrix_; //!> Stereo camera matrix

        tf::StampedTransform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

        tf::TransformListener tf_listener_; //!> Listen for tf between robot and camera.

        image_transport::Publisher stereo_kps_pub_; // !> Stereo keypoints publisher

        image_transport::Publisher stereo_matches_pub_; // !> Stereo matches publisher

        image_transport::Publisher successive_matches_pub_; // !> Successive frame matching publisher 

        image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                sensor_msgs::CameraInfo> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

        // ros::Duration delta_t double delta_t_sec

};

#endif // STEREO_MOTION_ESTIMATION_HPP