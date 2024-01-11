#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP


#include "tools.hpp"
#include "publisher.hpp"
#include "stereo_motion_estimation_modules.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>


class VisualOdometry: protected Tools, Publisher, StereoMotionEstimationModules 
{
    public:

        enum visualOdometryState
        {
            NOT_INITIALIZED   = 0,
            INITIALIZING      = 1,
            WORKING           = 2,
            LOST              = 3
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
         * messages are received.
         * \param l_img_msg ROS message containing the left image (input).
         * \param r_img_msg ROS message containing the right image (input).
         * \param l_info_msg ROS message containing the info of the left image (input).
         * \param r_info_msg ROS message containing the info of the right image (input).
         */  
        void imageCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                           const sensor_msgs::ImageConstPtr& r_img_msg,
                           const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                           const sensor_msgs::CameraInfoConstPtr& r_info_msg);


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


        /** \brief Gets the stereo camera model and the camera matrix from the ROS 
         *  messages containing the image information.
         *  \param l_info_msg ROS message containing the info of the left image (input).
         *  \param r_info_msg ROS message containing the info of the right image (input).
         *  @return If it has been possible to obtain the model of the camera
         */
        void getCameraModelMatrix(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg);


        /** \brief Calculates the 3D points of the keypoints of the image stereo pair.
         *  The points are related to the camera frame. 
         *  \param l_kps Left image keypoints (input).
         *  \param r_kps Right image keypoints (input).
         *  @return The 3D camera points.
         */
        std::vector<cv::Point3f> compute3DPoints(const cv::Mat& l_descs,
                                                 const cv::Mat& r_descs,
                                                 const std::vector<cv::DMatch>& matches,
                                                 const std::vector<cv::KeyPoint>& l_kps,
                                                 const std::vector<cv::KeyPoint>& r_kps,
                                                 cv::Mat& f_l_descs,
                                                 cv::Mat& f_r_descs,
                                                 std::vector<cv::DMatch>& f_matches,
                                                 std::vector<cv::KeyPoint>& f_l_kps,
                                                 std::vector<cv::KeyPoint>& f_r_kps);

    private:

        cv::Mat camera_matrix_; //!> Stereo camera matrix

        image_transport::Publisher stereo_kps_pub_; // !> Stereo keypoints publisher

        image_transport::Publisher stereo_matches_pub_; // !> Stereo matches publisher

        image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                sensor_msgs::CameraInfo> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

};

#endif // STEREO_MOTION_ESTIMATION_HPP