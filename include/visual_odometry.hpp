#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "stereo_motion_estimation_modules.hpp"


class VisualOdometry: public StereoMotionEstimationModules
{
    public:

        /** \brief Class constructor
         *  \param pub Frame publisher object pointer
         *  \param graph Graph object pointer
         */
        VisualOdometry() {}


        /** \brief Starts tracking
         */
        void run();

    protected:

        /** \brief Messages callback. This function is called when synchronized image
         * messages are received.
         * \param l_img_msg left stereo image message of type sensor_msgs::Image
         * \param r_img_msg right stereo image message of type sensor_msgs::Image
         * \param l_info_msg left stereo info message of type sensor_msgs::CameraInfo
         * \param r_info_msg right stereo info message of type sensor_msgs::CameraInfo
         */  
        void imageCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                           const sensor_msgs::ImageConstPtr& r_img_msg,
                           const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                           const sensor_msgs::CameraInfoConstPtr& r_info_msg);

    private:

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo,
                                                                sensor_msgs::CameraInfo> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

};

#endif // STEREO_MOTION_ESTIMATION_HPP