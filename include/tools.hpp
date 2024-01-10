#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


class Tools
{
    public:
        
        /** \brief Empty class constructor.
         */
        Tools() {}


        /** \brief Convert image messages to cv::Mat
         *  \param l_img_msg left stereo image message of type sensor_msgs::Image (input).
         *  \param r_img_msg right stereo image message of type sensor_msgs::Image (input).
         *  \param l_img Will contain the left image in cv::Mat type (output).
         *  \param r_img Will contain the right image in cv::Mat type (output).
         *  @return If the conversion has been possible.
         */
        bool imgMsgToMat(const sensor_msgs::ImageConstPtr& l_img_msg,
                         const sensor_msgs::ImageConstPtr& r_img_msg,
                         cv::Mat& l_img, 
                         cv::Mat& r_img);
};


#endif // TOOLS_HPP