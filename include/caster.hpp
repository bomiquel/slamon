/**
 *  @file
 *  @brief The Caster class is responsible for transforming from one complex type of variable to another complex type. 
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter-Spring 2024
 */


#ifndef CASTER_HPP
#define CASTER_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


class Caster
{
    public:
        
        /** \brief Empty class constructor.
         */
        Caster() {}

    
        /** \brief Convert ROS image message to cv::Mat
         *  \param msg ROS message containing an img (input).
         *  \param img Will contain the image in cv::Mat type (output).
         *  @return If the conversion has been possible.
         */
        bool fromImgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                             cv::Mat& img);


        /** \brief Convert motion cv::Mat to ROS transform
         *  \param r_vec OpenCV estimated rotation vector (input).
         *  \param t_vec OpenCV estimated translation vector (input).
         *  \param tf ROS estimated transform (output).
         *  @return If the cast could be made
         */
        bool fromMotionMatToTf(const cv::Mat& r_vec,
                               const cv::Mat& t_vec,
                               tf::Transform& tf);


        /** \brief Converts an integer to a string containing a
         *  five-digit number.
         *  \param in Input integer (input).
         *  @return The string containing the five-digit number.
        */
        std::string convertTo5digits(int in);
};


#endif // CASTER_HPP