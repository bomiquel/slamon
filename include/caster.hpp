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

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include "frame.hpp" // Frame class
#include "slamon/Frame.h" // Frame ROS msg


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
        bool fromMatToTf(const cv::Mat& r_vec,
                         const cv::Mat& t_vec,
                         tf::Transform& tf);


        /** \brief Convert custom ROS frame msg to frame object (slam::Frame)
         *  \param msg ROS message with relevant visual information (input).
         *  \param frame Will contain the relevant visual information (output).
         */
        void fromFrameMsgToFrameObject(const slamon::FrameConstPtr& msg,
                                       Frame& frame);


        /** \brief Convert cv::Point3f to tf::Transform.
         *  \param pt 3D point (input).
         *  @return 3D point converted into a transform.
         */
        tf::Transform fromPoint3DToTf(const cv::Point3f& pt);


        /** \brief Convert a tf::Transform to Eigen::Isometry3d
         *  \param tf The tf::Transform (input).
         *  @return The Eigen::Isometry3d matrix.
         */
        Eigen::Isometry3d fromTfToIsometry(const tf::Transform& tf);


        /** \brief Converts an integer to a string containing a
         *  five-digit number.
         *  \param in Input integer (input).
         *  @return The string containing the five-digit number.
         */
        std::string convertTo5Digits(int in);
};


#endif // CASTER_HPP