/**
 *  @file
 *  @brief Tools library.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024
 */


#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <ros/ros.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>


static inline bool fromMatToTf(const cv::Mat& r_vec,
                               const cv::Mat& t_vec,
                               tf::Transform& tf)
{
    // Sanity check
    if (r_vec.empty() || t_vec.empty())
    {
        ROS_ERROR_STREAM("[Caster:] Something is wrong, one or both of the movement matrices is empty.");
        return false;
    }

    // Build the quaternion of rotation
    tf::Vector3 axis(r_vec.at<double>(0, 0),
                     r_vec.at<double>(1, 0),
                     r_vec.at<double>(2, 0));
    double angle = norm(r_vec);
    tf::Quaternion q(axis, angle);

    // Build the vector of translation
    tf::Vector3 t(t_vec.at<double>(0, 0),
                  t_vec.at<double>(1, 0),
                  t_vec.at<double>(2, 0));
    
    // Build the coordinates transform
    tf.setRotation(q);
    tf.setOrigin(t);

    return true;
}


#endif // TOOLS_HPP