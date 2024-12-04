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

#include <opencv2/core.hpp>


/** \brief Converts cv::Mat into tf::Transform.
 *  \param r_vec cv::Mat containing the Euler angles (input).
 *  \param t_vec cv::Mat containing the translation vector (input).
 *  \param tf The resulting transform (output).
 *  @return whether the conversion could be done.
 */
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


/** \brief Computes the current depth by averaging all z-values of the 3D points 
 *  in the current image (which come from the keypoints).
 *  \param cam_pts Current left camera points (input).
 *  @return The current depth. 
 */
static inline double computeCurrentDepth(const std::vector<cv::Point3f>& cam_pts)
{
    double depth_mean = 0.0;
    if (cam_pts.size() > 0)
    {
        for (uint i = 0; i < cam_pts.size(); i++)
            depth_mean += cam_pts[i].z;

        depth_mean /= cam_pts.size();
    }
    return depth_mean;
}


#endif // TOOLS_HPP