/**
 *  \file tools.hpp
 *  \brief Utility functions for coordinate conversions, pose distance calculations, 
 *         and ROS image message handling.
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  \author Original: Pep Lluís Negre-Carrasco (2013)
 *  \author Modified: Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon). Both the original 
 *  work and the modifications are released under the BSD 4-Clause License. 
 *  See the top-level LICENSE file for details.
 */

 
#ifndef TOOLS_HPP
#define TOOLS_HPP


#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Transform.h>


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{


    /** \brief Converts OpenCV rotation and translation vectors into a `tf2::Transform`.
     *  \param[in] r_vec 3x1 `cv::Mat` containing the Rodrigues rotation vector.
     *  \param[in] t_vec 3x1 `cv::Mat` containing the translation vector.
     *  \param[out] tf Output `tf2::Transform` representing the 6-DoF transformation.
     *  \return `true` if the conversion was successful, `false` if either input matrix is empty.
     */
    static inline bool fromMatToTf(const cv::Mat& r_vec,
                                   const cv::Mat& t_vec,
                                   tf2::Transform& tf)
    {
        // Sanity check
        if (r_vec.empty() || t_vec.empty())
        {
            ROS_ERROR_STREAM("[Caster:] Something is wrong, one or both of the movement matrices is empty.");
            return false;
        }
        cv::Mat r_vec_d, t_vec_d;
        r_vec.convertTo(r_vec_d, CV_64F);
        t_vec.convertTo(t_vec_d, CV_64F);

        // Get rotation components.
        double rx = r_vec_d.at<double>(0);
        double ry = r_vec_d.at<double>(1);
        double rz = r_vec_d.at<double>(2);

        // Get angle.
        double angle = cv::norm(r_vec_d);

        // Build the quaternion.
        tf2::Quaternion q;
        if (angle < 1e-9) 
        {
            q = tf2::Quaternion::getIdentity();
        }
        else 
        {
            tf2::Vector3 axis(rx / angle, ry / angle, rz / angle);
            q.setRotation(axis, angle);
            q.normalize(); // Just in case.
        }

        // Build the vector of translation
        tf2::Vector3 t(t_vec_d.at<double>(0), t_vec_d.at<double>(1), t_vec_d.at<double>(2));

        // Build the coordinates transform
        tf.setRotation(q);
        tf.setOrigin(t);

        return true;
    }


    /** \brief Computes the 2D Euclidean translational distance between two poses (omitting the z-axis).
     *  \param[in] pose_0 First 6-DoF pose (`tf2::Transform`).
     *  \param[in] pose_1 Second 6-DoF pose (`tf2::Transform`).
     *  \return The 2D Euclidean distance between the two pose origins in the XY plane.
     */
    static inline double poseDiff2D(const tf2::Transform& pose_0, 
                                    const tf2::Transform& pose_1)
    {
        tf2::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
        return sqrt(d.x()*d.x() + d.y()*d.y());
    }


    /** \brief Computes the 3D Euclidean translational distance between two poses.
     *  \param[in] pose_0 First 6-DoF pose (`tf2::Transform`).
     *  \param[in] pose_1 Second 6-DoF pose (`tf2::Transform`).
     *  \return The 3D Euclidean distance between the two pose origins.
     */
    static inline double poseDiff3D(const tf2::Transform& pose_0,
                                    const tf2::Transform& pose_1)
    {
        tf2::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
        return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    }


    /** \brief Performs bounded non-linear normalization of a scalar value (e.g., for edge confidence weighting).
     *  \param[in] max Maximum expected reference value.
     *  \param[in] value Input scalar value to normalize.
     *  \return Normalized value clamped to the range `[0.05, 1.0]`.
     */
    static inline double normalisation(const double max,
                                       const double value)
    {
        // Minimum value.
        if (value == 0.0)
            return 0.05;

        // Avoids division by zero.
        if (max <= 0.0)
            return 0.05;

        // Normalisation.
        return std::clamp(value / max, 0.1, 1.0);
    }


    /** \brief Converts a ROS `sensor_msgs::Image` message into an OpenCV `cv::Mat` (BGR8 encoding).
     *  \param[in] msg Incoming ROS image message pointer.
     *  \param[out] img Output OpenCV `cv::Mat` image.
     *  \return `true` if the conversion succeeded, `false` if a `cv_bridge` exception occurred.
     */
    static inline bool fromImgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                                       cv::Mat& img)
    {
        // Convert message to cv::Mat
        cv_bridge::CvImagePtr img_ptr;
        try
        {
            // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. 
            img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("[Caster:] cv_bridge exception: " << e.what());
            return false;
        }

        // Set the images
        img = img_ptr->image;

        return true;
    }


} // namespace slamon


#endif // TOOLS_HPP