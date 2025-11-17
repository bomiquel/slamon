/**
 *  @file tools.hpp
 *  @brief Tools library.
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group 
 *  (srv.uib.es), University of the Balearic Islands. The original work is 
 *  licensed under the BSD 4-Clause License (see below).
 *
 *  @author
 *  Original: 
 *    Pep Lluís Negre-Carrasco
 *  Modified by: 
 *    Bo Miquel Nordfeldt-Fiol
 * 
 *  @copyright
 *  Original copyright holder:
 *      Systems, Robotics and Vision Group (srv.uib.es)
 *      University of the Balearic Islands
 *      Copyright (c) 2013
 *  Modified by:
 *      Bo Miquel Nordfeldt-Fiol (2024–2025)
 *      Copyright (c) 2025
 *      Released under the BSD 3-Clause License.
 *
 *  ---------------------------------------------------------------------------
 *  Original BSD 4-Clause License Notice:
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * All advertising materials mentioning features or use of this software
 *        must display the following acknowledgement:
 *          "This product includes software developed by the
 *           Systems, Robotics and Vision Group,
 *           University of the Balearic Islands"
 *      * Neither the name of the Systems, Robotics and Vision Group,
 *        University of the Balearic Islands nor the names of its contributors
 *        may be used to endorse or promote products derived from this software
 *        without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 *  ---------------------------------------------------------------------------
 *
 *  Additional license for modifications:
 *      The modifications made by Bo Miquel Nordfeldt-Fiol are released under
 *      the BSD 3-Clause License. See the top-level LICENSE file for details.
 */

#ifndef TOOLS_HPP
#define TOOLS_HPP


#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>


namespace slamon
{


    /** \brief Converts cv::Mat into tf::Transform.
     *  \param[in] r_vec cv::Mat containing the Euler angles.
     *  \param[in] t_vec cv::Mat containing the translation vector.
     *  \param[out] tf The resulting transform.
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


    /** \brief Computes the absolute diference between 2 poses (omitting z).
     *  \param[in] pose_0 transformation matrix of pose 0
     *  \param[in] pose_1 transformation matrix of pose 1
     *  @return the norm between two poses
     */
    static inline double poseDiff2D(const tf::Transform& pose_0, 
                                    const tf::Transform& pose_1)
    {
        tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
        return sqrt(d.x()*d.x() + d.y()*d.y());
    }


    /** \brief Computes the absolute diference between 2 poses.
     *  \param[in] pose_0 transformation matrix of pose 0
     *  \param[in] pose_1 transformation matrix of pose 1
     *  @return the norm between two poses
     */
    static inline double poseDiff3D(const tf::Transform& pose_0,
                                    const tf::Transform& pose_1)
    {
        tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
        return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
    }


    /** \brief Non-linear normalisation.
     *  \param[in] max The maximum expected value.
     *  \param[in] value The current value to standardize.
     *  @return Normalised value.
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


    /** \brief Convert ROS image message to cv::Mat
     *  \param[in] msg ROS message containing an img.
     *  \param[out] img Will contain the image in cv::Mat type.
     *  @return If the conversion has been possible.
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


}   // End namespace


#endif // TOOLS_HPP