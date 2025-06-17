/**
 *  @file
 *  @brief Tools library.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024-2025
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