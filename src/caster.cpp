#include "caster.hpp"


bool Caster::fromImgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                             cv::Mat& img)
{
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr img_ptr;
    try
    {
        // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. 
        // If SLAM uses only this cv_bridge::toCvCopy, we can copy
        // this function into the code and the warning will no longer appear.
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("[Caster:] cv_bridge exception: " << e.what());
        return false;
    }
    
    // Set the images
    img = img_ptr->image;
    
    return true;
}


bool Caster::fromMotionMatToTf(const cv::Mat& r_vec,
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

std::string Caster::convertTo5digits(int in)
{
    uint val = (uint)in;
    std::string output;
    int digits = 5;
    while (digits-- > 0) {
        output += ('0' + val % 10);
        val /= 10;
    }
    reverse(output.begin(), output.end());
    return output;
}