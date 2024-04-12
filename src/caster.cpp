#include "caster.hpp"


bool Caster::imgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
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
        ROS_ERROR_STREAM("[Img msg to CV mat:] cv_bridge exception: " << e.what());
        return false;
    }
    
    // Set the images
    img = img_ptr->image;
    
    return true;
}