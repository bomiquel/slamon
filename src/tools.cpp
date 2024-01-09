#include "tools.hpp"


bool Tools::imgMsgToMat(const sensor_msgs::ImageConstPtr& l_img_msg,
                        const sensor_msgs::ImageConstPtr& r_img_msg,
                        cv::Mat& l_img, 
                        cv::Mat& r_img)
{
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
    try
    {
        // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. If SLAM uses only this cv_bridge::toCvCopy, we can copy this function into the code and the warning will no longer appear.
        l_img_ptr = cv_bridge::toCvCopy(l_img_msg, sensor_msgs::image_encodings::BGR8);
        r_img_ptr = cv_bridge::toCvCopy(r_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("[SLAMON:] cv_bridge exception: %s", e.what());
        return false;
    }
    
    // Set the images
    l_img = l_img_ptr->image;
    r_img = r_img_ptr->image;
    return true;
}


cv::Mat Tools::drawPoints(int r, 
                          const cv::Mat& img, 
                          const cv::Scalar& color,
                          const std::vector<cv::Point2f>& pts)
{
    cv::Mat drawn_img;
    img.copyTo(drawn_img);

    for (uint i = 0; i < pts.size(); i++)
        cv::circle(drawn_img, pts[i], r, color, -1);
    
    return drawn_img;
}