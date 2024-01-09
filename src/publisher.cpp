#include "publisher.hpp"


void Publisher::publishImage(const image_transport::Publisher& pub, const cv::Mat& img, const ros::Time& stamp)
{
    cv_bridge::CvImage ros_image;
    ros_image.image = img;
    ros_image.header.stamp = stamp;
    ros_image.encoding = "bgr8";
    pub.publish(ros_image.toImageMsg());
}


