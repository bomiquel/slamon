#ifndef CASTER_HPP
#define CASTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>


class Caster
{
    public:
        
        /** \brief Empty class constructor.
         */
        Caster() {}

    protected:


    /** \brief Convert ROS image message to cv::Mat
     *  \param msg ROS message containing an img (input).
     *  \param img Will contain the image in cv::Mat type (output).
     *  @return If the conversion has been possible.
     */
    bool imgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                     cv::Mat& img);

    


};


#endif // CASTER_HPP