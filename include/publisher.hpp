#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>


class Publisher
{
    public:

        /** \brief Empty class constructor.
         */
        Publisher() {};


        /** \brief Publishes images
         *  \param pub The ROS publisher.
         *  \param img The image in cv::Mat type. 
         *  \param stamp The ROS timestamp.
         */
        void publishImage(const image_transport::Publisher& pub, const cv::Mat& img, const ros::Time& stamp);


    protected:


};


#endif // PUBLISHER_HPP