/**
 *  @file
 *  @brief The Publisher class is responsive to publish the image visualizations for debugging purposes (presentation).
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter 2024
 */

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


        /** \brief Publishes stereo images with its keypoints drawn.
         *  \param l_img The left image (input). 
         *  \param r_img The right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param l_kps The left image keypoints (input).
         *  \param r_kps The left image keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishStereoKeypoints(const cv::Mat& l_img,
                                    const cv::Mat& r_img,
                                    const ros::Time& stamp,
                                    const std::vector<cv::KeyPoint>& l_kps,
                                    const std::vector<cv::KeyPoint>& r_kps,
                                    const image_transport::Publisher& publisher);


        /** \brief Publishes stereo images with its keypoints drawn.
         *  \param l_img The left image (input). 
         *  \param r_img The right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param l_kps The left image keypoints (input).
         *  \param r_kps The left image keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishStereoMatches(const cv::Mat& l_img,
                                  const cv::Mat& r_img,
                                  const ros::Time& stamp,
                                  const std::vector<cv::DMatch>& matches,
                                  const std::vector<cv::KeyPoint>& l_kps,
                                  const std::vector<cv::KeyPoint>& r_kps,
                                  const image_transport::Publisher& publisher);


    protected:

        /** \brief Publishes images.
         *  \param img The image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishImage(const cv::Mat& img,
                          const ros::Time& stamp,
                          const image_transport::Publisher& publisher);


        /** \brief Writes text at the bottom of the image.
         *  \param in_img The image (input).
         *  \param s The text (input).
         *  \param txt_img The image with the txt at the bottom (output).
         */
        void writeTxtInImage(const cv::Mat& img,
                             const std::stringstream& s,
                             cv::Mat& txt_img);

};


#endif // PUBLISHER_HPP