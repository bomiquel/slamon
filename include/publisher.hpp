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


        /** \brief Draws and publishes vertical (successive) or horizontal (stereo)
         *  matches between two images.
         *  \param vertical If true, draws and publishes vertical matches 
         *  (case for the successive images). If false, draws and publishes horizontal
         *  matches (case for stereo pair) (input).
         *  \param c_img In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param q_img In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param f_c_kps Filtered and sorted current/left keypoints (input).
         *  \param f_q_kps Filtered and sorted previous/right keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishMatches(bool vertical, 
                            const cv::Mat& c_img,
                            const cv::Mat& q_img,
                            const ros::Time& stamp,
                            const std::vector<cv::KeyPoint>& f_c_kps,
                            const std::vector<cv::KeyPoint>& f_q_kps,
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


        /** \brief Draws vertical or horizontal matches between two images.
         *  \param vertical If true, the function concatenates the two images
         *  and paints the matchings vertically (case for the successive images). 
         *  In false, it does it horizontally (case for stereo pair) (input).
         *  \param c_img In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param q_img In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param color Color of the keypoints and matches (input).
         *  \param f_c_kps Filtered and sorted current/left keypoints (input).
         *  \param f_q_kps Filtered and sorted previous/right keypoints (input).
         *  \param out_img Concatened and drawn image (output). 
         */
        void drawMatches(bool vertical,
                         const cv::Mat& c_img,
                         const cv::Mat& q_img,
                         const cv::Scalar& color,
                         const std::vector<cv::KeyPoint> f_c_kps,
                         const std::vector<cv::KeyPoint> f_q_kps,
                         cv::Mat& out_img);
};


#endif // PUBLISHER_HPP