/**
 *  @file
 *  @brief The Publisher class is responsive to publish the image visualizations for debugging purposes (presentation).
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter 2024
 */

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


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
         *  \param l_kpts The left image keypoints (input).
         *  \param r_kpts The left image keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishStereoKeypoints(const cv::Mat& l_img,
                                    const cv::Mat& r_img,
                                    const ros::Time& stamp,
                                    const std::vector<cv::Point2f>& l_kpts,
                                    const std::vector<cv::Point2f>& r_kpts,
                                    const image_transport::Publisher& publisher);


        /** \brief Draws and publishes vertical (successive) or horizontal (stereo)
         *  matches between two images.
         *  \param vertical If true, draws and publishes vertical matches 
         *  (case for the successive images). If false, draws and publishes horizontal
         *  matches (case for stereo pair) (input).
         *  \param img_0 In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param img_1 In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param f_kpts_0 Filtered and sorted current/left keypoints (input).
         *  \param f_kpts_1 Filtered and sorted previous/right keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishMatches(bool vertical, 
                            const cv::Mat& img_0,
                            const cv::Mat& img_1,
                            const ros::Time& stamp,
                            const std::vector<cv::Point2f>& f_kpts_0,
                            const std::vector<cv::Point2f>& f_kpts_1,
                            const image_transport::Publisher& publisher);


        /** \brief Draws and publishes circular matches.
         *  \param c_l_img Current left image (input).
         *  \param c_r_img Current right image (input).
         *  \param p_l_img Previous left image (input).
         *  \param p_r_img Previous right image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param f_c_l_kpts Filtered and sorted current left keypoints (input).
         *  \param f_c_r_kpts Filtered and sorted current right keypoints (input).
         *  \param f_p_l_kpts Filtered and sorted previous left keypoints (input).
         *  \param f_p_r_kpts Filtered and sorted previous right keypoints (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishCircularMatches(const cv::Mat& c_l_img,                                    
                                    const cv::Mat& c_r_img,
                                    const cv::Mat& p_l_img,
                                    const cv::Mat& p_r_img,
                                    const ros::Time& stamp,
                                    const std::vector<cv::Point2f>& f_c_l_kpts,
                                    const std::vector<cv::Point2f>& f_c_r_kpts,
                                    const std::vector<cv::Point2f>& f_p_l_kpts,
                                    const std::vector<cv::Point2f>& f_p_r_kpts,
                                    const image_transport::Publisher& publisher);


        /** \brief Publishes images.
         *  \param img The image (input).
         *  \param stamp The ROS timestamp (input).
         *  \param publisher The ROS publisher (input).
         */
        void publishImage(const cv::Mat& img,
                          const ros::Time& stamp,
                          const image_transport::Publisher& publisher);


        /** \brief Publishes current robot pose and velocity.
         *  \param c_stamp Current ROS timestamp (input).
         *  \param p_stamp Previous ROS timestamp (input).
         *  \param pose The current robot pose to be published (input).
         *  \param motion The robot movement between current and previous frames.
         *  Used to calculate robot velocity (input).
         *  \param publisher The ROS publisher (input).
         *  \param world_frame_id World coordinate axis id (input).
         *  \param robot_frame_id Robot coordinate axis id (input).
         */
        void publishPoseAndVelocity(const ros::Time& c_stamp,
                                    const ros::Time& p_stamp,
                                    const tf::Transform& pose,
                                    const tf::Transform& motion,
                                    const ros::Publisher& publisher,
                                    const std::string& world_frame_id,
                                    const std::string& robot_frame_id);


    private:


        /** \brief Draw points to image.
         *  \param img Original image (input).
         *  \param color The colour desired to be used to paint the points (input).
         *  \param pts The points (input).
         *  @return The image with the points drawn.
         */
        cv::Mat drawPoints(const cv::Mat& img,
                           const cv::Scalar& color,
                           const std::vector<cv::Point2f>& pts);


        /** \brief Writes text at the bottom of the image.
         *  \param img The image (input).
         *  \param s The text (input).
         *  @return An image with the txt at the bottom.
         */
        cv::Mat writeTxtInImage(const cv::Mat& img,
                                const std::stringstream& s);


        /** \brief Draws vertical or horizontal matches between two images.
         *  \param vertical If true, the function concatenates the two images
         *  and paints the matchings vertically (case for the successive images). 
         *  In false, it does it horizontally (case for stereo pair) (input).
         *  \param img_0 In the case of successive images, it is the current image.
         *  In the stereo case, it is the left image (input).
         *  \param img_1 In the case of successive images, it is the previous image.
         *  In the stereo case, it is the right image (input).
         *  \param color Color of the keypoints and matches (input).
         *  \param f_kpts_0 Filtered and sorted current/left keypoints (input).
         *  \param f_kpts_1 Filtered and sorted previous/right keypoints (input).
         *  @return a concatened and drawn image. 
         */
        cv::Mat drawMatches(const bool vertical,
                            const cv::Mat& img_0,
                            const cv::Mat& img_1,
                            const cv::Scalar& color,
                            const std::vector<cv::Point2f>& f_kpts_0,
                            const std::vector<cv::Point2f>& f_kpts_1);
};


#endif // PUBLISHER_HPP