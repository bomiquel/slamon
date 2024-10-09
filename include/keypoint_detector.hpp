/**
 *  @brief The KeypointDetector class provides needed to detect keypoints and compute descriptors.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2024
 */

#ifndef KEYPOINT_DETECTOR_HPP
#define KEYPOINT_DETECTOR_HPP


#include <vector>
#include <string>
#include <thread>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class KeypointDetector
{

    public:


        /** \brief Empty class constructor.
         * 
         */
        KeypointDetector() {};


        /** \brief Detects keypoints of an image.
         *  \param img The image (input).
         *  \param enable_enhacement If true, the image is enhanced using CLAHE (input).
         *  \param detector_name String containing the name of the desired detector for
         *  detecting image keypoints (input).
         *  \param descs The image keypoints descriptors (output).
         *  \param kpts The image keypoints (output).
         */
        void detectKeypoints(const cv::Mat& img,
                             const bool enable_enhacement,
                             const std::string& detector_name,
                             cv::Mat& descs,
                             std::vector<cv::KeyPoint>& kpts);


        /** \brief Parallelizes stereo image keypoint detection.
         *  \param l_img The left image (input).
         *  \param r_img The right image (input).
         *  \param enable_enhacement If true, the image is enhanced 
         *  using CLAHE (input).
         *  \param detector_name String containing the name of the 
         *  desired detector for detecting image keypoints (input).
         *  \param l_descs The left image keypoints descriptors (output).
         *  \param r_descs The right image keypoints descriptors (output).
         *  \param l_kpts The left image keypoints (output).
         *  \param r_kpts The right image keypoints (output).
         */
        void detectStereoKeypoints(const cv::Mat& l_img,
                                   const cv::Mat& r_img,
                                   const bool enable_enhacement,
                                   const std::string& detector_name,
                                   cv::Mat& l_descs,
                                   cv::Mat& r_descs,
                                   std::vector<cv::KeyPoint>& l_kpts,
                                   std::vector<cv::KeyPoint>& r_kpts); 
};


#endif // KEYPOINT_DETECTOR_HPP