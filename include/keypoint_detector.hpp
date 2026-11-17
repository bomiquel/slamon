/**
 *  @file keypoint_detector.hpp
 *  @brief The KeypointDetector class provides the necessary tools to 
 *         detect keypoints and calculate descriptors.
 *  @package slamon
 *  @project SLAMON
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  @license BSD-4-Clause
 *    This file is part of the SLAMON project and is released under
 *    the BSD 4-Clause License. See the LICENSE file for details.
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
         *  \param[in] img The image.
         *  \param[in] max_kpts Maximum number of keypoints to be detected.
         *  \param[in] enable_enhacement If true, the image is enhanced using CLAHE.
         *  \param[in] detector_name String containing the name of the desired detector for
         *  detecting image keypoints.
         *  \param[out] descs The image keypoints descriptors.
         *  \param[out] kpts The image keypoints.
         */
        void detectKeypoints(const cv::Mat& img,
                             const int max_kpts,
                             const bool enable_enhacement,
                             const std::string& detector_name,
                             cv::Mat& descs,
                             std::vector<cv::KeyPoint>& kpts);


        /** \brief Parallelizes stereo image keypoint detection.
         *  \param[in] l_img The left image.
         *  \param[in] r_img The right image.
         *  \param[in] max_kpts Maximum number of keypoints to be detected.
         *  \param[in] enable_enhacement If true, the image is enhanced using CLAHE.
         *  \param[in] detector_name String containing the name of the desired detector for 
         *  detecting image keypoints.
         *  \param[out] l_descs The left image keypoints descriptors.
         *  \param[out] r_descs The right image keypoints descriptors.
         *  \param[out] l_kpts The left image keypoints.
         *  \param[out] r_kpts The right image keypoints.
         */
        void detectStereoKeypoints(const cv::Mat& l_img,
                                   const cv::Mat& r_img,
                                   const int max_kpts,
                                   const bool enable_enhacement,
                                   const std::string& detector_name,
                                   cv::Mat& l_descs,
                                   cv::Mat& r_descs,
                                   std::vector<cv::KeyPoint>& l_kpts,
                                   std::vector<cv::KeyPoint>& r_kpts); 
};


#endif // KEYPOINT_DETECTOR_HPP