/**
 *  \file keypoint_detector.hpp
 *  \brief The KeypointDetector class provides tools for 2D keypoint detection and feature descriptor extraction.
 *
 *  @author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
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


/** \class KeypointDetector
 *  \brief Detects 2D visual keypoints and computes feature descriptors from single or stereo images.
 *  
 *  This class supports multiple feature detector/descriptor algorithms, optional image 
 *  contrast enhancement via CLAHE (Contrast Limited Adaptive Histogram Equalization), 
 *  and parallelized processing for stereo image pairs.
 */
class KeypointDetector
{

    public:


        /** \brief Default class constructor.
         */
        KeypointDetector() {};


        /** \brief Detects 2D keypoints and computes descriptors for a single image.
         *  \param[in] img Input image.
         *  \param[in] max_kpts Maximum number of keypoints to retain.
         *  \param[in] enable_enhacement If `true`, the input image is pre-processed using CLAHE.
         *  \param[in] detector_name Name of the feature detector/descriptor algorithm to use (e.g., `"ORB"`, `"SIFT"`).
         *  \param[out] descs Output matrix containing the computed keypoint descriptors.
         *  \param[out] kpts Output vector containing the detected 2D keypoints.
         */
        void detectKeypoints(const cv::Mat& img,
                             const int max_kpts,
                             const bool enable_enhacement,
                             const std::string& detector_name,
                             cv::Mat& descs,
                             std::vector<cv::KeyPoint>& kpts);


        /** \brief Detects 2D keypoints and computes descriptors for a stereo image pair in parallel.
         *  \param[in] l_img Left input stereo image.
         *  \param[in] r_img Right input stereo image.
         *  \param[in] max_kpts Maximum number of keypoints to retain per image.
         *  \param[in] enable_enhacement If `true`, the stereo images are pre-processed using CLAHE.
         *  \param[in] detector_name Name of the feature detector/descriptor algorithm to use (e.g., `"ORB"`, `"SIFT"`).
         *  \param[out] l_descs Output matrix containing the left image keypoint descriptors.
         *  \param[out] r_descs Output matrix containing the right image keypoint descriptors.
         *  \param[out] l_kpts Output vector containing the detected left image 2D keypoints.
         *  \param[out] r_kpts Output vector containing the detected right image 2D keypoints.
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