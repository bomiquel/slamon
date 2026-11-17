/**
 *  \file keypoint_detector.cpp
 *  \brief The KeypointDetector class provides tools for 2D keypoint detection and feature descriptor extraction.
 *
 *  @author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#include "keypoint_detector.hpp"


void KeypointDetector::detectKeypoints(const cv::Mat& img,
                                       const int max_kpts,
                                       const bool enable_enhacement,
                                       const std::string& detector_name,
                                       cv::Mat& descs,
                                       std::vector<cv::KeyPoint>& kpts)
{
    // First sanity check.
    if (img.empty())
    {
        std::cerr << "Image is empty!" << std::endl;
        return;
    }

    // Second sanity check.
    if (img.type() != CV_8UC3 && img.type() != CV_8UC1)
    {
        std::cerr << "Unsupported image type. Only CV_8UC1 and CV_8UC3 are allowed." << std::endl;
        return;
    }

    // Third sanity check.
    kpts.clear();
    descs.release();

    // Converts to grayscale if needed.
    cv::Mat g_img;
    if (img.type() == CV_8UC3)
        cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);
    else 
        g_img = img.clone();

    // Enhances the image using CLAHE.
    cv::Mat proc_img;
    if (enable_enhacement)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(g_img, proc_img);
        if (proc_img.empty())
        {
            std::cerr << "CLAHE enhancement failed." << std::endl;
            return;
        }
    }
    else
    {
        proc_img = g_img;
    }

    // Selects feature detector.
    cv::Ptr<cv::Feature2D> detector;
    if (detector_name == "sift")
        detector = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);
    else if (detector_name == "surf")
        detector = cv::xfeatures2d::SURF::create(100, 4, 3, true, false);
    else if (detector_name == "orb")
        detector = cv::ORB::create(0, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    else
        detector = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);

    // Detects keypoints and computes descriptors.
    detector->detect(proc_img, kpts, cv::noArray());
    if (max_kpts > 0 && kpts.size() > static_cast<size_t>(max_kpts))
        cv::KeyPointsFilter::retainBest(kpts, max_kpts);
    detector->compute(proc_img, kpts, descs);
}


void KeypointDetector::detectStereoKeypoints(const cv::Mat& l_img,
                                             const cv::Mat& r_img,
                                             const int max_kpts,
                                             const bool enable_enhacement,
                                             const std::string& detector_name,
                                             cv::Mat& l_descs,
                                             cv::Mat& r_descs,
                                             std::vector<cv::KeyPoint>& l_kpts,
                                             std::vector<cv::KeyPoint>& r_kpts)
{ 
    std::thread l_detection(&KeypointDetector::detectKeypoints, this, std::ref(l_img), 
                            max_kpts, enable_enhacement, std::ref(detector_name), 
                            std::ref(l_descs), std::ref(l_kpts));
    std::thread r_detection(&KeypointDetector::detectKeypoints, this, std::ref(r_img),
                            max_kpts, enable_enhacement, std::ref(detector_name), 
                            std::ref(r_descs), std::ref(r_kpts));
    l_detection.join();
    r_detection.join();
}