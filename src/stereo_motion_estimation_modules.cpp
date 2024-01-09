#include "stereo_motion_estimation_modules.hpp"


void StereoMotionEstimationModules::detectKeypoints(const cv::Mat& img,
                                                    cv::Mat& descs,
                                                    std::vector<cv::KeyPoint>& kps)
{
    // Convert images to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);

    // Select detector and extract keypoints
    cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SIFT::create();
    detector->detectAndCompute(img_gray, cv::noArray(), kps, descs);
}


void StereoMotionEstimationModules::detectStereoKeypoints(const cv::Mat& l_img,
                                                          const cv::Mat& r_img,
                                                          cv::Mat& l_descs,
                                                          cv::Mat& r_descs,
                                                          std::vector<cv::KeyPoint>& l_kps,
                                                          std::vector<cv::KeyPoint>& r_kps)
{
    std::thread l_detection(&StereoMotionEstimationModules::detectKeypoints, this, std::ref(l_img), std::ref(l_descs), std::ref(l_kps));
    std::thread r_detection(&StereoMotionEstimationModules::detectKeypoints, this, std::ref(r_img), std::ref(r_descs), std::ref(r_kps));
    l_detection.join();
    r_detection.join();
}