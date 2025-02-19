#include "keypoint_detector.hpp"


void KeypointDetector::detectKeypoints(const cv::Mat& img,
                                       const int max_kpts,
                                       const bool enable_enhacement,
                                       const std::string& detector_name,
                                       cv::Mat& descs,
                                       std::vector<cv::KeyPoint>& kpts)
{
    // If necessary, converts the image to greyscale.
    cv::Mat g_img;
    if (img.type() == CV_8UC3)
    {
        cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);
    }
    else if (img.type() == CV_8UC1)
    {
        g_img = img;
    }
    else
    {
        std::cerr << "Invalid image type, only CV_8UC1 and CV_8UC3 are allowed. Exiting!" << std::endl;
        return;
    }

    // Enhances the image using CLAHE.
    cv::Mat proc_img;
    if (enable_enhacement)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2, cv::Size(8, 8));
        clahe->apply(g_img, proc_img);
        if (proc_img.rows == 0)
        {
            std::cerr << "Something went wrong with the image enhancement. Exiting!" << std::endl;
            return;
        }
    }
    else
    {
        proc_img = g_img;
    }

    // Selects detector.
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
    if (max_kpts > 0)
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