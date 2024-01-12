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


bool StereoMotionEstimationModules::matchStereoKeypoints(double ratio, 
                                                         const cv::Mat& l_descs,
                                                         const cv::Mat& r_descs,
                                                         const std::vector<cv::KeyPoint>& l_kps,
                                                         const std::vector<cv::KeyPoint>& r_kps,
                                                         cv::Mat& f_l_descs,
                                                         cv::Mat& f_r_descs,
                                                         std::vector<cv::DMatch>& f_matches,
                                                         std::vector<cv::KeyPoint>& f_l_kps,
                                                         std::vector<cv::KeyPoint>& f_r_kps)
{
    // Sanity check
    f_l_kps.clear();
    f_r_kps.clear();
    f_matches.clear();
    f_l_descs.release();
    f_r_descs.release();

    // Dummy variables
    std::vector<cv::Point3f> points_3d_0, points_3d_1, f_points_3d_0, f_points_3d_1;

    // Perform keypoint matching and Lowe's ratio
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::DMatch> h_f_matches;
    std::vector<cv::KeyPoint> h_f_l_kps, h_f_r_kps;
    if (!ratioMatching(ratio, l_descs, r_descs, l_kps, r_kps, points_3d_0, points_3d_1,
                       h_f_l_descs, h_f_r_descs, h_f_matches, h_f_l_kps, h_f_r_kps, f_points_3d_0, f_points_3d_1))
        return false;

    // Compute fundamental matrix
    cv::Mat F, mask;
    std::vector<cv::Point2f> aux_l_kps, aux_r_kps;
    cv::KeyPoint::convert(h_f_l_kps, aux_l_kps, std::vector<int>());
    cv::KeyPoint::convert(h_f_r_kps, aux_r_kps, std::vector<int>());
    try
    {
        F = cv::findFundamentalMat(aux_l_kps, aux_r_kps, mask, cv::RANSAC, 1.0);
    }
    catch(cv::Exception& e)
    {
        const char* err_msg = e.what();
        std::cerr << "[SLAMON:] Problems in calculating the fundamental matrix in the outlier elimination step. Exception: " << err_msg << std::endl;
        return false;
    }

    // Filter outliers using fundamental matrix
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_l_kps.push_back(h_f_l_kps[i]);
            f_r_kps.push_back(h_f_r_kps[i]);
            f_matches.push_back(h_f_matches[i]);
            f_l_descs.push_back(h_f_l_descs.row(i));
            f_r_descs.push_back(h_f_r_descs.row(i));
        }
    }

    return true;
}


bool StereoMotionEstimationModules::matchNonStereoKeypoints(double ratio,
                                                            const cv::Mat& c_l_descs,
                                                            const cv::Mat& c_r_descs,
                                                            const cv::Mat& q_l_descs,
                                                            const cv::Mat& q_r_descs,
                                                            const std::vector<cv::KeyPoint>& c_l_kps,
                                                            const std::vector<cv::KeyPoint>& c_r_kps,
                                                            const std::vector<cv::KeyPoint>& q_l_kps,
                                                            const std::vector<cv::KeyPoint>& q_r_kps,
                                                            const std::vector<cv::Point3f>& c_camera_points,
                                                            const std::vector<cv::Point3f>& q_camera_points,
                                                            cv::Mat& f_c_l_descs,
                                                            cv::Mat& f_c_r_descs,
                                                            cv::Mat& f_q_l_descs,
                                                            cv::Mat& f_q_r_descs,
                                                            std::vector<cv::DMatch>& f_l_matches,
                                                            std::vector<cv::KeyPoint>& f_c_l_kps,
                                                            std::vector<cv::KeyPoint>& f_c_r_kps,
                                                            std::vector<cv::KeyPoint>& f_q_l_kps,
                                                            std::vector<cv::KeyPoint>& f_q_r_kps,
                                                            std::vector<cv::Point3f>& f_c_camera_points,
                                                            std::vector<cv::Point3f>& f_q_camera_points)
{
    // Create all the necessary variables for all possible cases.
    // cv::Mat h_f_l_descs, h_f_r_descs;

    switch (match_type_)
    {
        case PLAIN_MATCHING:
            std::cout << "Plain matching" << std::endl;
            if (!ratioMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_camera_points, q_camera_points,
                               f_c_l_descs, f_q_l_descs, f_l_matches, f_c_l_kps, f_q_l_kps, f_c_camera_points, f_q_camera_points))
                return false;
            break;

        case PLAIN_MATCHING_H:
            std::cout << "Plain matching with homography" << std::endl;
            return false;
            break;

        case CIRCULAR_MATCHING:
            std::cout << "Circular matching" << std::endl;
            return false;
            break;

        case CIRCULAR_MATCHING_H:
            std::cout << "Quad matching with homography" << std::endl;
            return false;
            break;

        default:
            std::cout << "Default" << std::endl;
            if (!ratioMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_camera_points, q_camera_points,
                               f_c_l_descs, f_q_l_descs, f_l_matches, f_c_l_kps, f_q_l_kps, f_c_camera_points, f_q_camera_points))
                return false;
            break;
    }

    return true;
}


bool StereoMotionEstimationModules::ratioMatching(double ratio,
                                                  const cv::Mat& descs_0, 
                                                  const cv::Mat& descs_1,
                                                  const std::vector<cv::KeyPoint>& kps_0,
                                                  const std::vector<cv::KeyPoint>& kps_1,
                                                  const std::vector<cv::Point3f>& points_3d_0,
                                                  const std::vector<cv::Point3f>& points_3d_1,
                                                  cv::Mat& f_descs_0,
                                                  cv::Mat& f_descs_1,
                                                  std::vector<cv::DMatch>& f_matches,
                                                  std::vector<cv::KeyPoint>& f_kps_0,
                                                  std::vector<cv::KeyPoint>& f_kps_1,
                                                  std::vector<cv::Point3f>& f_points_3d_0,
                                                  std::vector<cv::Point3f>& f_points_3d_1)
{
    // Sanity check
    f_kps_0.clear();
    f_kps_1.clear();
    f_matches.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_points_3d_0.clear();
    f_points_3d_1.clear();
    if (descs_0.rows < 10 || descs_1.rows < 10) 
    {
        std::cerr << "[SLAMON:] There are not enough keypoints to match and filter." << std::endl;
        return false;
    }

    bool fill_3d;
    if (points_3d_0.size() > 0)
        fill_3d = true;
    else
        fill_3d = false;

    // Visual keypoint matching
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch> > knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(descs_0, descs_1, knn_matches, knn, match_mask);

    // Lowe's ratio
    int idx_0, idx_1;
    for (uint i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i].size() < 2)
            continue;

        if (knn_matches[i][0].distance <= knn_matches[i][1].distance * ratio)
        {
            idx_0 = knn_matches[i][0].queryIdx;
            idx_1 = knn_matches[i][0].trainIdx;
            f_matches.push_back(knn_matches[i][0]);
            f_kps_0.push_back(kps_0[idx_0]);
            f_kps_1.push_back(kps_1[idx_1]);
            f_descs_0.push_back(descs_0.row(idx_0));
            f_descs_1.push_back(descs_1.row(idx_1));
            
            if (fill_3d)
            {
                f_points_3d_0.push_back(points_3d_0[idx_0]);
                f_points_3d_1.push_back(points_3d_1[idx_0]);
            }
        }
    }
    
    // Sanity check
    if (f_kps_0.size() < 10)
    {
        std::cerr << "[SLAMON:] There are not enough keypoints after matching and filter keypoints." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}