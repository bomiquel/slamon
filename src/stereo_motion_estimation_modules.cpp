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
                                                         std::vector<cv::KeyPoint>& f_l_kps,
                                                         std::vector<cv::KeyPoint>& f_r_kps)
{
    // Sanity check
    f_l_kps.clear();
    f_r_kps.clear();
    f_l_descs.release();
    f_r_descs.release();

    // Dummy variables
    std::vector<cv::Point3f> points_3d_0, points_3d_1, f_points_3d_0, f_points_3d_1;

    // Perform keypoint matching and Lowe's ratio
    MatchResult result;
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::KeyPoint> h_f_l_kps, h_f_r_kps;
    result = ratioMatching(ratio, l_descs, r_descs, l_kps, r_kps, points_3d_0, points_3d_1,
                           h_f_l_descs, h_f_r_descs, h_f_l_kps, h_f_r_kps, f_points_3d_0, f_points_3d_1);
    
    // Check match result
    if (!parseMatchResult(enable_debugging_, "[Stereo Matching:]", result))
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
        if (enable_debugging_)
        {
            const char* err_msg = e.what();
            std::cerr << "[Stereo Matching:] Problems in calculating the fundamental matrix in the outlier elimination step. Exception: " << err_msg << std::endl;
        }
        return false;
    }

    // Filter outliers using fundamental matrix
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_l_kps.push_back(h_f_l_kps[i]);
            f_r_kps.push_back(h_f_r_kps[i]);
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
                                                            std::vector<cv::KeyPoint>& f_c_l_kps,
                                                            std::vector<cv::KeyPoint>& f_c_r_kps,
                                                            std::vector<cv::KeyPoint>& f_q_l_kps,
                                                            std::vector<cv::KeyPoint>& f_q_r_kps,
                                                            std::vector<cv::Point3f>& f_c_camera_points,
                                                            std::vector<cv::Point3f>& f_q_camera_points)
{
    bool successful_matching = false;
    cv::Mat f_c_l_descs, f_c_r_descs, f_q_l_descs, f_q_r_descs;
    switch (match_type_)
    {
        case PLAIN_MATCHING:
        {
            MatchResult result = ratioMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_camera_points, q_camera_points,
                                               f_c_l_descs, f_q_l_descs, f_c_l_kps, f_q_l_kps, f_c_camera_points, f_q_camera_points);
            successful_matching = parseMatchResult(enable_debugging_, "[Plain matching:]", result);
            break;
        }
        case PLAIN_MATCHING_H:
        {
            MatchResult result = ratioHomographyMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_camera_points, q_camera_points,
                                                         f_c_l_descs, f_q_l_descs, f_c_l_kps, f_q_l_kps, f_c_camera_points, f_q_camera_points);
            successful_matching = parseMatchResult(enable_debugging_, "[Plain matching with homography:]", result);
            break;
        }
        case CIRCULAR_MATCHING:
        {
            MatchResult l_result, r_result;
            std::vector<cv::KeyPoint> h_f_c_l_kps, h_f_c_r_kps, h_f_q_l_kps, h_f_q_r_kps;
            std::vector<cv::Point3f> c_l_camera_points = c_camera_points, c_r_camera_points = c_camera_points;
            std::vector<cv::Point3f> q_l_camera_points = q_camera_points, q_r_camera_points = q_camera_points;
            std::vector<cv::Point3f> h_f_c_l_camera_points, h_f_c_r_camera_points, h_f_q_l_camera_points, h_f_q_r_camera_points;
            std::thread l_matcher([&] {l_result = ratioMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_l_camera_points, q_l_camera_points,
                                                                f_c_l_descs, f_q_l_descs, f_c_l_kps, f_q_l_kps, h_f_c_l_camera_points, h_f_q_l_camera_points);});
            std::thread r_matcher([&] {r_result = ratioMatching(ratio, c_r_descs, q_r_descs, c_r_kps, q_r_kps, c_r_camera_points, q_r_camera_points,
                                                                f_c_r_descs, f_q_r_descs, f_c_r_kps, f_q_r_kps, h_f_c_r_camera_points, h_f_q_r_camera_points);});
            l_matcher.join();
            r_matcher.join();

            bool l_success = parseMatchResult(enable_debugging_, "[Left matching in circular matching:]", l_result);
            bool r_success = parseMatchResult(enable_debugging_, "[Right matching in circular matching:]", r_result);

            if (!(l_success && r_success))
            {
                successful_matching = false;
                break;
            }
            else
            {
                successful_matching = true;
                break;
            }
        }
        case CIRCULAR_MATCHING_H:
        {
            MatchResult l_result, r_result;
            cv::Mat h_f_c_l_descs, h_f_c_r_descs, h_f_q_l_descs, h_f_q_r_descs;
            std::vector<cv::KeyPoint> h_f_c_l_kps, h_f_c_r_kps, h_f_q_l_kps, h_f_q_r_kps;
            std::vector<cv::Point3f> c_l_camera_points = c_camera_points, c_r_camera_points = c_camera_points;
            std::vector<cv::Point3f> q_l_camera_points = q_camera_points, q_r_camera_points = q_camera_points;
            std::vector<cv::Point3f> h_f_c_l_camera_points, h_f_c_r_camera_points, h_f_q_l_camera_points, h_f_q_r_camera_points;
            std::thread l_matcher([&] {l_result = ratioHomographyMatching(ratio, c_l_descs, q_l_descs, c_l_kps, q_l_kps, c_l_camera_points, q_l_camera_points,
                                                                          f_c_l_descs, f_q_l_descs, f_c_l_kps, f_q_l_kps, h_f_c_l_camera_points, h_f_q_l_camera_points);});
            std::thread r_matcher([&] {r_result = ratioHomographyMatching(ratio, c_r_descs, q_r_descs, c_r_kps, q_r_kps, c_r_camera_points, q_r_camera_points,
                                                                          f_c_r_descs, f_q_r_descs, f_c_r_kps, f_q_r_kps, h_f_c_r_camera_points, h_f_q_r_camera_points);});
            l_matcher.join();
            r_matcher.join();

            bool l_success = parseMatchResult(enable_debugging_, "[Left matching in circular matching with homography:]", l_result);
            bool r_success = parseMatchResult(enable_debugging_, "[Right matching in circular matching with homography:]", r_result);

            if (!(l_success && r_success))
            {
                successful_matching = false;
                break;
            }
            else
            {
                successful_matching = true;
                break;
            }
        }
        default:
        {
            std::cout << " Unforeseen type of matching!" << std::endl;
            break;
        }
    }
    return successful_matching;
}


StereoMotionEstimationModules::MatchResult StereoMotionEstimationModules::ratioMatching(double ratio,
                                                                                        const cv::Mat& c_descs, 
                                                                                        const cv::Mat& q_descs,
                                                                                        const std::vector<cv::KeyPoint>& c_kps,
                                                                                        const std::vector<cv::KeyPoint>& q_kps,
                                                                                        const std::vector<cv::Point3f>& c_camera_points,
                                                                                        const std::vector<cv::Point3f>& q_camera_points,
                                                                                        cv::Mat& f_c_descs,
                                                                                        cv::Mat& f_q_descs,
                                                                                        std::vector<cv::KeyPoint>& f_c_kps,
                                                                                        std::vector<cv::KeyPoint>& f_q_kps,
                                                                                        std::vector<cv::Point3f>& f_c_camera_points,
                                                                                        std::vector<cv::Point3f>& f_q_camera_points)
{
    // Sanity check
    f_c_kps.clear();
    f_q_kps.clear();
    f_c_descs.release();
    f_q_descs.release();
    f_c_camera_points.clear();
    f_q_camera_points.clear();
    if (c_descs.rows < 10 || q_descs.rows < 10) 
        return NOT_ENOUGH_KPS;

    bool fill_3d;
    if (c_camera_points.size() > 0)
        fill_3d = true;
    else
        fill_3d = false;

    // Visual keypoint matching
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch> > knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(c_descs, q_descs, knn_matches, knn, match_mask);

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
            f_c_kps.push_back(c_kps[idx_0]);
            f_q_kps.push_back(q_kps[idx_1]);
            f_c_descs.push_back(c_descs.row(idx_0));
            f_q_descs.push_back(q_descs.row(idx_1));
            
            if (fill_3d)
            {
                f_c_camera_points.push_back(c_camera_points[idx_0]);
                f_q_camera_points.push_back(q_camera_points[idx_0]);
            }
        }
    }
    
    // Sanity check
    if (f_c_kps.size() < 10)
        return NOT_ENOUGH_F_KPS;
    else
        return SUCCESSFUL_MATCHING;
}


StereoMotionEstimationModules::MatchResult StereoMotionEstimationModules::ratioHomographyMatching(double ratio,
                                                                                                  const cv::Mat& c_descs, 
                                                                                                  const cv::Mat& q_descs,
                                                                                                  const std::vector<cv::KeyPoint>& c_kps,
                                                                                                  const std::vector<cv::KeyPoint>& q_kps,
                                                                                                  const std::vector<cv::Point3f>& c_camera_points,
                                                                                                  const std::vector<cv::Point3f>& q_camera_points,
                                                                                                  cv::Mat& f_c_descs,
                                                                                                  cv::Mat& f_q_descs,
                                                                                                  std::vector<cv::KeyPoint>& f_c_kps,
                                                                                                  std::vector<cv::KeyPoint>& f_q_kps,
                                                                                                  std::vector<cv::Point3f>& f_c_camera_points,
                                                                                                  std::vector<cv::Point3f>& f_q_camera_points)
{
    MatchResult result;
    cv::Mat h_f_c_descs, h_f_q_descs;
    std::vector<cv::KeyPoint> h_f_c_kps, h_f_q_kps;
    std::vector<cv::Point3f> h_f_c_camera_points, h_f_q_camera_points;
    result = ratioMatching(ratio, c_descs, q_descs, c_kps, q_kps, c_camera_points, q_camera_points,
                           h_f_c_descs, h_f_q_descs, h_f_c_kps, h_f_q_kps, h_f_c_camera_points, h_f_q_camera_points);
    
    if (result != SUCCESSFUL_MATCHING)
        return result;

    // Compute homomatrix
    cv::Mat F, mask;
    std::vector<cv::Point2f> aux_h_f_c_kps, aux_h_f_q_kps;
    cv::KeyPoint::convert(h_f_c_kps, aux_h_f_c_kps, std::vector<int>());
    cv::KeyPoint::convert(h_f_q_kps, aux_h_f_q_kps, std::vector<int>());
    try
    {
        F = cv::findHomography(aux_h_f_c_kps, aux_h_f_q_kps, mask, cv::RANSAC, 3.0);
    }
    catch (cv::Exception& e)
    {
        return HOMOGRAPHY_PROBLEM;
    }

    // Filter outliers using homography
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_c_kps.push_back(h_f_c_kps[i]);
            f_q_kps.push_back(h_f_q_kps[i]);
            f_c_descs.push_back(h_f_c_descs.row(i));
            f_q_descs.push_back(h_f_q_descs.row(i));
            f_c_camera_points.push_back(h_f_c_camera_points[i]);
            f_q_camera_points.push_back(h_f_q_camera_points[i]);
        }
    }

    // Sanity check
    if (f_c_kps.size() < 10)
        return NOT_ENOUGH_H_F_KPS;
    else
        return SUCCESSFUL_MATCHING;
}


bool StereoMotionEstimationModules::parseMatchResult(bool enable_printing, 
                                                     const std::string& text,
                                                     const MatchResult& result)
{
    bool successful_matching; 
    switch (result)
    {
        case NOT_ENOUGH_KPS:
        {
            if (enable_printing)
                std::cout << text << " There are not enough keypoints to match and filter!" << std::endl;
            successful_matching = false;
            break;
        }
        case NOT_ENOUGH_F_KPS:
        {
            if (enable_printing)
                std::cout << text << " There are not enough keypoints after matching and filter with Lowe's Ratio!" << std::endl;
            successful_matching = false;
            break;
        }
        case HOMOGRAPHY_PROBLEM:
        {
            if (enable_printing)
                std::cout << text << " Problems in calculating the homography in the outlier elimination step!" << std::endl;
            successful_matching = false;
            break;
        }
        case NOT_ENOUGH_H_F_KPS:
        {
            if (enable_printing)
                std::cout << text << " There are not enough keypoints after homography filtering!" << std::endl;
            successful_matching = false;
            break;
        }
        case SUCCESSFUL_MATCHING:
        {
            if (enable_printing)
                std::cout << text << " Successful matching!" << std::endl;
            successful_matching = true;
            break;
        }
        default:
        {
            std::cout << text << " Unforeseen result of matching!" << std::endl;
            successful_matching = false;
            break;
        }
    }
    return successful_matching;
}