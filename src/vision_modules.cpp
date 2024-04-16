#include "vision_modules.hpp"

VisionModules::VisionModules(double ratio,
                             int min_inliers,
                             const std::string &matching_type)
{
    ratio_ = ratio;
    min_inliers_ = min_inliers;
    if (matching_type == "essential_matching")
        matching_type_ = ESSENTIAL_MATCHING;
    else if (matching_type == "homography_matching")
        matching_type_ = HOMOGRAPHY_MATCHING;
    else if (matching_type == "circular_ratio_matching")
        matching_type_ = CIRCULAR_RATIO_MATCHING;
    else if (matching_type == "circular_essential_matching")
        matching_type_ = CIRCULAR_ESSENTIAL_MATCHING;
    else if (matching_type == "circular_homography_matching")
        matching_type_ = CIRCULAR_HOMOGRAPHY_MATCHING;
    else
        matching_type_ = RATIO_MATCHING;
}


void VisionModules::detectKeypoints(const cv::Mat &img,
                                    cv::Mat &descs,
                                    std::vector<cv::KeyPoint> &kps)
{
    // Convert images to grayscale
    cv::Mat g_img;
    cv::cvtColor(img, g_img, cv::COLOR_RGB2GRAY);

    // Select detector and extract keypoints
    cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SIFT::create();
    detector->detectAndCompute(g_img, cv::noArray(), kps, descs);
}


void VisionModules::detectStereoKeypoints(const cv::Mat &l_img,
                                          const cv::Mat &r_img,
                                          cv::Mat &l_descs,
                                          cv::Mat &r_descs,
                                          std::vector<cv::KeyPoint> &l_kps,
                                          std::vector<cv::KeyPoint> &r_kps)
{
    std::thread l_detection(&VisionModules::detectKeypoints, this, std::ref(l_img), std::ref(l_descs), std::ref(l_kps));
    std::thread r_detection(&VisionModules::detectKeypoints, this, std::ref(r_img), std::ref(r_descs), std::ref(r_kps));
    l_detection.join();
    r_detection.join();
}


VisionModules::VisionResult VisionModules::matchStereoKeypoints(const cv::Mat &l_descs,
                                                                const cv::Mat &r_descs,
                                                                const std::vector<cv::KeyPoint> &l_kps,
                                                                const std::vector<cv::KeyPoint> &r_kps,
                                                                cv::Mat &f_l_descs,
                                                                cv::Mat &f_r_descs,
                                                                std::vector<cv::KeyPoint> &f_l_kps,
                                                                std::vector<cv::KeyPoint> &f_r_kps)
{
    // Sanity check
    f_l_kps.clear();
    f_r_kps.clear();
    f_l_descs.release();
    f_r_descs.release();

    // Dummy variables
    std::vector<cv::Point3f> points_3d_0, points_3d_1, f_points_3d_0, f_points_3d_1;

    // Perform keypoint matching and Lowe's ratio
    VisionResult match_result;
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::KeyPoint> h_f_l_kps, h_f_r_kps;
    match_result = ratioMatching(l_descs, r_descs, l_kps, r_kps, points_3d_0, points_3d_1,
                                 h_f_l_descs, h_f_r_descs, h_f_l_kps, h_f_r_kps, f_points_3d_0, f_points_3d_1);

    // Check match result
    if (match_result != SUCCESSFUL)
        return match_result;

    // Epipolar filtering
    for (uint i = 0; i < h_f_l_kps.size(); i++)
    {
        if (abs(h_f_l_kps[i].pt.y - h_f_r_kps[i].pt.y) <= 1.0)
        {
            f_l_kps.push_back(h_f_l_kps[i]);
            f_r_kps.push_back(h_f_r_kps[i]);
            f_l_descs.push_back(h_f_l_descs.row(i));
            f_r_descs.push_back(h_f_r_descs.row(i));
        }
    }

    if (f_l_kps.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_E_KPS;
    else
        return SUCCESSFUL;
}


VisionModules::VisionResult VisionModules::matchNonStereoKeypoints(const cv::Mat &c_l_descs,
                                                                   const cv::Mat &c_r_descs,
                                                                   const cv::Mat &p_l_descs,
                                                                   const cv::Mat &p_r_descs,
                                                                   const std::vector<cv::KeyPoint> &c_l_kps,
                                                                   const std::vector<cv::KeyPoint> &c_r_kps,
                                                                   const std::vector<cv::KeyPoint> &p_l_kps,
                                                                   const std::vector<cv::KeyPoint> &p_r_kps,
                                                                   const std::vector<cv::Point3f> &c_camera_points,
                                                                   const std::vector<cv::Point3f> &p_camera_points,
                                                                   std::vector<cv::KeyPoint> &f_c_l_kps,
                                                                   std::vector<cv::KeyPoint> &f_c_r_kps,
                                                                   std::vector<cv::KeyPoint> &f_p_l_kps,
                                                                   std::vector<cv::KeyPoint> &f_p_r_kps,
                                                                   std::vector<cv::Point3f> &f_c_camera_points,
                                                                   std::vector<cv::Point3f> &f_p_camera_points)
{
    VisionResult match_result, aux_result;
    cv::Mat f_c_l_descs, f_c_r_descs, f_p_l_descs, f_p_r_descs;
    if (matching_type_ == RATIO_MATCHING)
    {
        match_result = ratioMatching(c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_camera_points, p_camera_points,
                                     f_c_l_descs, f_p_l_descs, f_c_l_kps, f_p_l_kps, f_c_camera_points, f_p_camera_points);
    }
    else if (matching_type_ == ESSENTIAL_MATCHING)
    {
        match_result = geometricalMatching(false, c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_camera_points, p_camera_points,
                                           f_c_l_descs, f_p_l_descs, f_c_l_kps, f_p_l_kps, f_c_camera_points, f_p_camera_points);
    }
    else if (matching_type_ == HOMOGRAPHY_MATCHING)
    {
        match_result = geometricalMatching(true, c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_camera_points, p_camera_points,
                                           f_c_l_descs, f_p_l_descs, f_c_l_kps, f_p_l_kps, f_c_camera_points, f_p_camera_points);
    }
    else if (matching_type_ == CIRCULAR_RATIO_MATCHING ||
             matching_type_ == CIRCULAR_ESSENTIAL_MATCHING ||
             matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
    {
        std::vector<cv::KeyPoint> h_f_c_l_kps, h_f_c_r_kps, h_f_p_l_kps, h_f_p_r_kps;
        std::vector<cv::Point3f> c_l_camera_points = c_camera_points, c_r_camera_points = c_camera_points;
        std::vector<cv::Point3f> p_l_camera_points = p_camera_points, p_r_camera_points = p_camera_points;
        std::vector<cv::Point3f> h_f_c_l_camera_points, h_f_c_r_camera_points, h_f_p_l_camera_points, h_f_p_r_camera_points;
        if (matching_type_ == CIRCULAR_RATIO_MATCHING)
        {
            std::thread l_matcher([&] { match_result = ratioMatching(c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_l_camera_points, p_l_camera_points,
                                                                     f_c_l_descs, f_p_l_descs, h_f_c_l_kps, h_f_p_l_kps, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = ratioMatching(c_r_descs, p_r_descs, c_r_kps, p_r_kps, c_r_camera_points, p_r_camera_points,
                                                                   f_c_r_descs, f_p_r_descs, h_f_c_r_kps, h_f_p_r_kps, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }
        else if (matching_type_ == CIRCULAR_ESSENTIAL_MATCHING)
        {
            std::thread l_matcher([&] { match_result = geometricalMatching(false, c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_l_camera_points, p_l_camera_points,
                                                                           f_c_l_descs, f_p_l_descs, h_f_c_l_kps, h_f_p_l_kps, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = geometricalMatching(false, c_r_descs, p_r_descs, c_r_kps, p_r_kps, c_r_camera_points, p_r_camera_points,
                                                                         f_c_r_descs, f_p_r_descs, h_f_c_r_kps, h_f_p_r_kps, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }
        else
        {
            std::thread l_matcher([&] { match_result = geometricalMatching(true, c_l_descs, p_l_descs, c_l_kps, p_l_kps, c_l_camera_points, p_l_camera_points,
                                                                           f_c_l_descs, f_p_l_descs, h_f_c_l_kps, h_f_p_l_kps, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = geometricalMatching(true, c_r_descs, p_r_descs, c_r_kps, p_r_kps, c_r_camera_points, p_r_camera_points,
                                                                         f_c_r_descs, f_p_r_descs, h_f_c_r_kps, h_f_p_r_kps, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }

        if (match_result == SUCCESSFUL && aux_result == SUCCESSFUL)
        {
            if (h_f_c_r_kps.size() < h_f_c_l_kps.size())
            {
                match_result = assembleCircularMatchings(c_r_kps, c_l_kps, p_r_kps, p_l_kps,
                                                         h_f_c_r_kps, h_f_c_l_kps, h_f_p_r_kps, h_f_p_l_kps,
                                                         h_f_c_r_camera_points, h_f_p_r_camera_points,
                                                         f_c_r_kps, f_c_l_kps, f_p_r_kps, f_p_l_kps,
                                                         f_c_camera_points, f_p_camera_points);
            }
            else
            {
                match_result = assembleCircularMatchings(c_l_kps, c_r_kps, p_l_kps, p_r_kps,
                                                         h_f_c_l_kps, h_f_c_r_kps, h_f_p_l_kps, h_f_p_r_kps,
                                                         h_f_c_l_camera_points, h_f_p_l_camera_points,
                                                         f_c_l_kps, f_c_r_kps, f_p_l_kps, f_p_r_kps,
                                                         f_c_camera_points, f_p_camera_points);
            }
        }
        else if (match_result != aux_result)
        {
            match_result = DIFFERENT_PROBLEMS;
        }
    }
    else
    {
        std::cerr << "Unforeseen type of matching!" << std::endl;
    }

    return match_result;
}


VisionModules::VisionResult VisionModules::estimateMotion(const std::vector<cv::KeyPoint>& f_c_l_kps,
                                                          const std::vector<cv::KeyPoint>& f_c_r_kps,
                                                          const std::vector<cv::Point3f>& f_p_camera_points,
                                                          cv::Mat& r_vec,
                                                          cv::Mat& t_vec)
{
    std::vector<int> inliers;
    std::vector<cv::Point2f> f_c_l_pts;
    cv::KeyPoint::convert(f_c_l_kps, f_c_l_pts);
    cv::solvePnPRansac(f_p_camera_points, f_c_l_pts, camera_matrix_, cv::Mat(),
                       r_vec, t_vec, false, 1000, 2.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    std::cout << "Inliers: " << inliers.size() << std::endl;    

    if (inliers.size() > (uint)min_inliers_)
        return SUCCESSFUL;
    else
        return NOT_ENOUGH_INLIERS;   
}


std::vector<std::vector<cv::DMatch>> VisionModules::matching(const cv::Mat &c_descs,
                                                             const cv::Mat &p_descs)
{
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(c_descs, p_descs, knn_matches, knn, match_mask);
    return knn_matches;
}


VisionModules::VisionResult VisionModules::ratioMatching(const cv::Mat &c_descs,
                                                         const cv::Mat &p_descs,
                                                         const std::vector<cv::KeyPoint> &c_kps,
                                                         const std::vector<cv::KeyPoint> &p_kps,
                                                         const std::vector<cv::Point3f> &c_camera_points,
                                                         const std::vector<cv::Point3f> &p_camera_points,
                                                         cv::Mat &f_c_descs,
                                                         cv::Mat &f_p_descs,
                                                         std::vector<cv::KeyPoint> &f_c_kps,
                                                         std::vector<cv::KeyPoint> &f_p_kps,
                                                         std::vector<cv::Point3f> &f_c_camera_points,
                                                         std::vector<cv::Point3f> &f_p_camera_points)
{
    // Sanity check
    f_c_kps.clear();
    f_p_kps.clear();
    f_c_descs.release();
    f_p_descs.release();
    f_c_camera_points.clear();
    f_p_camera_points.clear();
    if (c_descs.rows < (int)(1.5 * min_inliers_) || p_descs.rows < (int)(1.5 * min_inliers_))
        return NOT_ENOUGH_KPS;

    // During feature matching of the same stereo pair there are no 3D points.
    bool fill_3d;
    if (c_camera_points.size() > 0)
        fill_3d = true;
    else
        fill_3d = false;

    // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> knn_matches = matching(c_descs, p_descs);

    // Lowe's ratio
    int c_idx, p_idx;
    for (uint i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i].size() < 2)
            continue;

        if (knn_matches[i][0].distance <= knn_matches[i][1].distance * ratio_)
        {
            c_idx = knn_matches[i][0].queryIdx;
            p_idx = knn_matches[i][0].trainIdx;
            f_c_kps.push_back(c_kps[c_idx]);
            f_p_kps.push_back(p_kps[p_idx]);
            f_c_descs.push_back(c_descs.row(c_idx));
            f_p_descs.push_back(p_descs.row(p_idx));

            // During feature matching of the same stereo pair there are no 3D points.
            if (fill_3d)
            {
                f_c_camera_points.push_back(c_camera_points[c_idx]);
                f_p_camera_points.push_back(p_camera_points[p_idx]);
            }
        }
    }

    // Sanity check
    if (f_c_kps.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_F_KPS;
    else
        return SUCCESSFUL;
}


VisionModules::VisionResult VisionModules::geometricalMatching(bool coplanar_points,
                                                               const cv::Mat &c_descs,
                                                               const cv::Mat &p_descs,
                                                               const std::vector<cv::KeyPoint> &c_kps,
                                                               const std::vector<cv::KeyPoint> &p_kps,
                                                               const std::vector<cv::Point3f> &c_camera_points,
                                                               const std::vector<cv::Point3f> &p_camera_points,
                                                               cv::Mat &f_c_descs,
                                                               cv::Mat &f_p_descs,
                                                               std::vector<cv::KeyPoint> &f_c_kps,
                                                               std::vector<cv::KeyPoint> &f_p_kps,
                                                               std::vector<cv::Point3f> &f_c_camera_points,
                                                               std::vector<cv::Point3f> &f_p_camera_points)
{
    // Sanity check
    f_c_kps.clear();
    f_p_kps.clear();
    f_c_descs.release();
    f_p_descs.release();
    f_c_camera_points.clear();
    f_p_camera_points.clear();
    if (c_descs.rows < (int)(1.5 * min_inliers_) || p_descs.rows < (int)(1.5 * min_inliers_))
        return NOT_ENOUGH_KPS;

    // // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> matches = matching(c_descs, p_descs);

    int c_idx, p_idx;
    cv::Mat m_c_descs, m_p_descs;
    std::vector<cv::KeyPoint> m_c_kps, m_p_kps;
    std::vector<cv::Point2f> m_c_pts, m_p_pts;
    std::vector<cv::Point3f> m_c_camera_points, m_p_camera_points;
    for (uint i = 0; i < matches.size(); i++)
    {
        c_idx = matches[i][0].queryIdx;
        p_idx = matches[i][0].trainIdx;
        m_c_kps.push_back(c_kps[c_idx]);
        m_p_kps.push_back(p_kps[p_idx]);
        m_c_pts.push_back(c_kps[c_idx].pt);
        m_p_pts.push_back(p_kps[p_idx].pt);
        m_c_descs.push_back(c_descs.row(c_idx));
        m_p_descs.push_back(p_descs.row(p_idx));
        m_c_camera_points.push_back(c_camera_points[c_idx]);
        m_p_camera_points.push_back(p_camera_points[p_idx]);
    }

    // Compute homography or essential matrix
    cv::Mat M, mask;
    try
    {
        if (coplanar_points)
            M = cv::findHomography(m_c_pts, m_p_pts, mask, cv::RANSAC, 1.0);
        else
            M = cv::findEssentialMat(m_c_pts, m_p_pts, camera_matrix_, cv::RANSAC, 0.999, 1.0, mask);
    }
    catch (cv::Exception &e)
    {
        return GEOMETRIC_PROBLEM;
    }

    // Filter outliers using homography
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_c_kps.push_back(m_c_kps[i]);
            f_p_kps.push_back(m_p_kps[i]);
            f_c_descs.push_back(m_c_descs.row(i));
            f_p_descs.push_back(m_p_descs.row(i));
            f_c_camera_points.push_back(m_c_camera_points[i]);
            f_p_camera_points.push_back(m_p_camera_points[i]);
        }
    }

    // Sanity check
    if (f_c_kps.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_F_KPS;
    else
        return SUCCESSFUL;
}


VisionModules::VisionResult VisionModules::assembleCircularMatchings(const std::vector<cv::KeyPoint> &c_0_stereo_kps,
                                                                     const std::vector<cv::KeyPoint> &c_1_stereo_kps,
                                                                     const std::vector<cv::KeyPoint> &p_0_stereo_kps,
                                                                     const std::vector<cv::KeyPoint> &p_1_stereo_kps,
                                                                     const std::vector<cv::KeyPoint> &c_0_successive_kps,
                                                                     const std::vector<cv::KeyPoint> &c_1_successive_kps,
                                                                     const std::vector<cv::KeyPoint> &p_0_successive_kps,
                                                                     const std::vector<cv::KeyPoint> &p_1_successive_kps,
                                                                     const std::vector<cv::Point3f>& c_successive_camera_points,
                                                                     const std::vector<cv::Point3f>& p_successive_camera_points,
                                                                     std::vector<cv::KeyPoint> &f_c_0_kps,
                                                                     std::vector<cv::KeyPoint> &f_c_1_kps,
                                                                     std::vector<cv::KeyPoint> &f_p_0_kps,
                                                                     std::vector<cv::KeyPoint> &f_p_1_kps,
                                                                     std::vector<cv::Point3f>& f_c_camera_points,
                                                                     std::vector<cv::Point3f>& f_p_camera_points)
{
    uint j;
    uint k;
    uint l;
    for (uint i = 0; i < p_0_successive_kps.size(); i++)
    {
        for (j = 0; j < p_0_stereo_kps.size(); j++)
        {
            if (p_0_successive_kps[i].pt == p_0_stereo_kps[j].pt)
            {
                for (k = 0; k < p_1_successive_kps.size(); k++)
                {
                    if (p_1_stereo_kps[j].pt == p_1_successive_kps[k].pt)
                    {
                        for (l = 0; l < c_1_stereo_kps.size(); l++)
                        {
                            if (c_1_successive_kps[k].pt == c_1_stereo_kps[l].pt)
                            {
                                if (c_0_stereo_kps[l].pt == c_0_successive_kps[i].pt)
                                {
                                    f_c_0_kps.push_back(c_0_successive_kps[i]);
                                    f_c_1_kps.push_back(c_1_successive_kps[k]);
                                    f_p_0_kps.push_back(p_0_successive_kps[i]);
                                    f_p_1_kps.push_back(p_1_successive_kps[k]);
                                    f_c_camera_points.push_back(c_successive_camera_points[i]);
                                    f_p_camera_points.push_back(p_successive_camera_points[i]);
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
    }

    // Sanity check
    if (f_c_0_kps.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_CIRCLES;
    else
        return SUCCESSFUL;
}