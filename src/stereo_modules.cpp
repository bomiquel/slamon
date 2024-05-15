#include "stereo_modules.hpp"


StereoModules::StereoModules(const double ratio,
                             const int min_inliers,
                             std::string &matching_type)
{
    ratio_ = ratio;
    min_inliers_ = min_inliers;
    matching_type_ = selectMatchingType(matching_type);
}


StereoModules::StereoModules(const double ratio,
                             const int min_inliers,
                             const cv::Mat& l_proj_mat,
                             const cv::Mat& r_proj_mat,
                             std::string& matching_type)
{
    ratio_ = ratio;
    l_proj_mat_ = l_proj_mat;
    r_proj_mat_ = r_proj_mat;
    min_inliers_ = min_inliers;
    matching_type_ = selectMatchingType(matching_type);
}


StereoModules::StereoModules(const double b,
                             const double f_x,
                             const double f_y,
                             const double c_x,
                             const double c_y,
                             const double ratio,
                             const int min_inliers,
                             std::string& matching_type)
{
    ratio_ = ratio;
    min_inliers_ = min_inliers;
    matching_type_ = selectMatchingType(matching_type);
    l_proj_mat_ = computeProjectionMatrix(0.0, f_x, f_x, c_x, c_y);
    r_proj_mat_ = computeProjectionMatrix(b, f_x, f_x, c_x, c_y);
}


void StereoModules::detectKeypoints(const cv::Mat &img,
                                    const std::string& detector_name,
                                    cv::Mat &descs,
                                    std::vector<cv::Point2f> &kpts)
{
    // Convert images to grayscale
    cv::Mat g_img;
    cv::cvtColor(img, g_img, cv::COLOR_RGB2GRAY);

    // Select detector and extract keypoints
    std::vector<cv::KeyPoint> kps;
    cv::Ptr<cv::Feature2D> detector;
    if (detector_name == "sift")
    {
        detector = cv::xfeatures2d::SIFT::create();
        detector->detectAndCompute(g_img, cv::noArray(), kps, descs);
    }
    else if (detector_name == "surf")
    {
        detector = cv::xfeatures2d::SURF::create();
        detector->detectAndCompute(g_img, cv::noArray(), kps, descs);
    }
    else if (detector_name == "orb")
    {  
        detector = cv::ORB::create();
        detector->detectAndCompute(g_img, cv::noArray(), kps, descs);
    }
    else
    {
        detector = cv::xfeatures2d::SIFT::create();
        detector->detectAndCompute(g_img, cv::noArray(), kps, descs);
    }

    cv::KeyPoint::convert(kps, kpts);
}


void StereoModules::detectStereoKeypoints(const cv::Mat &l_img,
                                          const cv::Mat &r_img,
                                          const std::string& detector_name,
                                          cv::Mat &l_descs,
                                          cv::Mat &r_descs,
                                          std::vector<cv::Point2f> &l_kpts,
                                          std::vector<cv::Point2f> &r_kpts)
{
    std::thread l_detection(&StereoModules::detectKeypoints, this, 
                            std::ref(l_img), std::ref(detector_name), 
                            std::ref(l_descs), std::ref(l_kpts));
    std::thread r_detection(&StereoModules::detectKeypoints, this, 
                            std::ref(r_img), std::ref(detector_name), 
                            std::ref(r_descs), std::ref(r_kpts));
    l_detection.join();
    r_detection.join();
}


StereoModules::StereoResult StereoModules::matchStereoKeypoints(const cv::Mat &l_descs,
                                                                const cv::Mat &r_descs,
                                                                const std::vector<cv::Point2f> &l_kpts,
                                                                const std::vector<cv::Point2f> &r_kpts,
                                                                cv::Mat &f_l_descs,
                                                                cv::Mat &f_r_descs,
                                                                std::vector<cv::Point2f> &f_l_kpts,
                                                                std::vector<cv::Point2f> &f_r_kpts)
{
    // Sanity check
    f_l_kpts.clear();
    f_r_kpts.clear();
    f_l_descs.release();
    f_r_descs.release();

    // Dummy variables
    std::vector<cv::Point3f> points_3d_0, points_3d_1, f_points_3d_0, f_points_3d_1;

    // Perform keypoint matching and Lowe's ratio
    StereoResult match_result;
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::Point2f> h_f_l_kpts, h_f_r_kpts;
    match_result = ratioMatching(l_descs, r_descs, l_kpts, r_kpts, points_3d_0, points_3d_1,
                                 h_f_l_descs, h_f_r_descs, h_f_l_kpts, h_f_r_kpts, f_points_3d_0, f_points_3d_1);

    // Check match result
    if (match_result != SUCCESSFUL)
        return match_result;

    // Epipolar filtering
    for (uint i = 0; i < h_f_l_kpts.size(); i++)
    {
        if (abs(h_f_l_kpts[i].y - h_f_r_kpts[i].y) <= 1.0)
        {
            f_l_kpts.push_back(h_f_l_kpts[i]);
            f_r_kpts.push_back(h_f_r_kpts[i]);
            f_l_descs.push_back(h_f_l_descs.row(i));
            f_r_descs.push_back(h_f_r_descs.row(i));
        }
    }

    if (f_l_kpts.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_E_KPTS;
    else
        return SUCCESSFUL;
}


StereoModules::StereoResult StereoModules::compute3DPoints(const cv::Mat& l_descs,
                                                           const cv::Mat& r_descs,
                                                           const std::vector<cv::Point2f>& l_kpts,
                                                           const std::vector<cv::Point2f>& r_kpts,
                                                           cv::Mat& f_l_descs,
                                                           cv::Mat& f_r_descs,
                                                           std::vector<cv::Point2f>& f_l_kpts,
                                                           std::vector<cv::Point2f>& f_r_kpts,
                                                           std::vector<cv::Point3f>& f_obj_pts)
{
    // Triangulates keypoints.
    cv::Mat pts_4d, obj_pts;
    cv::triangulatePoints(l_proj_mat_, r_proj_mat_, l_kpts, r_kpts, pts_4d);
    cv::convertPointsFromHomogeneous(pts_4d.t(), obj_pts);
    obj_pts = obj_pts.reshape(1, obj_pts.rows).clone();

    // Computes baseline.
    double f_u = l_proj_mat_.at<double>(0, 0);
    double t_x = r_proj_mat_.at<double>(0, 3);
    double b = (t_x / -f_u);

    // Filters "far" keypoints.
    cv::Point3f pt;
    double z_limit = 40 * b;
    for (int i = 0; i < obj_pts.rows; i++)
    {
        pt.x = obj_pts.at<float>(i, 0);
        pt.y = obj_pts.at<float>(i, 1);
        pt.z = obj_pts.at<float>(i, 2);
        if (isfinite(pt.x) &&
            isfinite(pt.y) &&
            isfinite(pt.z) &&
            pt.z > 0.0 &&
            pt.z <= z_limit)
        {
            f_obj_pts.push_back(pt);
            f_l_kpts.push_back(l_kpts[i]);
            f_r_kpts.push_back(r_kpts[i]);
            f_l_descs.push_back(l_descs.row(i));
            f_r_descs.push_back(r_descs.row(i));
        }
    }
    
    if (f_obj_pts.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_3D_PTS;
    else
        return SUCCESSFUL;
}


StereoModules::StereoResult StereoModules::matchNonStereoKeypoints(const cv::Mat &c_l_descs,
                                                                   const cv::Mat &c_r_descs,
                                                                   const cv::Mat &p_l_descs,
                                                                   const cv::Mat &p_r_descs,
                                                                   const std::vector<cv::Point2f> &c_l_kpts,
                                                                   const std::vector<cv::Point2f> &c_r_kpts,
                                                                   const std::vector<cv::Point2f> &p_l_kpts,
                                                                   const std::vector<cv::Point2f> &p_r_kpts,
                                                                   const std::vector<cv::Point3f> &c_obj_pts,
                                                                   const std::vector<cv::Point3f> &p_obj_pts,
                                                                   std::vector<cv::Point2f> &f_c_l_kpts,
                                                                   std::vector<cv::Point2f> &f_c_r_kpts,
                                                                   std::vector<cv::Point2f> &f_p_l_kpts,
                                                                   std::vector<cv::Point2f> &f_p_r_kpts,
                                                                   std::vector<cv::Point3f> &f_c_obj_pts,
                                                                   std::vector<cv::Point3f> &f_p_obj_pts)
{
    StereoResult match_result, aux_result;
    cv::Mat f_c_l_descs, f_c_r_descs, f_p_l_descs, f_p_r_descs;
    if (matching_type_ == RATIO_MATCHING)
    {
        match_result = ratioMatching(c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_obj_pts, p_obj_pts,
                                     f_c_l_descs, f_p_l_descs, f_c_l_kpts, f_p_l_kpts, f_c_obj_pts, f_p_obj_pts);
    }
    else if (matching_type_ == ESSENTIAL_MATCHING)
    {
        match_result = geometricalMatching(false, c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_obj_pts, p_obj_pts,
                                           f_c_l_descs, f_p_l_descs, f_c_l_kpts, f_p_l_kpts, f_c_obj_pts, f_p_obj_pts);
    }
    else if (matching_type_ == HOMOGRAPHY_MATCHING)
    {
        match_result = geometricalMatching(true, c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_obj_pts, p_obj_pts,
                                           f_c_l_descs, f_p_l_descs, f_c_l_kpts, f_p_l_kpts, f_c_obj_pts, f_p_obj_pts);
    }
    else if (matching_type_ == CIRCULAR_RATIO_MATCHING ||
             matching_type_ == CIRCULAR_ESSENTIAL_MATCHING ||
             matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
    {
        std::vector<cv::Point2f> h_f_c_l_kpts, h_f_c_r_kpts, h_f_p_l_kpts, h_f_p_r_kpts;
        std::vector<cv::Point3f> c_l_camera_points = c_obj_pts, c_r_camera_points = c_obj_pts;
        std::vector<cv::Point3f> p_l_camera_points = p_obj_pts, p_r_camera_points = p_obj_pts;
        std::vector<cv::Point3f> h_f_c_l_camera_points, h_f_c_r_camera_points, h_f_p_l_camera_points, h_f_p_r_camera_points;
        if (matching_type_ == CIRCULAR_RATIO_MATCHING)
        {
            std::thread l_matcher([&] { match_result = ratioMatching(c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_l_camera_points, p_l_camera_points,
                                                                     f_c_l_descs, f_p_l_descs, h_f_c_l_kpts, h_f_p_l_kpts, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = ratioMatching(c_r_descs, p_r_descs, c_r_kpts, p_r_kpts, c_r_camera_points, p_r_camera_points,
                                                                   f_c_r_descs, f_p_r_descs, h_f_c_r_kpts, h_f_p_r_kpts, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }
        else if (matching_type_ == CIRCULAR_ESSENTIAL_MATCHING)
        {
            std::thread l_matcher([&] { match_result = geometricalMatching(false, c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_l_camera_points, p_l_camera_points,
                                                                           f_c_l_descs, f_p_l_descs, h_f_c_l_kpts, h_f_p_l_kpts, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = geometricalMatching(false, c_r_descs, p_r_descs, c_r_kpts, p_r_kpts, c_r_camera_points, p_r_camera_points,
                                                                         f_c_r_descs, f_p_r_descs, h_f_c_r_kpts, h_f_p_r_kpts, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }
        else
        {
            std::thread l_matcher([&] { match_result = geometricalMatching(true, c_l_descs, p_l_descs, c_l_kpts, p_l_kpts, c_l_camera_points, p_l_camera_points,
                                                                           f_c_l_descs, f_p_l_descs, h_f_c_l_kpts, h_f_p_l_kpts, h_f_c_l_camera_points, h_f_p_l_camera_points); });
            std::thread r_matcher([&] { aux_result = geometricalMatching(true, c_r_descs, p_r_descs, c_r_kpts, p_r_kpts, c_r_camera_points, p_r_camera_points,
                                                                         f_c_r_descs, f_p_r_descs, h_f_c_r_kpts, h_f_p_r_kpts, h_f_c_r_camera_points, h_f_p_r_camera_points); });
            l_matcher.join();
            r_matcher.join();
        }

        if (match_result == SUCCESSFUL && aux_result == SUCCESSFUL)
        {
            if (h_f_c_r_kpts.size() < h_f_c_l_kpts.size())
            {
                match_result = assembleCircularMatchings(c_r_kpts, c_l_kpts, p_r_kpts, p_l_kpts,
                                                         h_f_c_r_kpts, h_f_c_l_kpts, h_f_p_r_kpts, h_f_p_l_kpts,
                                                         h_f_c_r_camera_points, h_f_p_r_camera_points,
                                                         f_c_r_kpts, f_c_l_kpts, f_p_r_kpts, f_p_l_kpts,
                                                         f_c_obj_pts, f_p_obj_pts);
            }
            else
            {
                match_result = assembleCircularMatchings(c_l_kpts, c_r_kpts, p_l_kpts, p_r_kpts,
                                                         h_f_c_l_kpts, h_f_c_r_kpts, h_f_p_l_kpts, h_f_p_r_kpts,
                                                         h_f_c_l_camera_points, h_f_p_l_camera_points,
                                                         f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts,
                                                         f_c_obj_pts, f_p_obj_pts);
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


StereoModules::StereoResult StereoModules::estimateMotion(const std::vector<cv::Point2f>& c_l_kpts,
                                                          const std::vector<cv::Point2f>& c_r_kpts,
                                                          const std::vector<cv::Point3f>& p_obj_pts,
                                                          cv::Mat& r_vec,
                                                          cv::Mat& t_vec,
                                                          std::vector<cv::Point2f>& f_c_l_kpts,
                                                          std::vector<cv::Point2f>& f_c_r_kpts,
                                                          std::vector<cv::Point3f>& f_p_obj_pts)
{
    // Sanity check.
    r_vec.release();
    t_vec.release();

    // Estimate motion.
    std::vector<int> inliers;
    cv::Mat K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
    cv::solvePnPRansac(p_obj_pts, c_l_kpts, K, cv::Mat(), r_vec, t_vec, 
                       false, 2000, 2.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE); 

    // Filter keypoints and 3D points.
    if (c_r_kpts.size() > 0)
        for (uint i = 0; i < inliers.size(); i++)
        {
            f_c_l_kpts.push_back(c_l_kpts[inliers[i]]);
            f_c_r_kpts.push_back(c_r_kpts[inliers[i]]);
            f_p_obj_pts.push_back(p_obj_pts[inliers[i]]);
        }
    else
    {
        for (uint i = 0; i < inliers.size(); i++)
        {
            f_c_l_kpts.push_back(c_l_kpts[inliers[i]]);
            f_p_obj_pts.push_back(p_obj_pts[inliers[i]]);
        }
    }

    if (inliers.size() >= (uint)min_inliers_)
        return SUCCESSFUL;
    else
        return NOT_ENOUGH_INLIERS;   
}


StereoModules::StereoResult StereoModules::estimateMotion(const std::vector<cv::Point2f>& c_l_kpts,
                                                          const std::vector<cv::Point2f>& c_r_kpts,
                                                          const std::vector<cv::Point2f>& p_l_kpts,
                                                          const std::vector<cv::Point2f>& p_r_kpts,
                                                          const std::vector<cv::Point3f>& c_obj_pts,
                                                          const std::vector<cv::Point3f>& p_obj_pts,
                                                          cv::Mat& r_vec,
                                                          cv::Mat& t_vec,
                                                          std::vector<cv::Point2f>& f_c_l_kpts,
                                                          std::vector<cv::Point2f>& f_c_r_kpts,
                                                          std::vector<cv::Point2f>& f_p_l_kpts,
                                                          std::vector<cv::Point2f>& f_p_r_kpts,
                                                          std::vector<cv::Point3f>& f_c_obj_pts,
                                                          std::vector<cv::Point3f>& f_p_obj_pts)
{
    // Sanity check.
    r_vec.release();
    t_vec.release();

    // Estimate motion.
    std::vector<int> inliers;
    cv::Mat K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
    cv::solvePnPRansac(p_obj_pts, c_l_kpts, K, cv::Mat(), r_vec, t_vec, 
                       false, 2000, 2.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE); 

    // Filter keypoints and 3D points.
    if (c_r_kpts.size() > 0)
        for (uint i = 0; i < inliers.size(); i++)
        {
            f_c_l_kpts.push_back(c_l_kpts[inliers[i]]);
            f_c_r_kpts.push_back(c_r_kpts[inliers[i]]);
            f_p_l_kpts.push_back(p_l_kpts[inliers[i]]);
            f_p_r_kpts.push_back(p_r_kpts[inliers[i]]);
            f_c_obj_pts.push_back(c_obj_pts[inliers[i]]);
            f_p_obj_pts.push_back(p_obj_pts[inliers[i]]);
        }
    else
    {
        for (uint i = 0; i < inliers.size(); i++)
        {
            f_c_l_kpts.push_back(c_l_kpts[inliers[i]]);
            f_p_l_kpts.push_back(p_l_kpts[inliers[i]]);
            f_c_obj_pts.push_back(c_obj_pts[inliers[i]]);
            f_p_obj_pts.push_back(p_obj_pts[inliers[i]]);
        }
    }

    if (inliers.size() >= (uint)min_inliers_)
        return SUCCESSFUL;
    else
        return NOT_ENOUGH_INLIERS;   
}


StereoModules::MatchingType StereoModules::selectMatchingType(std::string& matching_type)
{
    // Sanity check
    for (uint i = 0; i < matching_type.size(); i++)
    {
        if (isupper(matching_type[i]))
            matching_type[i] = std::tolower(matching_type[i]);
    }

    std::cout << "Matching type: " << matching_type << std::endl;

    if (matching_type == "ratio_matching")
        return RATIO_MATCHING;
    else if (matching_type == "essential_matching")
        return ESSENTIAL_MATCHING;
    else if (matching_type == "homography_matching")
        return HOMOGRAPHY_MATCHING;
    else if (matching_type == "circular_ratio_matching")
        return CIRCULAR_RATIO_MATCHING;
    else if (matching_type == "circular_essential_matching")
        return CIRCULAR_ESSENTIAL_MATCHING;
    else if (matching_type == "circular_homography_matching")
        return CIRCULAR_HOMOGRAPHY_MATCHING;
    else
    {
        std::cerr << "Unforseen matching type" << std::endl;
        return RATIO_MATCHING;
    }
}


cv::Mat StereoModules::computeProjectionMatrix(const double b,
                                               const double f_x, 
                                               const double f_y, 
                                               const double c_x, 
                                               const double c_y)
{
    // Intrinsic camera matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);

    // Rotation matrix (identity for simplicity, replace with actual rotation)
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    // Translation vector (replace with actual translation)
    cv::Mat t = (cv::Mat_<double>(3, 1) << b, 0.0, 0.0);

    // Combine rotation and translation into a 3x4 extrinsic matrix [R | t]
    cv::Mat extrinsicMat;
    cv::hconcat(R, t, extrinsicMat);

    // Create the 3x4 projection matrix P = K * [R | t]
    cv::Mat P = K * extrinsicMat;    

    return P;
}

std::vector<std::vector<cv::DMatch>> StereoModules::matching(const cv::Mat &descs_0,
                                                             const cv::Mat &descs_1)
{
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(descs_0, descs_1, knn_matches, knn, match_mask);
    return knn_matches;
}


StereoModules::StereoResult StereoModules::ratioMatching(const cv::Mat &descs_0,
                                                         const cv::Mat &descs_1,
                                                         const std::vector<cv::Point2f> &kpts_0,
                                                         const std::vector<cv::Point2f> &kpts_1,
                                                         const std::vector<cv::Point3f> &obj_pts_0,
                                                         const std::vector<cv::Point3f> &obj_pts_1,
                                                         cv::Mat &f_descs_0,
                                                         cv::Mat &f_descs_1,
                                                         std::vector<cv::Point2f> &f_kpts_0,
                                                         std::vector<cv::Point2f> &f_kpts_1,
                                                         std::vector<cv::Point3f> &f_obj_pts_0,
                                                         std::vector<cv::Point3f> &f_obj_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_obj_pts_0.clear();
    f_obj_pts_1.clear();
    if (descs_0.rows < (int)(1.5 * min_inliers_) || descs_1.rows < (int)(1.5 * min_inliers_))
        return NOT_ENOUGH_KPTS;

    // During feature matching of the same stereo pair there are no 3D points.
    bool fill_3d;
    if (obj_pts_0.size() > 0)
        fill_3d = true;
    else
        fill_3d = false;

    // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> knn_matches = matching(descs_0, descs_1);

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
            f_kpts_0.push_back(kpts_0[c_idx]);
            f_kpts_1.push_back(kpts_1[p_idx]);
            f_descs_0.push_back(descs_0.row(c_idx));
            f_descs_1.push_back(descs_1.row(p_idx));

            // During feature matching of the same stereo pair there are no 3D points.
            if (fill_3d)
            {
                f_obj_pts_0.push_back(obj_pts_0[c_idx]);
                f_obj_pts_1.push_back(obj_pts_1[p_idx]);
            }
        }
    }

    // Sanity check
    if (f_kpts_0.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_F_KPTS;
    else
        return SUCCESSFUL;
}


StereoModules::StereoResult StereoModules::geometricalMatching(bool coplanar_pts,
                                                               const cv::Mat &descs_0,
                                                               const cv::Mat &descs_1,
                                                               const std::vector<cv::Point2f> &kpts_0,
                                                               const std::vector<cv::Point2f> &kpts_1,
                                                               const std::vector<cv::Point3f> &obj_pts_0,
                                                               const std::vector<cv::Point3f> &obj_pts_1,
                                                               cv::Mat &f_descs_0,
                                                               cv::Mat &f_descs_1,
                                                               std::vector<cv::Point2f> &f_kpts_0,
                                                               std::vector<cv::Point2f> &f_kpts_1,
                                                               std::vector<cv::Point3f> &f_obj_pts_0,
                                                               std::vector<cv::Point3f> &f_obj_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_obj_pts_0.clear();
    f_obj_pts_1.clear();
    if (descs_0.rows < (int)(1.5 * min_inliers_) || descs_1.rows < (int)(1.5 * min_inliers_))
        return NOT_ENOUGH_KPTS;

    // // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> matches = matching(descs_0, descs_1);

    int c_idx, p_idx;
    cv::Mat m_descs_0, m_descs_1;
    std::vector<cv::Point2f> m_kpts_0, m_kpts_1;
    std::vector<cv::Point3f> m_obj_pts_0, m_obj_pts_1;
    for (uint i = 0; i < matches.size(); i++)
    {
        c_idx = matches[i][0].queryIdx;
        p_idx = matches[i][0].trainIdx;
        m_kpts_0.push_back(kpts_0[c_idx]);
        m_kpts_1.push_back(kpts_1[p_idx]);
        m_descs_0.push_back(descs_0.row(c_idx));
        m_descs_1.push_back(descs_1.row(p_idx));
        m_obj_pts_0.push_back(obj_pts_0[c_idx]);
        m_obj_pts_1.push_back(obj_pts_1[p_idx]);
    }

    // Compute homography or essential matrix
    cv::Mat M, mask, K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
    try
    {
        if (coplanar_pts)
            M = cv::findHomography(m_kpts_0, m_kpts_1, mask, cv::RANSAC, 1.0);
        else
            M = cv::findEssentialMat(m_kpts_0, m_kpts_1, K, cv::RANSAC, 0.999, 1.0, mask);
    }
    catch (cv::Exception &e)
    {
        return GEOMETRIC_PROBLEM;
    }

    // Filter outliers using homography or essential matrix
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_kpts_0.push_back(m_kpts_0[i]);
            f_kpts_1.push_back(m_kpts_1[i]);
            f_descs_0.push_back(m_descs_0.row(i));
            f_descs_1.push_back(m_descs_1.row(i));
            f_obj_pts_0.push_back(m_obj_pts_0[i]);
            f_obj_pts_1.push_back(m_obj_pts_1[i]);
        }
    }

    // Sanity check
    if (f_kpts_0.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_F_KPTS;
    else
        return SUCCESSFUL;
}


StereoModules::StereoResult StereoModules::assembleCircularMatchings(const std::vector<cv::Point2f> &c_0_stereo_kpts,
                                                                     const std::vector<cv::Point2f> &c_1_stereo_kpts,
                                                                     const std::vector<cv::Point2f> &p_0_stereo_kpts,
                                                                     const std::vector<cv::Point2f> &p_1_stereo_kpts,
                                                                     const std::vector<cv::Point2f> &c_0_successive_kpts,
                                                                     const std::vector<cv::Point2f> &c_1_successive_kpts,
                                                                     const std::vector<cv::Point2f> &p_0_successive_kpts,
                                                                     const std::vector<cv::Point2f> &p_1_successive_kpts,
                                                                     const std::vector<cv::Point3f>& c_successive_obj_pts,
                                                                     const std::vector<cv::Point3f>& p_successive_obj_pts,
                                                                     std::vector<cv::Point2f> &f_c_0_kpts,
                                                                     std::vector<cv::Point2f> &f_c_1_kpts,
                                                                     std::vector<cv::Point2f> &f_p_0_kpts,
                                                                     std::vector<cv::Point2f> &f_p_1_kpts,
                                                                     std::vector<cv::Point3f>& f_c_obj_pts,
                                                                     std::vector<cv::Point3f>& f_p_obj_pts)
{
    uint j;
    uint k;
    uint l;
    for (uint i = 0; i < p_0_successive_kpts.size(); i++)
    {
        for (j = 0; j < p_0_stereo_kpts.size(); j++)
        {
            if (p_0_successive_kpts[i] == p_0_stereo_kpts[j])
            {
                for (k = 0; k < p_1_successive_kpts.size(); k++)
                {
                    if (p_1_stereo_kpts[j] == p_1_successive_kpts[k])
                    {
                        for (l = 0; l < c_1_stereo_kpts.size(); l++)
                        {
                            if (c_1_successive_kpts[k] == c_1_stereo_kpts[l])
                            {
                                if (c_0_stereo_kpts[l] == c_0_successive_kpts[i])
                                {
                                    f_c_0_kpts.push_back(c_0_successive_kpts[i]);
                                    f_c_1_kpts.push_back(c_1_successive_kpts[k]);
                                    f_p_0_kpts.push_back(p_0_successive_kpts[i]);
                                    f_p_1_kpts.push_back(p_1_successive_kpts[k]);
                                    f_c_obj_pts.push_back(c_successive_obj_pts[i]);
                                    f_p_obj_pts.push_back(p_successive_obj_pts[i]);
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
    if (f_c_0_kpts.size() < (uint)(1.5 * min_inliers_))
        return NOT_ENOUGH_CIRCLES;
    else
        return SUCCESSFUL;
}