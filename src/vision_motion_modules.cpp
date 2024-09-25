#include "vision_motion_modules.hpp"


VisionMotionModules::VisionMotionModules(const double ratio,
                                         std::string& matching_type,
                                         std::string& motion_estimation_type)
{
    ratio_ = ratio;
    matching_type_ = selectMatchingType(matching_type);
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


VisionMotionModules::VisionMotionModules(const double ratio,
                                         const cv::Mat& l_proj_mat,
                                         const cv::Mat& r_proj_mat,
                                         std::string& matching_type,
                                         std::string& motion_estimation_type)
{
    ratio_ = ratio;
    l_proj_mat_ = l_proj_mat;
    r_proj_mat_ = r_proj_mat;
    matching_type_ = selectMatchingType(matching_type);
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


VisionMotionModules::VisionMotionModules(const double b,
                                         const double f_x,
                                         const double f_y,
                                         const double c_x,
                                         const double c_y,
                                         const double ratio,
                                         std::string& matching_type,
                                         std::string& motion_estimation_type)
{
    ratio_ = ratio;
    matching_type_ = selectMatchingType(matching_type);
    r_proj_mat_ = computeProjectionMatrix(b, f_x, f_x, c_x, c_y);
    l_proj_mat_ = computeProjectionMatrix(0.0, f_x, f_x, c_x, c_y);
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


void VisionMotionModules::detectKeypoints(const cv::Mat& img,
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
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(6, cv::Size(8, 8));
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

    // Select detector and extract keypoints
    cv::Ptr<cv::Feature2D> detector;
    if (detector_name == "sift")
    {
        detector = cv::xfeatures2d::SIFT::create(1800, 3, 0.04, 10, 1.6);
        detector->detectAndCompute(proc_img, cv::noArray(), kpts, descs);
    }
    else if (detector_name == "surf")
    {
        detector = cv::xfeatures2d::SURF::create(100, 4, 3, false, false);
        detector->detectAndCompute(proc_img, cv::noArray(), kpts, descs);
    }
    else if (detector_name == "orb")
    {  
        detector = cv::ORB::create(1500, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        detector->detectAndCompute(proc_img, cv::noArray(), kpts, descs);
    }
    else
    {
        detector = cv::xfeatures2d::SIFT::create(1500, 3, 0.04, 10, 1.6);
        detector->detectAndCompute(proc_img, cv::noArray(), kpts, descs);
    }
}


void VisionMotionModules::detectStereoKeypoints(const cv::Mat& l_img,
                                                const cv::Mat& r_img,
                                                const bool enable_enhacement,
                                                const std::string& detector_name,
                                                cv::Mat& l_descs,
                                                cv::Mat& r_descs,
                                                std::vector<cv::KeyPoint>& l_kpts,
                                                std::vector<cv::KeyPoint>& r_kpts)
{ 
    std::thread l_detection(&VisionMotionModules::detectKeypoints, this, std::ref(l_img), 
                            enable_enhacement, std::ref(detector_name), 
                            std::ref(l_descs), std::ref(l_kpts));
    std::thread r_detection(&VisionMotionModules::detectKeypoints, this, std::ref(r_img),
                            enable_enhacement, std::ref(detector_name), 
                            std::ref(r_descs), std::ref(r_kpts));
    l_detection.join();
    r_detection.join();
}


void VisionMotionModules::matchStereoKeypoints(const cv::Mat& l_descs,
                                               const cv::Mat& r_descs,
                                               const std::vector<cv::KeyPoint>& l_kpts,
                                               const std::vector<cv::KeyPoint>& r_kpts,
                                               cv::Mat& f_l_descs,
                                               cv::Mat& f_r_descs,
                                               std::vector<cv::KeyPoint>& f_l_kpts,
                                               std::vector<cv::KeyPoint>& f_r_kpts)
{
    // Sanity check
    f_l_kpts.clear();
    f_r_kpts.clear();
    f_l_descs.release();
    f_r_descs.release();

    // Dummy variables
    std::vector<int> l_subset_ids, r_subset_ids, f_l_subset_ids, f_r_subset_ids;
    std::vector<cv::Point3f> points_3d_0, points_3d_1, f_points_3d_0, f_points_3d_1;

    // Perform keypoint matching and Lowe's ratio
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::KeyPoint> h_f_l_kpts, h_f_r_kpts;
    ratioMatching(l_descs, r_descs, l_subset_ids, r_subset_ids, l_kpts, r_kpts, points_3d_0, points_3d_1,
                  h_f_l_descs, h_f_r_descs, f_l_subset_ids, f_r_subset_ids, h_f_l_kpts, h_f_r_kpts, 
                  f_points_3d_0, f_points_3d_1);

    // Epipolar filtering
    for (uint i = 0; i < h_f_l_kpts.size(); i++)
    {
        if (abs(h_f_l_kpts[i].pt.y - h_f_r_kpts[i].pt.y) <= 1.0)
        {
            f_l_kpts.push_back(h_f_l_kpts[i]);
            f_r_kpts.push_back(h_f_r_kpts[i]);
            f_l_descs.push_back(h_f_l_descs.row(i));
            f_r_descs.push_back(h_f_r_descs.row(i));
        }
    }
}


void VisionMotionModules::compute3DPoints(const cv::Mat& l_descs,
                                          const cv::Mat& r_descs,
                                          const std::vector<cv::KeyPoint>& l_kpts,
                                          const std::vector<cv::KeyPoint>& r_kpts,
                                          cv::Mat& f_l_descs,
                                          cv::Mat& f_r_descs,
                                          std::vector<cv::KeyPoint>& f_l_kpts,
                                          std::vector<cv::KeyPoint>& f_r_kpts,
                                          std::vector<cv::Point3f>& f_cam_pts)
{
    // Triangulates keypoints.
    cv::Mat pts_4d, cam_pts;
    std::vector<cv::Point2f> l_pts, r_pts;
    cv::KeyPoint::convert(l_kpts, l_pts);
    cv::KeyPoint::convert(r_kpts, r_pts);
    cv::triangulatePoints(l_proj_mat_, r_proj_mat_, l_pts, r_pts, pts_4d);
    cv::convertPointsFromHomogeneous(pts_4d.t(), cam_pts);
    cam_pts = cam_pts.reshape(1, cam_pts.rows).clone();

    // Computes baseline.
    double f_u = l_proj_mat_.at<double>(0, 0);
    double t_x = r_proj_mat_.at<double>(0, 3);
    double b = (t_x / -f_u);

    // Filters "far" keypoints.
    cv::Point3f pt;
    double z_limit = 40 * b;
    for (int i = 0; i < cam_pts.rows; i++)
    {
        pt.x = cam_pts.at<float>(i, 0);
        pt.y = cam_pts.at<float>(i, 1);
        pt.z = cam_pts.at<float>(i, 2);
        if (isfinite(pt.x) &&
            isfinite(pt.y) &&
            isfinite(pt.z) &&
            pt.z > 0.0 &&
            pt.z <= z_limit)
        {
            f_cam_pts.push_back(pt);
            f_l_kpts.push_back(l_kpts[i]);
            f_r_kpts.push_back(r_kpts[i]);
            f_l_descs.push_back(l_descs.row(i));
            f_r_descs.push_back(r_descs.row(i));
        }
    }
}


void VisionMotionModules::matchNonStereoKeypoints(const cv::Mat& l_descs_0,
                                                  const cv::Mat& r_descs_0,
                                                  const cv::Mat& l_descs_1,
                                                  const cv::Mat& r_descs_1,
                                                  const std::vector<cv::KeyPoint>& l_kpts_0,
                                                  const std::vector<cv::KeyPoint>& r_kpts_0,
                                                  const std::vector<cv::KeyPoint>& l_kpts_1,
                                                  const std::vector<cv::KeyPoint>& r_kpts_1,
                                                  const std::vector<cv::Point3f>& cam_pts_0,
                                                  const std::vector<cv::Point3f>& cam_pts_1,
                                                  std::vector<cv::KeyPoint>& f_l_kpts_0,
                                                  std::vector<cv::KeyPoint>& f_r_kpts_0,
                                                  std::vector<cv::KeyPoint>& f_l_kpts_1,
                                                  std::vector<cv::KeyPoint>& f_r_kpts_1,
                                                  std::vector<cv::Point3f>& f_cam_pts_0,
                                                  std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Dummy variables.
    std::vector<int> cluster_ids_0, cluster_ids_1, f_cluster_ids_0, f_cluster_ids_1;
    
    // Matches keypoints and removes outliers using a general function.
    matchKeypointClusters(l_descs_0, r_descs_0, l_descs_1, r_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, r_kpts_0, l_kpts_1, r_kpts_1, cam_pts_0, cam_pts_1,
                          f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_r_kpts_0, f_l_kpts_1, f_r_kpts_1, f_cam_pts_0, f_cam_pts_1);
}


void VisionMotionModules::matchKeypointClusters(const cv::Mat& l_descs_0,
                                                const cv::Mat& r_descs_0,
                                                const cv::Mat& l_descs_1,
                                                const cv::Mat& r_descs_1,
                                                const std::vector<int>& cluster_ids_0,
                                                const std::vector<int>& cluster_ids_1,
                                                const std::vector<cv::KeyPoint>& l_kpts_0,
                                                const std::vector<cv::KeyPoint>& r_kpts_0,
                                                const std::vector<cv::KeyPoint>& l_kpts_1,
                                                const std::vector<cv::KeyPoint>& r_kpts_1,
                                                const std::vector<cv::Point3f>& cam_pts_0,
                                                const std::vector<cv::Point3f>& cam_pts_1,
                                                std::vector<int>& f_cluster_ids_0,
                                                std::vector<int>& f_cluster_ids_1,
                                                std::vector<cv::KeyPoint>& f_l_kpts_0,
                                                std::vector<cv::KeyPoint>& f_r_kpts_0,
                                                std::vector<cv::KeyPoint>& f_l_kpts_1,
                                                std::vector<cv::KeyPoint>& f_r_kpts_1,
                                                std::vector<cv::Point3f>& f_cam_pts_0,
                                                std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Dummy variables.
    cv::Mat f_l_descs_0, f_r_descs_0, f_l_descs_1, f_r_descs_1;

    // Matches.
    if (matching_type_ == RATIO_MATCHING)
    {
        ratioMatching(l_descs_0, l_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, l_kpts_1, cam_pts_0, cam_pts_1,
                      f_l_descs_0, f_l_descs_1, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_l_kpts_1, f_cam_pts_0, f_cam_pts_1);
    }
    else if (matching_type_ == ESSENTIAL_MATCHING)
    {
        geometricalMatching(false, l_descs_0, l_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, l_kpts_1, cam_pts_0, cam_pts_1,
                            f_l_descs_0, f_l_descs_1, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_l_kpts_1, f_cam_pts_0, f_cam_pts_1);
    }
    else if (matching_type_ == HOMOGRAPHY_MATCHING)
    {
        geometricalMatching(true, l_descs_0, l_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, l_kpts_1, cam_pts_0, cam_pts_1,
                            f_l_descs_0, f_l_descs_1, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_l_kpts_1, f_cam_pts_0, f_cam_pts_1);
    }
    else if (matching_type_ == CIRCULAR_RATIO_MATCHING ||
             matching_type_ == CIRCULAR_ESSENTIAL_MATCHING ||
             matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
    {
        // Temporal variables
        std::vector<cv::Point3f> l_cam_pts_0 = cam_pts_0, r_cam_pts_0 = cam_pts_0;
        std::vector<cv::Point3f> l_cam_pts_1 = cam_pts_1, r_cam_pts_1 = cam_pts_1;
        std::vector<cv::KeyPoint> h_f_l_kpts_0, h_f_r_kpts_0, h_f_l_kpts_1, h_f_r_kpts_1;
        std::vector<int> l_cluster_ids_0 = cluster_ids_0, r_cluster_ids_0 = cluster_ids_0;
        std::vector<int> l_cluster_ids_1 = cluster_ids_1, r_cluster_ids_1 = cluster_ids_1;
        std::vector<cv::Point3f> h_f_l_cam_pts_0, h_f_r_cam_pts_0, h_f_l_cam_pts_1, h_f_r_cam_pts_1;
        std::vector<int> h_f_l_cluster_ids_0, h_f_r_cluster_ids_0, h_f_l_cluster_ids_1, h_f_r_cluster_ids_1;
        if (matching_type_ == CIRCULAR_RATIO_MATCHING)
        {
            std::thread l_matcher(&VisionMotionModules::ratioMatching, this, std::ref(l_descs_0), std::ref(l_descs_1), std::ref(l_cluster_ids_0), std::ref(l_cluster_ids_1),
                                  std::ref(l_kpts_0), std::ref(l_kpts_1), std::ref(l_cam_pts_0), std::ref(l_cam_pts_1), std::ref(f_l_descs_0), std::ref(f_l_descs_1),
                                  std::ref(h_f_l_cluster_ids_0), std::ref(h_f_l_cluster_ids_1), std::ref(h_f_l_kpts_0), std::ref(h_f_l_kpts_1), std::ref(h_f_l_cam_pts_0), std::ref(h_f_l_cam_pts_1));
            std::thread r_matcher(&VisionMotionModules::ratioMatching, this, std::ref(r_descs_0), std::ref(r_descs_1), std::ref(r_cluster_ids_0), std::ref(r_cluster_ids_1),
                                  std::ref(r_kpts_0), std::ref(r_kpts_1), std::ref(r_cam_pts_0), std::ref(r_cam_pts_1), std::ref(f_r_descs_0), std::ref(f_r_descs_1),
                                  std::ref(h_f_r_cluster_ids_0), std::ref(h_f_r_cluster_ids_1), std::ref(h_f_r_kpts_0), std::ref(h_f_r_kpts_1), std::ref(h_f_r_cam_pts_0), std::ref(h_f_r_cam_pts_1));
            l_matcher.join();
            r_matcher.join();
        }
        else if (matching_type_ == CIRCULAR_ESSENTIAL_MATCHING)
        {
            std::thread l_matcher(&VisionMotionModules::geometricalMatching, this, false, std::ref(l_descs_0), std::ref(l_descs_1), std::ref(l_cluster_ids_0), std::ref(l_cluster_ids_1),
                                  std::ref(l_kpts_0), std::ref(l_kpts_1), std::ref(l_cam_pts_0), std::ref(l_cam_pts_1), std::ref(f_l_descs_0), std::ref(f_l_descs_1),
                                  std::ref(h_f_l_cluster_ids_0), std::ref(h_f_l_cluster_ids_1), std::ref(h_f_l_kpts_0), std::ref(h_f_l_kpts_1), std::ref(h_f_l_cam_pts_0), std::ref(h_f_l_cam_pts_1));
            std::thread r_matcher(&VisionMotionModules::geometricalMatching, this, false, std::ref(r_descs_0), std::ref(r_descs_1), std::ref(r_cluster_ids_0), std::ref(r_cluster_ids_1),
                                  std::ref(r_kpts_0), std::ref(r_kpts_1), std::ref(r_cam_pts_0), std::ref(r_cam_pts_1), std::ref(f_r_descs_0), std::ref(f_r_descs_1), 
                                  std::ref(h_f_r_cluster_ids_0), std::ref(h_f_r_cluster_ids_1), std::ref(h_f_r_kpts_0), std::ref(h_f_r_kpts_1), std::ref(h_f_r_cam_pts_0), std::ref(h_f_r_cam_pts_1));
            l_matcher.join();
            r_matcher.join();
        }
        else
        {
            std::thread l_matcher(&VisionMotionModules::geometricalMatching, this, true, std::ref(l_descs_0), std::ref(l_descs_1), std::ref(l_cluster_ids_0), std::ref(l_cluster_ids_1),
                                  std::ref(l_kpts_0), std::ref(l_kpts_1), std::ref(l_cam_pts_0), std::ref(l_cam_pts_1), std::ref(f_l_descs_0), std::ref(f_l_descs_1), 
                                  std::ref(h_f_l_cluster_ids_0), std::ref(h_f_l_cluster_ids_1), std::ref(h_f_l_kpts_0), std::ref(h_f_l_kpts_1), std::ref(h_f_l_cam_pts_0), std::ref(h_f_l_cam_pts_1));
            std::thread r_matcher(&VisionMotionModules::geometricalMatching, this, true, std::ref(r_descs_0), std::ref(r_descs_1), std::ref(r_cluster_ids_0), std::ref(r_cluster_ids_1),
                                  std::ref(r_kpts_0), std::ref(r_kpts_1), std::ref(r_cam_pts_0), std::ref(r_cam_pts_1), std::ref(f_r_descs_0), std::ref(f_r_descs_1),
                                  std::ref(h_f_r_cluster_ids_0), std::ref(h_f_r_cluster_ids_1), std::ref(h_f_r_kpts_0), std::ref(h_f_r_kpts_1), std::ref(h_f_r_cam_pts_0), std::ref(h_f_r_cam_pts_1));
            l_matcher.join();
            r_matcher.join();
        }

        if (h_f_r_kpts_0.size() < h_f_l_kpts_0.size())
        {
            assembleCircularMatchings(r_kpts_0, l_kpts_0, r_kpts_1, l_kpts_1, h_f_r_kpts_0, h_f_l_kpts_0, h_f_r_kpts_1, h_f_l_kpts_1,
                                      h_f_r_cam_pts_0, h_f_r_cam_pts_1, h_f_r_cluster_ids_0, h_f_r_cluster_ids_1, 
                                      f_r_kpts_0, f_l_kpts_0, f_r_kpts_1, f_l_kpts_1, f_cam_pts_0, f_cam_pts_1, f_cluster_ids_0, f_cluster_ids_1);
        }
        else
        {
            assembleCircularMatchings(l_kpts_0, r_kpts_0, l_kpts_1, r_kpts_1, h_f_l_kpts_0, h_f_r_kpts_0, h_f_l_kpts_1, h_f_r_kpts_1,
                                      h_f_l_cam_pts_0, h_f_l_cam_pts_1, h_f_l_cluster_ids_0, h_f_l_cluster_ids_1,
                                      f_l_kpts_0, f_r_kpts_0, f_l_kpts_1, f_r_kpts_1, f_cam_pts_0, f_cam_pts_1, f_cluster_ids_0, f_cluster_ids_1);
        }
    }
    else
    {
        std::cerr << "Unforeseen type of matching!" << std::endl;
    }
}


void VisionMotionModules::estimateMotion(const std::vector<cv::KeyPoint>& l_kpts_0,
                                         const std::vector<cv::KeyPoint>& r_kpts_0,
                                         const std::vector<cv::KeyPoint>& l_kpts_1,
                                         const std::vector<cv::KeyPoint>& r_kpts_1,
                                         const std::vector<cv::Point3f>& cam_pts_0,
                                         const std::vector<cv::Point3f>& cam_pts_1,
                                         cv::Mat& r_vec,
                                         cv::Mat& t_vec,
                                         std::vector<cv::KeyPoint>& f_l_kpts_0,
                                         std::vector<cv::KeyPoint>& f_r_kpts_0,
                                         std::vector<cv::KeyPoint>& f_l_kpts_1,
                                         std::vector<cv::KeyPoint>& f_r_kpts_1,
                                         std::vector<cv::Point3f>& f_cam_pts_0,
                                         std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Dummy variables.
    std::vector<int> cluster_ids_0, cluster_ids_1, f_cluster_ids_0, f_cluster_ids_1;
    estimateMotion(cluster_ids_0, cluster_ids_1, l_kpts_0, r_kpts_0, l_kpts_1, r_kpts_1, cam_pts_0, cam_pts_1,
                   r_vec, t_vec, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_r_kpts_0, f_l_kpts_1, f_r_kpts_1, f_cam_pts_0, f_cam_pts_1);
}


void VisionMotionModules::estimateMotion(const std::vector<int>& cluster_ids_0,
                                         const std::vector<int>& cluster_ids_1,
                                         const std::vector<cv::KeyPoint>& l_kpts_0,
                                         const std::vector<cv::KeyPoint>& r_kpts_0,
                                         const std::vector<cv::KeyPoint>& l_kpts_1,
                                         const std::vector<cv::KeyPoint>& r_kpts_1,
                                         const std::vector<cv::Point3f>& cam_pts_0,
                                         const std::vector<cv::Point3f>& cam_pts_1,
                                         cv::Mat& r_vec,
                                         cv::Mat& t_vec,
                                         std::vector<int>& f_cluster_ids_0,
                                         std::vector<int>& f_cluster_ids_1,        
                                         std::vector<cv::KeyPoint>& f_l_kpts_0,
                                         std::vector<cv::KeyPoint>& f_r_kpts_0,
                                         std::vector<cv::KeyPoint>& f_l_kpts_1,
                                         std::vector<cv::KeyPoint>& f_r_kpts_1,
                                         std::vector<cv::Point3f>& f_cam_pts_0,
                                         std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Sanity check.
    r_vec.release();
    t_vec.release();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    bool fill_right;
    if (r_kpts_0.size() > 0)
        fill_right = true;
    else
        fill_right = false;

    // If there are no clusters, the filtered cluster vectors must not be filled.
    bool fill_clusters;
    if (cluster_ids_0.size() > 0)
        fill_clusters = true;
    else
        fill_clusters = false;

    // Estimate motion.
    std::vector<int> inliers;
    std::vector<cv::Point2f> l_pts_0;
    cv::KeyPoint::convert(l_kpts_0, l_pts_0);
    cv::Mat K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
    cv::solvePnPRansac(cam_pts_1, l_pts_0, K, cv::Mat(), r_vec, t_vec, 
                       false, 2000, 2.0, 0.99, inliers, motion_estimation_type_);

    // Filter keypoints, 3D points and clusters.
    for (uint i = 0; i < inliers.size(); i++)
    {
        f_l_kpts_0.push_back(l_kpts_0[inliers[i]]);
        f_l_kpts_1.push_back(l_kpts_1[inliers[i]]);
        f_cam_pts_0.push_back(cam_pts_0[inliers[i]]);
        f_cam_pts_1.push_back(cam_pts_1[inliers[i]]);
        if (fill_right)
        {
            f_r_kpts_0.push_back(r_kpts_0[inliers[i]]);
            f_r_kpts_1.push_back(r_kpts_1[inliers[i]]);
        }
        if (fill_clusters)
        {
            f_cluster_ids_0.push_back(cluster_ids_0[inliers[i]]);
            f_cluster_ids_1.push_back(cluster_ids_1[inliers[i]]);
        }
    }
}


std::vector<std::vector<std::pair<int, cv::KeyPoint>>> VisionMotionModules::bucketKeypoints(const int n_rows_img,
                                                                                            const int n_cols_img,
                                                                                            const uint n_kpts_bucket,
                                                                                            const std::vector<cv::KeyPoint>& l_kpts_0,
                                                                                            const std::vector<cv::KeyPoint>& r_kpts_0,
                                                                                            const std::vector<cv::KeyPoint>& l_kpts_1,
                                                                                            const std::vector<cv::KeyPoint>& r_kpts_1,
                                                                                            const std::vector<cv::Point3f>& cam_pts_0,
                                                                                            const std::vector<cv::Point3f>& cam_pts_1,
                                                                                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                                                                                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                                                                                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                                                                                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                                                                                            std::vector<cv::Point3f>& f_cam_pts_0,
                                                                                            std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Sanity check.
    f_l_kpts_0.clear();
    f_r_kpts_0.clear();
    f_l_kpts_1.clear();
    f_r_kpts_1.clear();
    f_cam_pts_0.clear();
    f_cam_pts_1.clear();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    bool fill_right;
    if (r_kpts_0.size() > 0)
        fill_right = true;
    else
        fill_right = false;

    // Calculate the number of rows and columns that each bucket has.
    float n_rows_bucket = (float)n_rows_img / 10.0;
    float n_cols_bucket = (float)n_cols_img / 10.0;

    // Fill buckets with keypoints and its indices.
    int bucket_idx, bucket_row, bucket_col;
    std::vector<std::vector<std::pair<int, cv::KeyPoint>>> buckets(100);
    for (uint i = 0; i < l_kpts_0.size(); i++)
    {
        bucket_col = (int) floor(l_kpts_0[i].pt.x / n_cols_bucket);
        bucket_row = (int) floor(l_kpts_0[i].pt.y / n_rows_bucket);
        bucket_idx = (int) (10 * bucket_col + bucket_row);
        buckets[bucket_idx].push_back(std::make_pair(i, l_kpts_0[i]));
    }

    // Remove excess keypoints from the buckets.
    int idx;
    float slope, delta_x;
    cv::KeyPoint l_kpt_0, l_kpt_1;
    std::vector<std::pair<float, std::pair<int, cv::KeyPoint>>> bucket_slopes;
    for (uint i = 0; i < buckets.size(); i++)
    {
        if (buckets[i].size() > n_kpts_bucket)
        {
            // Calculate the coincidence slopes.
            bucket_slopes.clear();
            for (uint j = 0; j < buckets[i].size(); j++)
            {
                idx = buckets[i][j].first;
                l_kpt_0 = l_kpts_0[idx];
                l_kpt_1 = l_kpts_1[idx];
                delta_x = l_kpt_1.pt.x - l_kpt_0.pt.x;
                if (delta_x != 0)
                    slope = (l_kpt_1.pt.y - l_kpt_0.pt.y) / (delta_x);
                else
                    slope = 0.0;
                bucket_slopes.push_back(std::make_pair(slope, buckets[i][j]));
            }

            // Sort the keypoints of the bucket according to the slope of their matches.
            std::sort(bucket_slopes.begin(), bucket_slopes.end(), SortBucketsByMatchesSlopesAscending());

            // Detects whether the number of keypoints is odd or even.
            bool odd = true;
            if (bucket_slopes.size() % 2 == 0)
                odd = false;

            // Gets n center values.
            float median = 0.0;
            bool init = false;
            std::vector<std::pair<int, cv::KeyPoint>> inliers;
            int half_longitude = std::ceil(bucket_slopes.size() / 2);
            int idx_backwards = half_longitude - 1;
            uint idx_forwards = half_longitude + 1;
            while (inliers.size() < n_kpts_bucket)
            {
                if (!init)
                {
                    if (odd)
                    {
                        median = bucket_slopes[half_longitude].first;
                        inliers.push_back(bucket_slopes[half_longitude].second);
                    }
                    else
                    {
                        median = ((bucket_slopes[half_longitude].first + bucket_slopes[idx_backwards].first) / 2);
                        inliers.push_back(bucket_slopes[half_longitude].second);
                        if (n_kpts_bucket > 1)
                        {
                            inliers.push_back(bucket_slopes[idx_backwards].second);
                            idx_backwards--;
                        }
                    }
                    init = true;
                }
                else
                {
                    if (idx_backwards < 0)
                    {
                        inliers.push_back(bucket_slopes[idx_forwards].second);
                        idx_forwards++;
                    }
                    else if (idx_forwards >= bucket_slopes.size())
                    {
                        inliers.push_back(bucket_slopes[idx_backwards].second);
                        idx_backwards--;
                    }
                    else if ((median - bucket_slopes[idx_backwards].first) < (bucket_slopes[idx_forwards].first - median))
                    {
                        inliers.push_back(bucket_slopes[idx_backwards].second);
                        idx_backwards--;
                    }
                    else
                    {
                        inliers.push_back(bucket_slopes[idx_forwards].second);
                        idx_forwards++;
                    }
                }
            }

            buckets[i] = inliers;
        }

        for (uint j = 0; j < buckets[i].size(); j++)
        {
            idx = buckets[i][j].first;
            f_l_kpts_0.push_back(l_kpts_0[idx]);
            f_l_kpts_1.push_back(l_kpts_1[idx]);
            f_cam_pts_0.push_back(cam_pts_0[idx]);
            f_cam_pts_1.push_back(cam_pts_1[idx]);

            if (fill_right)
            {
                f_r_kpts_0.push_back(r_kpts_0[idx]);
                f_r_kpts_1.push_back(r_kpts_1[idx]);
            }
        }
    }

    return buckets;
}


VisionMotionModules::MatchingType VisionMotionModules::selectMatchingType(std::string& matching_type)
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


cv::SolvePnPMethod VisionMotionModules::selectMotionEstimationType(std::string& motion_estimation_type)
{
    // Sanity check
    for (uint i = 0; i < motion_estimation_type.size(); i++)
    {
        if (isupper(motion_estimation_type[i]))
            motion_estimation_type[i] = std::tolower(motion_estimation_type[i]);
    }

    std::cout << "Motion estimation type: " << motion_estimation_type << std::endl;

    if (motion_estimation_type == "solvepnp_iterative" || motion_estimation_type == "iterative")
        return cv::SOLVEPNP_ITERATIVE;
    else if (motion_estimation_type == "solvepnp_p3p" || motion_estimation_type == "p3p")
        return cv::SOLVEPNP_P3P;
    else if (motion_estimation_type == "solvepnp_ap3p" || motion_estimation_type == "ap3p")
        return cv::SOLVEPNP_AP3P;
    else if (motion_estimation_type == "solvepnp_epnp" || motion_estimation_type == "epnp")
        return cv::SOLVEPNP_EPNP;
    else
    {
        std::cerr << "Unforseen motion estimation type" << std::endl;
        return cv::SOLVEPNP_EPNP;
    }
}


cv::Mat VisionMotionModules::computeProjectionMatrix(const double b,
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

std::vector<std::vector<cv::DMatch>> VisionMotionModules::matching(const cv::Mat& descs_0,
                                                                   const cv::Mat& descs_1)
{
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(descs_0, descs_1, knn_matches, knn, match_mask);
    return knn_matches;
}


void VisionMotionModules::ratioMatching(const cv::Mat& descs_0,
                                        const cv::Mat& descs_1,
                                        const std::vector<int>& cluster_ids_0,
                                        const std::vector<int>& cluster_ids_1,
                                        const std::vector<cv::KeyPoint>& kpts_0,
                                        const std::vector<cv::KeyPoint>& kpts_1,
                                        const std::vector<cv::Point3f>& cam_pts_0,
                                        const std::vector<cv::Point3f>& cam_pts_1,
                                        cv::Mat& f_descs_0,
                                        cv::Mat& f_descs_1,
                                        std::vector<int>& f_cluster_ids_0,
                                        std::vector<int>& f_cluster_ids_1,
                                        std::vector<cv::KeyPoint>& f_kpts_0,
                                        std::vector<cv::KeyPoint>& f_kpts_1,
                                        std::vector<cv::Point3f>& f_cam_pts_0,
                                        std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_cam_pts_0.clear();
    f_cam_pts_1.clear();
    f_cluster_ids_0.clear();
    f_cluster_ids_1.clear();
    if (descs_0.rows < 10 || descs_1.rows < 10)
        return;

    // During feature matching of the same stereo pair there are no 3D points.
    bool fill_3d;
    if (cam_pts_0.size() > 0)
        fill_3d = true;
    else
        fill_3d = false;

    // If there are no clusters, the filtered cluster vectors must not be filled.
    bool fill_clusters;
    if (cluster_ids_0.size() > 0)
        fill_clusters = true;
    else
        fill_clusters = false;

    // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> knn_matches = matching(descs_0, descs_1);

    // Lowe's ratio
    int idx_0, idx_1;
    for (uint i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i].size() < 2)
            continue;

        if (knn_matches[i][0].distance <= knn_matches[i][1].distance * ratio_)
        {
            idx_0 = knn_matches[i][0].queryIdx;
            idx_1 = knn_matches[i][0].trainIdx;
            f_kpts_0.push_back(kpts_0[idx_0]);
            f_kpts_1.push_back(kpts_1[idx_1]);
            f_descs_0.push_back(descs_0.row(idx_0));
            f_descs_1.push_back(descs_1.row(idx_1));
            if (fill_3d)
            {
                f_cam_pts_0.push_back(cam_pts_0[idx_0]);
                f_cam_pts_1.push_back(cam_pts_1[idx_1]);
            }
            if (fill_clusters)
            {
                f_cluster_ids_0.push_back(cluster_ids_0[idx_0]);
                f_cluster_ids_1.push_back(cluster_ids_1[idx_1]);
            }
        }
    }
}


void VisionMotionModules::geometricalMatching(bool coplanar_pts,
                                              const cv::Mat& descs_0,
                                              const cv::Mat& descs_1,
                                              const std::vector<int>& cluster_ids_0,
                                              const std::vector<int>& cluster_ids_1,
                                              const std::vector<cv::KeyPoint>& kpts_0,
                                              const std::vector<cv::KeyPoint>& kpts_1,
                                              const std::vector<cv::Point3f>& cam_pts_0,
                                              const std::vector<cv::Point3f>& cam_pts_1,
                                              cv::Mat& f_descs_0,
                                              cv::Mat& f_descs_1,
                                              std::vector<int>& f_cluster_ids_0,
                                              std::vector<int>& f_cluster_ids_1,
                                              std::vector<cv::KeyPoint>& f_kpts_0,
                                              std::vector<cv::KeyPoint>& f_kpts_1,
                                              std::vector<cv::Point3f>& f_cam_pts_0,
                                              std::vector<cv::Point3f>& f_cam_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_cam_pts_0.clear();
    f_cam_pts_1.clear();
    if (descs_0.rows < 10 || descs_1.rows < 10)
        return;

    // If there are no clusters, the filtered cluster vectors must not be filled.
    bool fill_clusters;
    if (cluster_ids_0.size() > 0)
        fill_clusters = true;
    else
        fill_clusters = false;

    // Visual keypoint matching
    std::vector<std::vector<cv::DMatch>> matches = matching(descs_0, descs_1);

    int idx_0, idx_1;
    cv::Mat m_descs_0, m_descs_1;
    std::vector<cv::KeyPoint> m_kpts_0, m_kpts_1;
    std::vector<int> m_cluster_ids_0, m_cluster_ids_1;
    std::vector<cv::Point3f> m_cam_pts_0, m_cam_pts_1;
    for (uint i = 0; i < matches.size(); i++)
    {
        idx_0 = matches[i][0].queryIdx;
        idx_1 = matches[i][0].trainIdx;
        m_kpts_0.push_back(kpts_0[idx_0]);
        m_kpts_1.push_back(kpts_1[idx_1]);
        m_descs_0.push_back(descs_0.row(idx_0));
        m_descs_1.push_back(descs_1.row(idx_1));
        m_cam_pts_0.push_back(cam_pts_0[idx_0]);
        m_cam_pts_1.push_back(cam_pts_1[idx_1]);
        if (fill_clusters)
        {
            m_cluster_ids_0.push_back(cluster_ids_0[idx_0]);
            m_cluster_ids_1.push_back(cluster_ids_1[idx_1]);
        }
    }

    // Compute homography or essential matrix
    std::vector<cv::Point2f> m_pts_0, m_pts_1;
    cv::KeyPoint::convert(m_kpts_0, m_pts_0);
    cv::KeyPoint::convert(m_kpts_1, m_pts_1);
    cv::Mat M, mask, K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
    try
    {
        if (coplanar_pts)
            M = cv::findHomography(m_pts_0, m_pts_1, mask, cv::RANSAC, 1.0);
        else
            M = cv::findEssentialMat(m_pts_0, m_pts_1, K, cv::RANSAC, 0.999, 1.0, mask);
    }
    catch (cv::Exception &e)
    {
        const char* err_msg = e.what();
        std::cerr << "Exception caught: " << err_msg << std::endl;
        return;
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
            f_cam_pts_0.push_back(m_cam_pts_0[i]);
            f_cam_pts_1.push_back(m_cam_pts_1[i]);
            if (fill_clusters)
            {
                f_cluster_ids_0.push_back(m_cluster_ids_0[i]);
                f_cluster_ids_1.push_back(m_cluster_ids_1[i]);
            }
        }
    }
}


void VisionMotionModules::assembleCircularMatchings(const std::vector<cv::KeyPoint>& stereo_kpts_0_lowest,
                                                    const std::vector<cv::KeyPoint>& stereo_kpts_0_highest,
                                                    const std::vector<cv::KeyPoint>& stereo_kpts_1_lowest,
                                                    const std::vector<cv::KeyPoint>& stereo_kpts_1_highest,
                                                    const std::vector<cv::KeyPoint>& successive_kpts_0_lowest,
                                                    const std::vector<cv::KeyPoint>& successive_kpts_0_highest,
                                                    const std::vector<cv::KeyPoint>& successive_kpts_1_lowest,
                                                    const std::vector<cv::KeyPoint>& successive_kpts_1_highest,
                                                    const std::vector<cv::Point3f>& successive_cam_pts_0,
                                                    const std::vector<cv::Point3f>& successive_cam_pts_1,
                                                    const std::vector<int>& successive_cluster_ids_0,
                                                    const std::vector<int>& successive_cluster_ids_1,
                                                    std::vector<cv::KeyPoint>& f_kpts_0_lowest,
                                                    std::vector<cv::KeyPoint>& f_kpts_0_highest,
                                                    std::vector<cv::KeyPoint>& f_kpts_1_lowest,
                                                    std::vector<cv::KeyPoint>& f_kpts_1_highest,
                                                    std::vector<cv::Point3f>& f_cam_pts_0,
                                                    std::vector<cv::Point3f>& f_cam_pts_1,
                                                    std::vector<int>& f_cluster_ids_0,
                                                    std::vector<int>& f_cluster_ids_1)
{
    // If there are no clusters, the filtered clusters vectors must not be filled.
    bool fill_clusters;
    if (successive_cluster_ids_0.size() > 0)
        fill_clusters = true;
    else
        fill_clusters = false;

    // Assembles
    uint j;
    uint k;
    uint l;
    for (uint i = 0; i < successive_kpts_1_lowest.size(); i++)
    {
        for (j = 0; j < stereo_kpts_1_lowest.size(); j++)
        {
            if (successive_kpts_1_lowest[i].pt == stereo_kpts_1_lowest[j].pt)
            {
                for (k = 0; k < successive_kpts_1_highest.size(); k++)
                {
                    if (stereo_kpts_1_highest[j].pt == successive_kpts_1_highest[k].pt)
                    {
                        for (l = 0; l < stereo_kpts_0_highest.size(); l++)
                        {
                            if (successive_kpts_0_highest[k].pt == stereo_kpts_0_highest[l].pt)
                            {
                                if (stereo_kpts_0_lowest[l].pt == successive_kpts_0_lowest[i].pt)
                                {
                                    f_kpts_0_lowest.push_back(successive_kpts_0_lowest[i]);
                                    f_kpts_0_highest.push_back(successive_kpts_0_highest[k]);
                                    f_kpts_1_lowest.push_back(successive_kpts_1_lowest[i]);
                                    f_kpts_1_highest.push_back(successive_kpts_1_highest[k]);
                                    f_cam_pts_0.push_back(successive_cam_pts_0[i]);
                                    f_cam_pts_1.push_back(successive_cam_pts_1[i]);
                                    if (fill_clusters)
                                    {
                                        f_cluster_ids_0.push_back(successive_cluster_ids_0[i]);
                                        f_cluster_ids_1.push_back(successive_cluster_ids_1[i]);
                                    }
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
}