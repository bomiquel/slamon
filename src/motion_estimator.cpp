#include "motion_estimator.hpp"


MotionEstimator::MotionEstimator(std::string& motion_estimation_type)
{
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


MotionEstimator::MotionEstimator(const cv::Mat& l_proj_mat,
                                 const cv::Mat& r_proj_mat,
                                 std::string& motion_estimation_type)
{
    l_proj_mat_ = l_proj_mat;
    r_proj_mat_ = r_proj_mat;
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


MotionEstimator::MotionEstimator(const double b,
                                 const double f_x,
                                 const double f_y,
                                 const double c_x,
                                 const double c_y,
                                 std::string& motion_estimation_type)
{
    r_proj_mat_ = computeProjectionMatrix(b, f_x, f_x, c_x, c_y);
    l_proj_mat_ = computeProjectionMatrix(0.0, f_x, f_x, c_x, c_y);
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


void MotionEstimator::compute3DPoints(const cv::Mat& l_descs,
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
        if (std::isfinite(pt.x) &&
            std::isfinite(pt.y) &&
            std::isfinite(pt.z) &&
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


std::vector<std::vector<std::pair<int, cv::KeyPoint>>> MotionEstimator::bucketKeypoints(const uint max_kpts,
                                                                                        const float img_width,
                                                                                        const float img_height,
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
    float bucket_width = img_width / 10.0;
    float bucket_height = img_height / 10.0;

    // Fill buckets with keypoints and its indices.
    int bucket_idx, bucket_row, bucket_col;
    std::vector<std::vector<std::pair<int, cv::KeyPoint>>> buckets(100);
    for (uint i = 0; i < l_kpts_0.size(); i++)
    {
        bucket_col = (int) floor(l_kpts_0[i].pt.x / bucket_width);
        bucket_row = (int) floor(l_kpts_0[i].pt.y / bucket_height);
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
        if (buckets[i].size() > max_kpts)
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
            while (inliers.size() < max_kpts)
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
                        if (max_kpts > 1)
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


void MotionEstimator::estimateMotion(const std::vector<cv::KeyPoint>& l_kpts_0,
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


void MotionEstimator::estimateMotion(const std::vector<int>& cluster_ids_0,
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
    std::vector<cv::Point2f> l_pts_0, r_pts_0;
    cv::KeyPoint::convert(l_kpts_0, l_pts_0);
    cv::KeyPoint::convert(r_kpts_0, r_pts_0);
    if (motion_estimation_type_ == DUAL)
    {
        solveDualPnPRansac(2000, l_pts_0, r_pts_0, cam_pts_1, r_vec, t_vec, inliers);
    }
    else
    {
        cv::Mat K = l_proj_mat_.colRange(cv::Range(0, 3)).clone();
        cv::solvePnPRansac(cam_pts_1, l_pts_0, K, cv::Mat(), r_vec, t_vec, 
                           false, 2000, 2.0, 0.99, inliers, motion_estimation_type_);
    }

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


MotionEstimator::Type MotionEstimator::selectMotionEstimationType(std::string& motion_estimation_type)
{
    // Sanity check
    for (uint i = 0; i < motion_estimation_type.size(); i++)
    {
        if (isupper(motion_estimation_type[i]))
            motion_estimation_type[i] = std::tolower(motion_estimation_type[i]);
    }

    if (motion_estimation_type == "solvepnp_p3p" || motion_estimation_type == "p3p")
        return P3P;
    else if (motion_estimation_type == "solvepnp_ap3p" || motion_estimation_type == "ap3p")
        return AP3P;
    else if (motion_estimation_type == "solvepnp_epnp" || motion_estimation_type == "epnp")
        return EPNP;
    else if (motion_estimation_type == "solvepnp_dual" || motion_estimation_type == "dual")
        return DUAL;
    else
    {
        std::cerr << "Unforseen motion estimation type" << std::endl;
        return AP3P;
    }
}


cv::Mat MotionEstimator::computeProjectionMatrix(const double b,
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


void MotionEstimator::solveDualPnPRansac(const uint ransac_iter,
                                         const std::vector<cv::Point2f>& l_pts_0,
                                         const std::vector<cv::Point2f>& r_pts_0,
                                         const std::vector<cv::Point3f>& cam_pts_1,
                                         cv::Mat& r_vec,
                                         cv::Mat& t_vec,
                                         std::vector<int>& inliers)
{
    // Sanity check.
    inliers.clear();
    r_vec.release();
    t_vec.release();

    // Variable initialisation.
    bool success = true;
    int n = l_pts_0.size();
    if (n < 6)
        return;

    // Matrix initialisation.
    std::vector<double> tr_vec(6);
    std::vector<double> c_tr_vec(6);
    std::vector<double> J(4 * n * 6); // Jacobians.
    std::vector<double> obs_pts(4 * n); // Observed 2d points. Current/query left/right keypoints.
    std::vector<double> pre_pts(4 * n); // Predicted 2d points. Reprojected previous/candidates left/right keypoints.
    std::vector<double> res_pts(4 * n); // Residuals. (res_pts = obs_pts - pre_pts).

    // RANSAC
    int iter;
    DualResult result;
    std::vector<int> c_idxs;
    std::vector<int> c_inliers;
    for (uint i = 0; i < ransac_iter; i++)
    {
        // Gets random active indices.
        c_idxs = getRandomIndices(n, 3);

        // Clears current parameters.
        for (uint j = 0; j < 3; j++)
            c_tr_vec[j] = 0;

        // Minimizes reprojection errors.
        iter = 0;
        result = UPDATED;
        while (result == UPDATED)
        {
            result = updateDualParams(1e-6, 1, c_idxs, l_pts_0, r_pts_0, cam_pts_1, J, c_tr_vec, obs_pts, pre_pts, res_pts);
            if (iter++ > 20 || result == CONVERGED)
                break;
        }

        // Overwrite best parameters if there are more inliers.
        if (result != FAILED)
        {
            c_inliers = getInliers(l_pts_0, r_pts_0, cam_pts_1, J, c_tr_vec, obs_pts, pre_pts, res_pts);
            if (c_inliers.size() > inliers.size())
            {
                tr_vec = c_tr_vec;
                inliers = c_inliers;
            }
        }
    }

    // Final optimization (refinement)
    iter = 0;
    result = UPDATED;
    if (inliers.size() > 6)
    {
        while (result == UPDATED)
        {
            result = updateDualParams(1e-8, 1, inliers, l_pts_0, r_pts_0, cam_pts_1, J, tr_vec, obs_pts, pre_pts, res_pts);
            if (iter++ > 100 || result == CONVERGED)
                break;
        }

        if (result != CONVERGED)
            success = false;
    }
    else
    {
        success = false;
    }

    // Parameter estimate succeeded?
    if (success)
    {
        r_vec = cv::Mat(std::vector<double>{tr_vec[0], tr_vec[1], tr_vec[2]}, CV_32FC1);
        t_vec = cv::Mat(std::vector<double>{tr_vec[3], tr_vec[4], tr_vec[5]}, CV_32FC1);
    }
}


std::vector<int> MotionEstimator::getRandomIndices(uint size, uint n_rand_idxs)
{
    // Inits random and total vector indices.
    std::vector<int> rand_idxs;
    std::vector<int> total_idxs;

    // Creates vector containing all indices.
    for (uint i = 0; i < size; i++)
        total_idxs.push_back(i);

    // Add indices to random indices vector.
    uint idx;
    rand_idxs.clear();
    for (uint i = 0; i < n_rand_idxs; i++)
    {
        idx = rand() % total_idxs.size();
        rand_idxs.push_back(total_idxs[idx]);
        total_idxs.erase(total_idxs.begin() + idx);
    }

    // Return random indices.
    return rand_idxs;
}


MotionEstimator::DualResult MotionEstimator::updateDualParams(const double eps,
                                                              const double step_size,
                                                              const std::vector<int>& c_idxs,
                                                              const std::vector<cv::Point2f>& l_pts_0,
                                                              const std::vector<cv::Point2f>& r_pts_0,
                                                              const std::vector<cv::Point3f>& cam_pts_1,
                                                              std::vector<double>& J,
                                                              std::vector<double>& tr,
                                                              std::vector<double>& obs_pts,
                                                              std::vector<double>& pre_pts,
                                                              std::vector<double>& res_pts)
{
    // At least 3 observations are needed.
    if (c_idxs.size() < 3)
        return FAILED;

    // Computes observations, predictions, residuals and Jacobians.
    computeVectorsAndJacobians(c_idxs,
                               l_pts_0,
                               r_pts_0,
                               cam_pts_1,
                               J,
                               tr,
                               obs_pts,
                               pre_pts,
                               res_pts);

    // Init.
    cv::Mat A(cv::Size(6, 6), CV_64F);
    cv::Mat B(cv::Size(1, 6), CV_64F);

    // Fills A and B matrices.
    double a = 0, b = 0;
    for (int m = 0; m < 6; m++)
    {
        for (int n = 0; n < 6; n++)
        {
            a = 0;
            for (int i = 0; i < 4 * (int)c_idxs.size(); i++)
                a += J[i * 6 + m] * J[i * 6 + n];
            A.at<double>(m, n) = a;
        }
            
        b = 0;
        for (int i = 0; i < 4 * (int)c_idxs.size(); i++)
            b += J[i * 6 + m] * (res_pts[i]);
        B.at<double>(m, 0) = b;
    }

    // Performs elimination.
    cv::Mat coeff;
    bool converged = true;
    if (cv::solve(A, B, coeff, cv::DECOMP_LU))
    {
        for (int m = 0; m < 6; m++)
        {
            tr[m] += step_size * coeff.at<double>(m, 0);
            if(fabs(coeff.at<double>(m, 0) > eps))
                converged = false; 
        }
        if (converged)
            return CONVERGED;
        else
            return UPDATED;
    }
    else
    {
        return FAILED;
    }
}


void MotionEstimator::computeVectorsAndJacobians(const std::vector<int>& c_idxs,
                                                 const std::vector<cv::Point2f>& l_pts_0,
                                                 const std::vector<cv::Point2f>& r_pts_0,
                                                 const std::vector<cv::Point3f>& cam_pts_1,
                                                 std::vector<double>& J,
                                                 std::vector<double>& tr,
                                                 std::vector<double>& obs_pts,
                                                 std::vector<double>& pre_pts,
                                                 std::vector<double>& res_pts)
{
    // Extracts motion parameters.
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

    // Precomputes sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

    // Computes rotation matrix and derivatives
    double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
    double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
    double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
    double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
    double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
    double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
    double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
    double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
    double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
    double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
    double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

    // Computes baseline.
    double f_u = l_proj_mat_.at<double>(0, 0);
    double t_x = r_proj_mat_.at<double>(0, 3);
    double b = (t_x / -f_u);

    cv::Point3f cam_pt_1; // Previous/candidate camera point relative to previous/candidate left camera.
    cv::Point3f cam_pt_1_tr_0; // Previous/candidate camera point relative to current/query left camera.
    cv::Point3f cam_pt_1_tr_0_r; // Previous/candidate camera point relative to current/query right camera.
    cv::Point3f cam_pt_1_tr_0_d; // Derived previous/candidate camera point relative to current/query left camera.

    // For all observations do
    for (uint i = 0; i < c_idxs.size(); i++)
    {
        // Sets ith observation
        obs_pts[4 * i + 0] = l_pts_0[c_idxs[i]].x;
        obs_pts[4 * i + 1] = l_pts_0[c_idxs[i]].y;
        obs_pts[4 * i + 2] = r_pts_0[c_idxs[i]].x;
        obs_pts[4 * i + 3] = r_pts_0[c_idxs[i]].y;

        // Gets ith previous/candidate 3d camera point (respect left image).
        cam_pt_1 = cam_pts_1[c_idxs[i]];

        // Transforms to current/query left coordinate system.
        cam_pt_1_tr_0.x = r00 * cam_pt_1.x + r01 * cam_pt_1.y + r02 * cam_pt_1.z + tx;
        cam_pt_1_tr_0.y = r10 * cam_pt_1.x + r11 * cam_pt_1.y + r12 * cam_pt_1.z + ty;
        cam_pt_1_tr_0.z = r20 * cam_pt_1.x + r21 * cam_pt_1.y + r22 * cam_pt_1.z + tz;

        // Computes 3d point in current/query right coordinate system
        cam_pt_1_tr_0_r.x = cam_pt_1_tr_0.x - b;

        // For all paramters do
        for (int32_t j=0; j<6; j++) 
        {
            // derivatives of 3d pt. in curr. left coordinates wrt. param j
            switch (j) 
            {
                case 0: cam_pt_1_tr_0_d.x = 0;
                        cam_pt_1_tr_0_d.y = rdrx10 * cam_pt_1.x + rdrx11 * cam_pt_1.y + rdrx12 * cam_pt_1.z;
                        cam_pt_1_tr_0_d.z = rdrx20 * cam_pt_1.x + rdrx21 * cam_pt_1.y + rdrx22 * cam_pt_1.z;
                        break;
                case 1: cam_pt_1_tr_0_d.x = rdry00 * cam_pt_1.x + rdry01 * cam_pt_1.y + rdry02 * cam_pt_1.z;
                        cam_pt_1_tr_0_d.y = rdry10 * cam_pt_1.x + rdry11 * cam_pt_1.y + rdry12 * cam_pt_1.z;
                        cam_pt_1_tr_0_d.z = rdry20 * cam_pt_1.x + rdry21 * cam_pt_1.y + rdry22 * cam_pt_1.z;
                        break;
                case 2: cam_pt_1_tr_0_d.x = rdrz00 * cam_pt_1.x + rdrz01 * cam_pt_1.y;
                        cam_pt_1_tr_0_d.y = rdrz10 * cam_pt_1.x + rdrz11 * cam_pt_1.y;
                        cam_pt_1_tr_0_d.z = rdrz20 * cam_pt_1.x + rdrz21 * cam_pt_1.y;
                        break;
                case 3: cam_pt_1_tr_0_d.x = 1; cam_pt_1_tr_0_d.y = 0; cam_pt_1_tr_0_d.z = 0; break;
                case 4: cam_pt_1_tr_0_d.x = 0; cam_pt_1_tr_0_d.y = 1; cam_pt_1_tr_0_d.z = 0; break;
                case 5: cam_pt_1_tr_0_d.x = 0; cam_pt_1_tr_0_d.y = 0; cam_pt_1_tr_0_d.z = 1; break;
            }

            // Sets jacobian entries (project via K)
            J[(4*i+0)*6+j] = l_proj_mat_.at<double>(0, 0) * (cam_pt_1_tr_0_d.x * cam_pt_1_tr_0.z - cam_pt_1_tr_0.x * cam_pt_1_tr_0_d.z) / (cam_pt_1_tr_0.z * cam_pt_1_tr_0.z); // left u'
            J[(4*i+1)*6+j] = l_proj_mat_.at<double>(0, 0) * (cam_pt_1_tr_0_d.y * cam_pt_1_tr_0.z - cam_pt_1_tr_0.y * cam_pt_1_tr_0_d.z) / (cam_pt_1_tr_0.z * cam_pt_1_tr_0.z); // left v'
            J[(4*i+2)*6+j] = l_proj_mat_.at<double>(0, 0) * (cam_pt_1_tr_0_d.x * cam_pt_1_tr_0.z - cam_pt_1_tr_0_r.x * cam_pt_1_tr_0_d.z) / (cam_pt_1_tr_0.z * cam_pt_1_tr_0.z); // right u'
            J[(4*i+3)*6+j] = l_proj_mat_.at<double>(0, 0) * (cam_pt_1_tr_0_d.y * cam_pt_1_tr_0.z - cam_pt_1_tr_0.y * cam_pt_1_tr_0_d.z) / (cam_pt_1_tr_0.z * cam_pt_1_tr_0.z); // right v'
        }

        // Sets prediction (project via K)
        pre_pts[4*i+0] = l_proj_mat_.at<double>(0, 0) * cam_pt_1_tr_0.x / cam_pt_1_tr_0.z + l_proj_mat_.at<double>(0, 2); // left u
        pre_pts[4*i+1] = l_proj_mat_.at<double>(0, 0) * cam_pt_1_tr_0.y / cam_pt_1_tr_0.z + l_proj_mat_.at<double>(1, 2); // left v
        pre_pts[4*i+2] = l_proj_mat_.at<double>(0, 0) * cam_pt_1_tr_0_r.x / cam_pt_1_tr_0.z + l_proj_mat_.at<double>(0, 2); // right u
        pre_pts[4*i+3] = l_proj_mat_.at<double>(0, 0) * cam_pt_1_tr_0.y / cam_pt_1_tr_0.z + l_proj_mat_.at<double>(1, 2); // right v

        // Sets residuals
        res_pts[4*i+0] = (obs_pts[4 * i + 0] - pre_pts[4 * i + 0]);
        res_pts[4*i+1] = (obs_pts[4 * i + 1] - pre_pts[4 * i + 1]);
        res_pts[4*i+2] = (obs_pts[4 * i + 2] - pre_pts[4 * i + 2]);
        res_pts[4*i+3] = (obs_pts[4 * i + 3] - pre_pts[4 * i + 3]);
    }
}


std::vector<int> MotionEstimator::getInliers(const std::vector<cv::Point2f>& l_pts_0,
                                             const std::vector<cv::Point2f>& r_pts_0,
                                             const std::vector<cv::Point3f>& cam_pts_1,
                                             std::vector<double>& J,
                                             std::vector<double>& tr,
                                             std::vector<double>& obs_pts,
                                             std::vector<double>& pre_pts,
                                             std::vector<double>& res_pts)
{
    // Marks all observations active
    std::vector<int> c_idxs;
    for (uint i = 0; i < l_pts_0.size(); i++)
        c_idxs.push_back(i);

    // Computes observations, predictions, residuals and Jacobians.
    computeVectorsAndJacobians(c_idxs,
                               l_pts_0,
                               r_pts_0,
                               cam_pts_1,
                               J,
                               tr,
                               obs_pts,
                               pre_pts,
                               res_pts);

    // Computes inliers.
    std::vector<int> inliers;
    for (uint i = 0; i < l_pts_0.size(); i++)
    {
        if ((pow(obs_pts[4 * i + 0] - pre_pts[4 * i + 0], 2) + pow(obs_pts[4 * i + 1] - pre_pts[4 * i + 1], 2) +
             pow(obs_pts[4 * i + 2] - pre_pts[4 * i + 2], 2) + pow(obs_pts[4 * i + 3] - pre_pts[4 * i + 3], 2)) < (2.0 * 2.0))
            inliers.push_back(i);
    }

    return inliers;
}