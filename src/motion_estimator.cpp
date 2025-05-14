#include "motion_estimator.hpp"


MotionEstimator::MotionEstimator(std::string& motion_estimation_type)
{
    motion_estimation_type_ = selectMotionEstimationType(motion_estimation_type);
}


cv::Mat MotionEstimator::buildProjectionMatrix(const double b,
                                               const double f_x, 
                                               const double f_y, 
                                               const double c_x, 
                                               const double c_y)
{
    // Intrinsic camera matrix
    cv::Mat intrinsic_mat = (cv::Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);

    // Rotation matrix (identity for simplicity, replace with actual rotation)
    cv::Mat rotation_mat = cv::Mat::eye(3, 3, CV_64F);

    // Translation vector (replace with actual translation)
    cv::Mat trans_vec = (cv::Mat_<double>(3, 1) << b, 0.0, 0.0);

    // Combine rotation and translation into a 3x4 extrinsic matrix [R | t]
    cv::Mat extrinsic_mat;
    cv::hconcat(rotation_mat, trans_vec, extrinsic_mat);

    // Create the 3x4 projection matrix P = K * [R | t]
    cv::Mat proj_mat = intrinsic_mat * extrinsic_mat;    

    return proj_mat;
}


void MotionEstimator::compute3DPoints(const cv::Mat& l_proj_mat,
                                      const cv::Mat& r_proj_mat,
                                      const cv::Mat& l_descs,
                                      const cv::Mat& r_descs,
                                      const std::vector<cv::KeyPoint>& l_kpts,
                                      const std::vector<cv::KeyPoint>& r_kpts,
                                      cv::Mat& f_l_descs,
                                      cv::Mat& f_r_descs,
                                      std::vector<cv::KeyPoint>& f_l_kpts,
                                      std::vector<cv::KeyPoint>& f_r_kpts,
                                      std::vector<cv::Point3f>& f_l_cam_pts)
{
    // Triangulates keypoints.
    cv::Mat pts_4d, l_cam_pts;
    std::vector<cv::Point2f> l_pts, r_pts;
    cv::KeyPoint::convert(l_kpts, l_pts);
    cv::KeyPoint::convert(r_kpts, r_pts);
    cv::triangulatePoints(l_proj_mat, r_proj_mat, l_pts, r_pts, pts_4d);
    cv::convertPointsFromHomogeneous(pts_4d.t(), l_cam_pts);
    l_cam_pts = l_cam_pts.reshape(1, l_cam_pts.rows).clone();

    // Computes baseline.
    double f_u = l_proj_mat.at<double>(0, 0);
    double t_x = r_proj_mat.at<double>(0, 3);
    double b = (t_x / -f_u);

    // Filters "far" keypoints.
    cv::Point3f l_cam_pt;
    double z_limit = 40 * b;
    for (int i = 0; i < l_cam_pts.rows; i++)
    {
        l_cam_pt.x = l_cam_pts.at<float>(i, 0);
        l_cam_pt.y = l_cam_pts.at<float>(i, 1);
        l_cam_pt.z = l_cam_pts.at<float>(i, 2);
        if (std::isfinite(l_cam_pt.x) &&
            std::isfinite(l_cam_pt.y) &&
            std::isfinite(l_cam_pt.z) &&
            l_cam_pt.z > 0.0 &&
            l_cam_pt.z <= z_limit)
        {
            f_l_kpts.push_back(l_kpts[i]);
            f_r_kpts.push_back(r_kpts[i]);
            f_l_cam_pts.push_back(l_cam_pt);
            f_l_descs.push_back(l_descs.row(i));
            f_r_descs.push_back(r_descs.row(i));
        }
    }
}


std::vector<std::vector<std::pair<int, cv::KeyPoint>>> MotionEstimator::bucketKeypoints(const float img_width,
                                                                                        const float img_height,
                                                                                        const unsigned int max_kpts,
                                                                                        const std::vector<cv::KeyPoint>& c_l_kpts,
                                                                                        const std::vector<cv::KeyPoint>& c_r_kpts,
                                                                                        const std::vector<cv::KeyPoint>& p_l_kpts,
                                                                                        const std::vector<cv::KeyPoint>& p_r_kpts,
                                                                                        const std::vector<cv::Point3f>& c_l_cam_pts,
                                                                                        const std::vector<cv::Point3f>& p_l_cam_pts,
                                                                                        std::vector<cv::KeyPoint>& f_c_l_kpts,
                                                                                        std::vector<cv::KeyPoint>& f_c_r_kpts,
                                                                                        std::vector<cv::KeyPoint>& f_p_l_kpts,
                                                                                        std::vector<cv::KeyPoint>& f_p_r_kpts,
                                                                                        std::vector<cv::Point3f>& f_c_l_cam_pts,
                                                                                        std::vector<cv::Point3f>& f_p_l_cam_pts)
{
    // Sanity check.
    f_c_l_kpts.clear();
    f_c_r_kpts.clear();
    f_p_l_kpts.clear();
    f_p_r_kpts.clear();
    f_c_l_cam_pts.clear();
    f_p_l_cam_pts.clear();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    bool fill_right;
    if (c_r_kpts.size() > 0)
        fill_right = true;
    else
        fill_right = false;

    // Calculate the number of rows and columns that each bucket has.
    float bucket_width = img_width / 10.0;
    float bucket_height = img_height / 10.0;

    // Fill buckets with keypoints and its indices.
    int bucket_idx, bucket_row, bucket_col;
    std::vector<std::vector<std::pair<int, cv::KeyPoint>>> buckets(100);
    for (unsigned int i = 0; i < c_l_kpts.size(); i++)
    {
        bucket_col = (int) floor(c_l_kpts[i].pt.x / bucket_width);
        bucket_row = (int) floor(c_l_kpts[i].pt.y / bucket_height);
        bucket_idx = (int) (10 * bucket_col + bucket_row);
        buckets[bucket_idx].push_back(std::make_pair(i, c_l_kpts[i]));
    }

    // Remove excess keypoints from the buckets.
    int idx;
    float slope, delta_x;
    cv::KeyPoint l_kpt_0, l_kpt_1;
    std::vector<std::pair<float, std::pair<int, cv::KeyPoint>>> bucket_slopes;
    for (unsigned int i = 0; i < buckets.size(); i++)
    {
        if (buckets[i].size() > max_kpts)
        {
            // Calculate the coincidence slopes.
            bucket_slopes.clear();
            for (unsigned int j = 0; j < buckets[i].size(); j++)
            {
                idx = buckets[i][j].first;
                l_kpt_0 = c_l_kpts[idx];
                l_kpt_1 = p_l_kpts[idx];
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
            unsigned int idx_forwards = half_longitude + 1;
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

        for (unsigned int j = 0; j < buckets[i].size(); j++)
        {
            idx = buckets[i][j].first;
            f_c_l_kpts.push_back(c_l_kpts[idx]);
            f_p_l_kpts.push_back(p_l_kpts[idx]);
            f_c_l_cam_pts.push_back(c_l_cam_pts[idx]);
            f_p_l_cam_pts.push_back(p_l_cam_pts[idx]);

            if (fill_right)
            {
                f_c_r_kpts.push_back(c_r_kpts[idx]);
                f_p_r_kpts.push_back(p_r_kpts[idx]);
            }
        }
    }

    return buckets;
}


void MotionEstimator::estimateMotion(const cv::Mat& l_proj_mat,
                                     const cv::Mat& r_proj_mat,
                                     const std::vector<cv::KeyPoint>& l_kpts_0,
                                     const std::vector<cv::KeyPoint>& r_kpts_0,
                                     const std::vector<cv::KeyPoint>& l_kpts_1,
                                     const std::vector<cv::KeyPoint>& r_kpts_1,
                                     const std::vector<cv::Point3f>& scene_pts_0,
                                     const std::vector<cv::Point3f>& scene_pts_1,
                                     cv::Mat& r_vec,
                                     cv::Mat& t_vec,
                                     std::vector<cv::KeyPoint>& f_l_kpts_0,
                                     std::vector<cv::KeyPoint>& f_r_kpts_0,
                                     std::vector<cv::KeyPoint>& f_l_kpts_1,
                                     std::vector<cv::KeyPoint>& f_r_kpts_1,
                                     std::vector<cv::Point3f>& f_scene_pts_0,
                                     std::vector<cv::Point3f>& f_scene_pts_1)
{
    // Dummy variables.
    std::vector<int> cluster_ids_0, cluster_ids_1, f_cluster_ids_0, f_cluster_ids_1;
    estimateMotion(l_proj_mat, r_proj_mat,
                   cluster_ids_0, cluster_ids_1, 
                   l_kpts_0, r_kpts_0, 
                   l_kpts_1, r_kpts_1, 
                   scene_pts_0, scene_pts_1,
                   r_vec, t_vec, 
                   f_cluster_ids_0, f_cluster_ids_1, 
                   f_l_kpts_0, f_r_kpts_0, 
                   f_l_kpts_1, f_r_kpts_1, 
                   f_scene_pts_0, f_scene_pts_1);
}


void MotionEstimator::estimateMotion(const cv::Mat& l_proj_mat,
                                     const cv::Mat& r_proj_mat,
                                     const std::vector<int>& cluster_ids_0,
                                     const std::vector<int>& cluster_ids_1,
                                     const std::vector<cv::KeyPoint>& l_kpts_0,
                                     const std::vector<cv::KeyPoint>& r_kpts_0,
                                     const std::vector<cv::KeyPoint>& l_kpts_1,
                                     const std::vector<cv::KeyPoint>& r_kpts_1,
                                     const std::vector<cv::Point3f>& scene_pts_0,
                                     const std::vector<cv::Point3f>& scene_pts_1,
                                     cv::Mat& r_vec,
                                     cv::Mat& t_vec,
                                     std::vector<int>& f_cluster_ids_0,
                                     std::vector<int>& f_cluster_ids_1,        
                                     std::vector<cv::KeyPoint>& f_l_kpts_0,
                                     std::vector<cv::KeyPoint>& f_r_kpts_0,
                                     std::vector<cv::KeyPoint>& f_l_kpts_1,
                                     std::vector<cv::KeyPoint>& f_r_kpts_1,
                                     std::vector<cv::Point3f>& f_scene_pts_0,
                                     std::vector<cv::Point3f>& f_scene_pts_1)
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
        solveDualPnPRansac(2000, l_proj_mat, r_proj_mat, l_pts_0, r_pts_0, scene_pts_1, r_vec, t_vec, inliers);
    }
    else
    {
        cv::Mat K = l_proj_mat.colRange(cv::Range(0, 3)).clone();
        cv::solvePnPRansac(scene_pts_1, l_pts_0, K, cv::Mat(), r_vec, t_vec, 
                           false, 2000, 1.0, 0.99, inliers, motion_estimation_type_);
    }

    // Filter keypoints, 3D points and clusters.
    for (unsigned int i = 0; i < inliers.size(); i++)
    {
        f_l_kpts_0.push_back(l_kpts_0[inliers[i]]);
        f_l_kpts_1.push_back(l_kpts_1[inliers[i]]);
        f_scene_pts_0.push_back(scene_pts_0[inliers[i]]);
        f_scene_pts_1.push_back(scene_pts_1[inliers[i]]);
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
    for (unsigned int i = 0; i < motion_estimation_type.size(); i++)
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


void MotionEstimator::solveDualPnPRansac(const int ransac_iter,
                                         const cv::Mat& l_proj_mat,
                                         const cv::Mat& r_proj_mat,
                                         const std::vector<cv::Point2f>& l_pts_0,
                                         const std::vector<cv::Point2f>& r_pts_0,
                                         const std::vector<cv::Point3f>& scene_pts_1,
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
    for (int i = 0; i < ransac_iter; i++)
    {
        // Gets random active indices.
        c_idxs = getRandomIndices(n, 3);

        // Clears current parameters.
        for (unsigned int j = 0; j < c_tr_vec.size(); j++)
            c_tr_vec[j] = 0;

        // Minimizes reprojection errors.
        iter = 0;
        result = UPDATED;
        while (result == UPDATED)
        {
            result = updateDualParams(1e-6, 1, l_proj_mat, r_proj_mat, 
                                      c_idxs, l_pts_0, r_pts_0, scene_pts_1, 
                                      J, c_tr_vec, obs_pts, pre_pts, res_pts);
            if (iter++ > 20 || result == CONVERGED)
                break;
        }

        // Overwrite best parameters if there are more inliers.
        if (result != FAILED)
        {
            c_inliers = getInliers(2.0, l_proj_mat, r_proj_mat, 
                                   l_pts_0, r_pts_0, scene_pts_1, 
                                   J, c_tr_vec, obs_pts, pre_pts, res_pts);
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
            result = updateDualParams(1e-7, 1, l_proj_mat, r_proj_mat, 
                                      inliers, l_pts_0, r_pts_0, scene_pts_1, 
                                      J, tr_vec, obs_pts, pre_pts, res_pts);
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


std::vector<int> MotionEstimator::getRandomIndices(unsigned int size, unsigned int n_rand_idxs)
{
    // Inits random and total vector indices.
    std::vector<int> rand_idxs;
    std::vector<int> total_idxs;

    // Creates vector containing all indices.
    for (unsigned int i = 0; i < size; i++)
        total_idxs.push_back(i);

    // Add indices to random indices vector.
    unsigned int idx;
    rand_idxs.clear();
    for (unsigned int i = 0; i < n_rand_idxs; i++)
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
                                                              const cv::Mat& l_proj_mat,
                                                              const cv::Mat& r_proj_mat,
                                                              const std::vector<int>& c_idxs,
                                                              const std::vector<cv::Point2f>& l_pts_0,
                                                              const std::vector<cv::Point2f>& r_pts_0,
                                                              const std::vector<cv::Point3f>& scene_pts_1,
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
    computeVectorsAndJacobians(l_proj_mat,
                               r_proj_mat,
                               c_idxs,
                               l_pts_0,
                               r_pts_0,
                               scene_pts_1,
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
        B.at<double>(0, m) = b;
    }

    // Performs elimination.
    cv::Mat coeff;
    bool converged = true;
    if (cv::solve(A, B, coeff, cv::DECOMP_LU))
    {
        for (int m = 0; m < 6; m++)
        {
            tr[m] += step_size * coeff.at<double>(0, m);
            if(fabs(coeff.at<double>(0, m) > eps))
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


void MotionEstimator::computeVectorsAndJacobians(const cv::Mat& l_proj_mat,
                                                 const cv::Mat& r_proj_mat,
                                                 const std::vector<int>& c_idxs,
                                                 const std::vector<cv::Point2f>& l_pts_0,
                                                 const std::vector<cv::Point2f>& r_pts_0,
                                                 const std::vector<cv::Point3f>& scene_pts_1,
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

    cv::Point3f l_scene_pt_1; // Previous/candidate 3D point relative to previous left camera/world.
    cv::Point3f l_scene_pt_1_tr_0; // Previous/candidate 3D point relative to current/query left camera.
    cv::Point3f r_scene_pt_1_tr_0; // Previous/candidate 3D point relative to current/query right camera.
    cv::Point3f l_scene_pt_1_tr_0_d; // Derived previous/candidate 3D point relative current/query left camera.

    // Computes baseline.
    double f_u = l_proj_mat.at<double>(0, 0);
    double t_x = r_proj_mat.at<double>(0, 3);
    double b = (t_x / -f_u);

    // For all observations do
    for (unsigned int i = 0; i < c_idxs.size(); i++)
    {
        // Sets ith observation
        obs_pts[4 * i + 0] = l_pts_0[c_idxs[i]].x;
        obs_pts[4 * i + 1] = l_pts_0[c_idxs[i]].y;
        obs_pts[4 * i + 2] = r_pts_0[c_idxs[i]].x;
        obs_pts[4 * i + 3] = r_pts_0[c_idxs[i]].y;

        // Gets ith previous/candidate 3d camera point (respect left image).
        l_scene_pt_1 = scene_pts_1[c_idxs[i]];

        // Transforms to current/query left coordinate system.
        l_scene_pt_1_tr_0.x = r00 * l_scene_pt_1.x + r01 * l_scene_pt_1.y + r02 * l_scene_pt_1.z + tx;
        l_scene_pt_1_tr_0.y = r10 * l_scene_pt_1.x + r11 * l_scene_pt_1.y + r12 * l_scene_pt_1.z + ty;
        l_scene_pt_1_tr_0.z = r20 * l_scene_pt_1.x + r21 * l_scene_pt_1.y + r22 * l_scene_pt_1.z + tz;

        // Computes 3d point in current/query right coordinate system
        r_scene_pt_1_tr_0 = l_scene_pt_1_tr_0;
        r_scene_pt_1_tr_0.x = l_scene_pt_1_tr_0.x - b;

        // For all paramters do
        for (int32_t j=0; j<6; j++) 
        {
            // derivatives of 3d pt. in curr. left coordinates wrt. param j
            switch (j) 
            {
                case 0: l_scene_pt_1_tr_0_d.x = 0;
                        l_scene_pt_1_tr_0_d.y = rdrx10 * l_scene_pt_1.x + rdrx11 * l_scene_pt_1.y + rdrx12 * l_scene_pt_1.z;
                        l_scene_pt_1_tr_0_d.z = rdrx20 * l_scene_pt_1.x + rdrx21 * l_scene_pt_1.y + rdrx22 * l_scene_pt_1.z;
                        break;
                case 1: l_scene_pt_1_tr_0_d.x = rdry00 * l_scene_pt_1.x + rdry01 * l_scene_pt_1.y + rdry02 * l_scene_pt_1.z;
                        l_scene_pt_1_tr_0_d.y = rdry10 * l_scene_pt_1.x + rdry11 * l_scene_pt_1.y + rdry12 * l_scene_pt_1.z;
                        l_scene_pt_1_tr_0_d.z = rdry20 * l_scene_pt_1.x + rdry21 * l_scene_pt_1.y + rdry22 * l_scene_pt_1.z;
                        break;
                case 2: l_scene_pt_1_tr_0_d.x = rdrz00 * l_scene_pt_1.x + rdrz01 * l_scene_pt_1.y;
                        l_scene_pt_1_tr_0_d.y = rdrz10 * l_scene_pt_1.x + rdrz11 * l_scene_pt_1.y;
                        l_scene_pt_1_tr_0_d.z = rdrz20 * l_scene_pt_1.x + rdrz21 * l_scene_pt_1.y;
                        break;
                case 3: l_scene_pt_1_tr_0_d.x = 1; l_scene_pt_1_tr_0_d.y = 0; l_scene_pt_1_tr_0_d.z = 0; break;
                case 4: l_scene_pt_1_tr_0_d.x = 0; l_scene_pt_1_tr_0_d.y = 1; l_scene_pt_1_tr_0_d.z = 0; break;
                case 5: l_scene_pt_1_tr_0_d.x = 0; l_scene_pt_1_tr_0_d.y = 0; l_scene_pt_1_tr_0_d.z = 1; break;
            }

            // Sets jacobian entries (project via K)
            J[(4*i+0)*6+j] = l_proj_mat.at<double>(0, 0) * (l_scene_pt_1_tr_0_d.x * l_scene_pt_1_tr_0.z - l_scene_pt_1_tr_0.x * l_scene_pt_1_tr_0_d.z) / (l_scene_pt_1_tr_0.z * l_scene_pt_1_tr_0.z); // left u'
            J[(4*i+1)*6+j] = l_proj_mat.at<double>(0, 0) * (l_scene_pt_1_tr_0_d.y * l_scene_pt_1_tr_0.z - l_scene_pt_1_tr_0.y * l_scene_pt_1_tr_0_d.z) / (l_scene_pt_1_tr_0.z * l_scene_pt_1_tr_0.z); // left v'
            J[(4*i+2)*6+j] = l_proj_mat.at<double>(0, 0) * (l_scene_pt_1_tr_0_d.x * l_scene_pt_1_tr_0.z - r_scene_pt_1_tr_0.x * l_scene_pt_1_tr_0_d.z) / (l_scene_pt_1_tr_0.z * l_scene_pt_1_tr_0.z); // right u'
            J[(4*i+3)*6+j] = l_proj_mat.at<double>(0, 0) * (l_scene_pt_1_tr_0_d.y * l_scene_pt_1_tr_0.z - l_scene_pt_1_tr_0.y * l_scene_pt_1_tr_0_d.z) / (l_scene_pt_1_tr_0.z * l_scene_pt_1_tr_0.z); // right v'
        }

        // Sets prediction (project via K)
        pre_pts[4*i+0] = l_proj_mat.at<double>(0, 0) * l_scene_pt_1_tr_0.x / l_scene_pt_1_tr_0.z + l_proj_mat.at<double>(0, 2); // left u
        pre_pts[4*i+1] = l_proj_mat.at<double>(0, 0) * l_scene_pt_1_tr_0.y / l_scene_pt_1_tr_0.z + l_proj_mat.at<double>(1, 2); // left v
        pre_pts[4*i+2] = l_proj_mat.at<double>(0, 0) * r_scene_pt_1_tr_0.x / l_scene_pt_1_tr_0.z + l_proj_mat.at<double>(0, 2); // right u
        pre_pts[4*i+3] = l_proj_mat.at<double>(0, 0) * l_scene_pt_1_tr_0.y / l_scene_pt_1_tr_0.z + l_proj_mat.at<double>(1, 2); // right v

        // Sets residuals
        res_pts[4*i+0] = (obs_pts[4 * i + 0] - pre_pts[4 * i + 0]);
        res_pts[4*i+1] = (obs_pts[4 * i + 1] - pre_pts[4 * i + 1]);
        res_pts[4*i+2] = (obs_pts[4 * i + 2] - pre_pts[4 * i + 2]);
        res_pts[4*i+3] = (obs_pts[4 * i + 3] - pre_pts[4 * i + 3]);
    }
}


std::vector<int> MotionEstimator::getInliers(double reproj_error,
                                             const cv::Mat& l_proj_mat,
                                             const cv::Mat& r_proj_mat,
                                             const std::vector<cv::Point2f>& l_pts_0,
                                             const std::vector<cv::Point2f>& r_pts_0,
                                             const std::vector<cv::Point3f>& scene_pts_1,
                                             std::vector<double>& J,
                                             std::vector<double>& tr,
                                             std::vector<double>& obs_pts,
                                             std::vector<double>& pre_pts,
                                             std::vector<double>& res_pts)
{
    // Marks all observations active
    std::vector<int> c_idxs;
    for (unsigned int i = 0; i < l_pts_0.size(); i++)
        c_idxs.push_back(i);

    // Computes observations, predictions, residuals and Jacobians.
    computeVectorsAndJacobians(l_proj_mat,
                               r_proj_mat,
                               c_idxs,
                               l_pts_0,
                               r_pts_0,
                               scene_pts_1,
                               J,
                               tr,
                               obs_pts,
                               pre_pts,
                               res_pts);

    // Computes inliers.
    std::vector<int> inliers;
    for (unsigned int i = 0; i < l_pts_0.size(); i++)
    {
        if ((pow(obs_pts[4 * i + 0] - pre_pts[4 * i + 0], 2) + pow(obs_pts[4 * i + 1] - pre_pts[4 * i + 1], 2) +
             pow(obs_pts[4 * i + 2] - pre_pts[4 * i + 2], 2) + pow(obs_pts[4 * i + 3] - pre_pts[4 * i + 3], 2)) < (reproj_error * reproj_error))
            inliers.push_back(i);
    }

    return inliers;
}