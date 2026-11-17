/**
 *  @file motion_estimator.cpp
 *  @brief The MotionEstimtor class provides tools to compute the motion between stereo images.
 *  @package slamon
 *  @project SLAMON
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  @license BSD-4-Clause
 *    This file is part of the SLAMON project and is released under
 *    the BSD 4-Clause License. See the LICENSE file for details.
 */

 
#include "motion_estimator.hpp"


MotionEstimator::MotionEstimator(const std::string& motion_estimation_type)
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
    const double f_u = l_proj_mat.at<double>(0, 0);
    const double t_x = r_proj_mat.at<double>(0, 3);
    const double b = (t_x / -f_u);

    // Pre-allocates memory.
    const int l_cam_pts_size = l_cam_pts.rows;
    f_l_kpts.reserve(l_cam_pts_size);
    f_r_kpts.reserve(l_cam_pts_size);
    f_l_cam_pts.reserve(l_cam_pts_size);

    // Filters "far" keypoints.
    cv::Point3f l_cam_pt;
    const double z_limit = 40 * b;
    for (int i = 0; i < l_cam_pts_size; ++i)
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
            f_l_kpts.emplace_back(l_kpts[i]);
            f_r_kpts.emplace_back(r_kpts[i]);
            f_l_cam_pts.emplace_back(l_cam_pt);
            f_l_descs.push_back(l_descs.row(i));
            f_r_descs.push_back(r_descs.row(i));
        }
    }
}


std::vector<std::vector<std::pair<int, cv::KeyPoint>>> MotionEstimator::bucketKeypoints(const size_t max_kpts,
                                                                                        const float img_width,
                                                                                        const float img_height,
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
    f_c_l_kpts.clear(); f_c_r_kpts.clear();
    f_p_l_kpts.clear(); f_p_r_kpts.clear();
    f_c_l_cam_pts.clear(); f_p_l_cam_pts.clear();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    const bool has_right = !c_r_kpts.empty();

    // Calculate the number of rows and columns that each bucket has.
    const float bucket_width = img_width / 10.0;
    const float bucket_height = img_height / 10.0;

    // Fill buckets with keypoints and its indices.
    int bucket_idx, bucket_row, bucket_col;
    std::vector<std::vector<std::pair<int, cv::KeyPoint>>> buckets(100);
    for (size_t i = 0; i < c_l_kpts.size(); ++i)
    {
        bucket_col = static_cast<int>(std::floor(c_l_kpts[i].pt.x / bucket_width));
        bucket_row = static_cast<int>(std::floor(c_l_kpts[i].pt.y / bucket_height));
        bucket_idx = static_cast<int>(10 * bucket_col + bucket_row);
        buckets[bucket_idx].emplace_back(i, c_l_kpts[i]);
    }

    // Remove excess keypoints from the buckets.
    bool odd, init;
    size_t idx_forwards;
    float slope, median, delta_x;
    cv::KeyPoint l_kpt_0, l_kpt_1;
    std::vector<std::pair<int, cv::KeyPoint>> inliers;
    int idx, idx_backwards, half_longitude;
    std::vector<std::pair<float, std::pair<int, cv::KeyPoint>>> bucket_slopes;
    for (size_t i = 0; i < buckets.size(); ++i)
    {
        if (buckets[i].size() > max_kpts)
        {
            // Calculate the coincidence slopes.
            bucket_slopes.clear();
            for (size_t j = 0; j < buckets[i].size(); ++j)
            {
                idx = buckets[i][j].first;
                l_kpt_0 = c_l_kpts[idx];
                l_kpt_1 = p_l_kpts[idx];
                delta_x = l_kpt_1.pt.x - l_kpt_0.pt.x;
                if (delta_x != 0)
                    slope = (l_kpt_1.pt.y - l_kpt_0.pt.y) / (delta_x);
                else
                    slope = 0.0;
                bucket_slopes.emplace_back(slope, buckets[i][j]);
            }

            // Sort the keypoints of the bucket according to the slope of their matches.
            std::sort(bucket_slopes.begin(), bucket_slopes.end(), SortBucketsByMatchesSlopesAscending());

            // Detects whether the number of keypoints is odd or even.
            odd = true;
            if (bucket_slopes.size() % 2 == 0)
                odd = false;

            // Gets n center values.
            median = 0.0;
            init = false;
            inliers.clear();
            half_longitude = std::ceil(bucket_slopes.size() / 2);
            idx_backwards = half_longitude - 1;
            idx_forwards = half_longitude + 1;
            while (inliers.size() < max_kpts)
            {
                if (!init)
                {
                    if (odd)
                    {
                        median = bucket_slopes[half_longitude].first;
                        inliers.emplace_back(bucket_slopes[half_longitude].second);
                    }
                    else
                    {
                        median = ((bucket_slopes[half_longitude].first + bucket_slopes[idx_backwards].first) / 2);
                        inliers.emplace_back(bucket_slopes[half_longitude].second);
                        if (max_kpts > 1)
                        {
                            inliers.emplace_back(bucket_slopes[idx_backwards].second);
                            idx_backwards--;
                        }
                    }
                    init = true;
                }
                else
                {
                    if (idx_backwards < 0)
                    {
                        inliers.emplace_back(bucket_slopes[idx_forwards].second);
                        idx_forwards++;
                    }
                    else if (idx_forwards >= bucket_slopes.size())
                    {
                        inliers.emplace_back(bucket_slopes[idx_backwards].second);
                        idx_backwards--;
                    }
                    else if ((median - bucket_slopes[idx_backwards].first) < (bucket_slopes[idx_forwards].first - median))
                    {
                        inliers.emplace_back(bucket_slopes[idx_backwards].second);
                        idx_backwards--;
                    }
                    else
                    {
                        inliers.emplace_back(bucket_slopes[idx_forwards].second);
                        idx_forwards++;
                    }
                }
            }

            buckets[i] = inliers;
        }

        for (size_t j = 0; j < buckets[i].size(); ++j)
        {
            idx = buckets[i][j].first;
            f_c_l_kpts.emplace_back(c_l_kpts[idx]);
            f_p_l_kpts.emplace_back(p_l_kpts[idx]);
            f_c_l_cam_pts.emplace_back(c_l_cam_pts[idx]);
            f_p_l_cam_pts.emplace_back(p_l_cam_pts[idx]);

            if (has_right)
            {
                f_c_r_kpts.emplace_back(c_r_kpts[idx]);
                f_p_r_kpts.emplace_back(p_r_kpts[idx]);
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
    r_vec.release(); t_vec.release();
    f_l_kpts_0.clear(); f_l_kpts_1.clear();
    f_r_kpts_0.clear(); f_r_kpts_1.clear();
    f_scene_pts_0.clear(); f_scene_pts_1.clear();
    f_cluster_ids_0.clear(); f_cluster_ids_1.clear();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    const bool has_right = !r_kpts_0.empty();

    // If there are no clusters, the filtered cluster vectors must not be filled.
    const bool has_clusters = !cluster_ids_0.empty();

    // From keypoints to points.
    std::vector<cv::Point2f> l_pts_0, r_pts_0;
    cv::KeyPoint::convert(l_kpts_0, l_pts_0);
    cv::KeyPoint::convert(r_kpts_0, r_pts_0);

    // Estimate motion.
    std::vector<int> inliers;
    const cv::Mat K = l_proj_mat(cv::Rect(0, 0, 3, 3));
    cv::solvePnPRansac(scene_pts_1, l_pts_0, K, cv::noArray(), r_vec, t_vec, 
                       false, 2000, 1.0, 0.99, inliers, motion_estimation_type_);

    // Pre-allocates memory.
    const size_t inliers_size = inliers.size();
    f_l_kpts_0.reserve(inliers_size);
    f_l_kpts_1.reserve(inliers_size);
    f_scene_pts_0.reserve(inliers_size);
    f_scene_pts_1.reserve(inliers_size);
    if (has_right)
    {
        f_r_kpts_0.reserve(inliers_size);
        f_r_kpts_1.reserve(inliers_size);
    }
    if (has_clusters)
    {
        f_cluster_ids_0.reserve(inliers_size);
        f_cluster_ids_1.reserve(inliers_size);
    }

    // Filter keypoints, 3D points and clusters.
    for (size_t i = 0; i < inliers_size; ++i)
    {
        f_l_kpts_0.emplace_back(l_kpts_0[inliers[i]]);
        f_l_kpts_1.emplace_back(l_kpts_1[inliers[i]]);
        f_scene_pts_0.emplace_back(scene_pts_0[inliers[i]]);
        f_scene_pts_1.emplace_back(scene_pts_1[inliers[i]]);
        if (has_right)
        {
            f_r_kpts_0.emplace_back(r_kpts_0[inliers[i]]);
            f_r_kpts_1.emplace_back(r_kpts_1[inliers[i]]);
        }
        if (has_clusters)
        {
            f_cluster_ids_0.emplace_back(cluster_ids_0[inliers[i]]);
            f_cluster_ids_1.emplace_back(cluster_ids_1[inliers[i]]);
        }
    }
}


void MotionEstimator::estimateRefinedMotion(const cv::Mat& l_proj_mat,
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
    estimateRefinedMotion(l_proj_mat, r_proj_mat,
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


void MotionEstimator::estimateRefinedMotion(const cv::Mat& l_proj_mat,
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
    r_vec.release(); t_vec.release();
    f_l_kpts_0.clear(); f_l_kpts_1.clear();
    f_r_kpts_0.clear(); f_r_kpts_1.clear();
    f_scene_pts_0.clear(); f_scene_pts_1.clear();
    f_cluster_ids_0.clear(); f_cluster_ids_1.clear();

    // If there are no right keypoints, the filtered right verctor must not be filled.
    const bool has_right = !r_kpts_0.empty();

    // If there are no clusters, the filtered cluster vectors must not be filled.
    const bool has_clusters = !cluster_ids_0.empty();

    // From keypoints to points.
    std::vector<cv::Point2f> l_pts_0, r_pts_0;
    cv::KeyPoint::convert(l_kpts_0, l_pts_0);
    cv::KeyPoint::convert(r_kpts_0, r_pts_0);

    // Estimate motion.
    std::vector<int> inliers;
    const cv::Mat K = l_proj_mat(cv::Rect(0, 0, 3, 3));
    cv::solvePnPRansac(scene_pts_1, l_pts_0, K, cv::noArray(), r_vec, t_vec, 
                       false, 2000, 1.0, 0.99, inliers, motion_estimation_type_);

    // Pre-allocates memory.
    const size_t inliers_size = inliers.size();
    std::vector<cv::Point2f> f_l_pts_0;
    f_l_pts_0.reserve(inliers_size);
    f_l_kpts_0.reserve(inliers_size);
    f_l_kpts_1.reserve(inliers_size);
    f_scene_pts_0.reserve(inliers_size);
    f_scene_pts_1.reserve(inliers_size);
    if (has_right)
    {
        f_r_kpts_0.reserve(inliers_size);
        f_r_kpts_1.reserve(inliers_size);
    }
    if (has_clusters)
    {
        f_cluster_ids_0.reserve(inliers_size);
        f_cluster_ids_1.reserve(inliers_size);
    }

    // Filter points, keypoints, 3D points and clusters.
    for (size_t i = 0; i < inliers_size; ++i)
    {
        f_l_pts_0.emplace_back(l_pts_0[inliers[i]]);
        f_l_kpts_0.emplace_back(l_kpts_0[inliers[i]]);
        f_l_kpts_1.emplace_back(l_kpts_1[inliers[i]]);
        f_scene_pts_0.emplace_back(scene_pts_0[inliers[i]]);
        f_scene_pts_1.emplace_back(scene_pts_1[inliers[i]]);
        if (has_right)
        {
            f_r_kpts_0.emplace_back(r_kpts_0[inliers[i]]);
            f_r_kpts_1.emplace_back(r_kpts_1[inliers[i]]);
        }
        if (has_clusters)
        {
            f_cluster_ids_0.emplace_back(cluster_ids_0[inliers[i]]);
            f_cluster_ids_1.emplace_back(cluster_ids_1[inliers[i]]);
        }
    }

    // Refines pose.
    if (f_scene_pts_1.size() >= 3 && f_l_pts_0.size() >= 3)
        cv::solvePnPRefineLM(f_scene_pts_1, f_l_pts_0, K, cv::noArray(), r_vec, t_vec);
}


MotionEstimator::Type MotionEstimator::selectMotionEstimationType(const std::string& motion_estimation_type)
{
    // Sanity check
    std::string lower_motion_estimation_type = motion_estimation_type;
    std::transform(lower_motion_estimation_type.begin(), lower_motion_estimation_type.end(), lower_motion_estimation_type.begin(), ::tolower);

    if (motion_estimation_type == "solvepnp_p3p" || motion_estimation_type == "p3p")
        return P3P;
    else if (motion_estimation_type == "solvepnp_ap3p" || motion_estimation_type == "ap3p")
        return AP3P;
    else if (motion_estimation_type == "solvepnp_epnp" || motion_estimation_type == "epnp")
        return EPNP;
    else
    {
        std::cerr << "Unforseen motion estimation type" << std::endl;
        return AP3P;
    }
}