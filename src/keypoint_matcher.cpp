#include "keypoint_matcher.hpp"


KeypointMatcher::KeypointMatcher(const double ratio,
                                 std::string& matching_type): 
                                 ratio_{ratio}
{
    // Sanity check
    for (unsigned int i = 0; i < matching_type.size(); i++)
    {
        if (isupper(matching_type[i]))
            matching_type[i] = std::tolower(matching_type[i]);
    }

    // Gets matching type.
    if (matching_type == "ratio_matching")
        matching_type_ = RATIO_MATCHING;
    else if (matching_type == "homography_matching")
        matching_type_ = HOMOGRAPHY_MATCHING;
    else if (matching_type == "circular_ratio_matching")
        matching_type_ = CIRCULAR_RATIO_MATCHING;
    else if (matching_type == "circular_homography_matching")
        matching_type_ = CIRCULAR_HOMOGRAPHY_MATCHING;
    else
    {
        std::cerr << "Unforseen matching type" << std::endl;
        matching_type_ = RATIO_MATCHING;
    }
}


void KeypointMatcher::matchStereoKeypoints(const cv::Mat& l_descs,
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
    for (unsigned int i = 0; i < h_f_l_kpts.size(); i++)
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


void KeypointMatcher::matchTemporalKeypoints(const cv::Mat& l_descs_0,
                                             const cv::Mat& r_descs_0,
                                             const cv::Mat& l_descs_1,
                                             const cv::Mat& r_descs_1,
                                             const std::vector<cv::KeyPoint>& l_kpts_0,
                                             const std::vector<cv::KeyPoint>& r_kpts_0,
                                             const std::vector<cv::KeyPoint>& l_kpts_1,
                                             const std::vector<cv::KeyPoint>& r_kpts_1,
                                             const std::vector<cv::Point3f>& scene_pts_0,
                                             const std::vector<cv::Point3f>& scene_pts_1,
                                             std::vector<cv::KeyPoint>& f_l_kpts_0,
                                             std::vector<cv::KeyPoint>& f_r_kpts_0,
                                             std::vector<cv::KeyPoint>& f_l_kpts_1,
                                             std::vector<cv::KeyPoint>& f_r_kpts_1,
                                             std::vector<cv::Point3f>& f_scene_pts_0,
                                             std::vector<cv::Point3f>& f_scene_pts_1)
{
    // Dummy variables.
    std::vector<int> cluster_ids_0, cluster_ids_1, f_cluster_ids_0, f_cluster_ids_1;
    
    // Matches keypoints and removes outliers using a general function.
    matchTemporalClusters(l_descs_0, r_descs_0, 
                          l_descs_1, r_descs_1, 
                          cluster_ids_0, cluster_ids_1, 
                          l_kpts_0, r_kpts_0, 
                          l_kpts_1, r_kpts_1, 
                          scene_pts_0, scene_pts_1,
                          f_cluster_ids_0, f_cluster_ids_1,
                          f_l_kpts_0, f_r_kpts_0, 
                          f_l_kpts_1, f_r_kpts_1, 
                          f_scene_pts_0, f_scene_pts_1);
}


void KeypointMatcher::matchTemporalClusters(const cv::Mat& l_descs_0,
                                            const cv::Mat& r_descs_0,
                                            const cv::Mat& l_descs_1,
                                            const cv::Mat& r_descs_1,
                                            const std::vector<int>& cluster_ids_0,
                                            const std::vector<int>& cluster_ids_1,
                                            const std::vector<cv::KeyPoint>& l_kpts_0,
                                            const std::vector<cv::KeyPoint>& r_kpts_0,
                                            const std::vector<cv::KeyPoint>& l_kpts_1,
                                            const std::vector<cv::KeyPoint>& r_kpts_1,
                                            const std::vector<cv::Point3f>& scene_pts_0,
                                            const std::vector<cv::Point3f>& scene_pts_1,
                                            std::vector<int>& f_cluster_ids_0,
                                            std::vector<int>& f_cluster_ids_1,
                                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                                            std::vector<cv::Point3f>& f_scene_pts_0,
                                            std::vector<cv::Point3f>& f_scene_pts_1)
{
    // Dummy variables.
    cv::Mat f_l_descs_0, f_r_descs_0, f_l_descs_1, f_r_descs_1;

    // Matches.
    if (matching_type_ == RATIO_MATCHING)
    {
        ratioMatching(l_descs_0, l_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, l_kpts_1, scene_pts_0, scene_pts_1,
                      f_l_descs_0, f_l_descs_1, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_l_kpts_1, f_scene_pts_0, f_scene_pts_1);
    }
    else if (matching_type_ == HOMOGRAPHY_MATCHING)
    {
        geometricalMatching(l_descs_0, l_descs_1, cluster_ids_0, cluster_ids_1, l_kpts_0, l_kpts_1, scene_pts_0, scene_pts_1,
                            f_l_descs_0, f_l_descs_1, f_cluster_ids_0, f_cluster_ids_1, f_l_kpts_0, f_l_kpts_1, f_scene_pts_0, f_scene_pts_1);
    }
    else if (matching_type_ == CIRCULAR_RATIO_MATCHING ||
             matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
    {
        // Temporal variables
        std::vector<cv::Point3f> r_scene_pts_0 = scene_pts_0, l_scene_pts_0 = scene_pts_0;
        std::vector<cv::Point3f> r_scene_pts_1 = scene_pts_1, l_scene_pts_1 = scene_pts_1;
        std::vector<cv::KeyPoint> h_f_l_kpts_0, h_f_r_kpts_0, h_f_l_kpts_1, h_f_r_kpts_1;
        std::vector<int> l_cluster_ids_0 = cluster_ids_0, r_cluster_ids_0 = cluster_ids_0;
        std::vector<int> l_cluster_ids_1 = cluster_ids_1, r_cluster_ids_1 = cluster_ids_1;
        std::vector<cv::Point3f> h_f_l_scene_pts_0, h_f_r_scene_pts_0, h_f_l_scene_pts_1, h_f_r_scene_pts_1;
        std::vector<int> h_f_l_cluster_ids_0, h_f_r_cluster_ids_0, h_f_l_cluster_ids_1, h_f_r_cluster_ids_1;
        if (matching_type_ == CIRCULAR_RATIO_MATCHING)
        {
            std::thread l_matcher(&KeypointMatcher::ratioMatching, this, std::ref(l_descs_0), std::ref(l_descs_1), std::ref(l_cluster_ids_0), std::ref(l_cluster_ids_1),
                                  std::ref(l_kpts_0), std::ref(l_kpts_1), std::ref(l_scene_pts_0), std::ref(l_scene_pts_1), std::ref(f_l_descs_0), std::ref(f_l_descs_1),
                                  std::ref(h_f_l_cluster_ids_0), std::ref(h_f_l_cluster_ids_1), std::ref(h_f_l_kpts_0), std::ref(h_f_l_kpts_1), std::ref(h_f_l_scene_pts_0), std::ref(h_f_l_scene_pts_1));
            std::thread r_matcher(&KeypointMatcher::ratioMatching, this, std::ref(r_descs_0), std::ref(r_descs_1), std::ref(r_cluster_ids_0), std::ref(r_cluster_ids_1),
                                  std::ref(r_kpts_0), std::ref(r_kpts_1), std::ref(r_scene_pts_0), std::ref(r_scene_pts_1), std::ref(f_r_descs_0), std::ref(f_r_descs_1),
                                  std::ref(h_f_r_cluster_ids_0), std::ref(h_f_r_cluster_ids_1), std::ref(h_f_r_kpts_0), std::ref(h_f_r_kpts_1), std::ref(h_f_r_scene_pts_0), std::ref(h_f_r_scene_pts_1));
            l_matcher.join();
            r_matcher.join();
        }
        else
        {
            std::thread l_matcher(&KeypointMatcher::geometricalMatching, this, std::ref(l_descs_0), std::ref(l_descs_1), std::ref(l_cluster_ids_0), std::ref(l_cluster_ids_1),
                                  std::ref(l_kpts_0), std::ref(l_kpts_1), std::ref(l_scene_pts_0), std::ref(l_scene_pts_0), std::ref(f_l_descs_0), std::ref(f_l_descs_1), 
                                  std::ref(h_f_l_cluster_ids_0), std::ref(h_f_l_cluster_ids_1), std::ref(h_f_l_kpts_0), std::ref(h_f_l_kpts_1), std::ref(h_f_l_scene_pts_0), std::ref(h_f_l_scene_pts_0));
            std::thread r_matcher(&KeypointMatcher::geometricalMatching, this, std::ref(r_descs_0), std::ref(r_descs_1), std::ref(r_cluster_ids_0), std::ref(r_cluster_ids_1),
                                  std::ref(r_kpts_0), std::ref(r_kpts_1), std::ref(r_scene_pts_0), std::ref(r_scene_pts_1), std::ref(f_r_descs_0), std::ref(f_r_descs_1),
                                  std::ref(h_f_r_cluster_ids_0), std::ref(h_f_r_cluster_ids_1), std::ref(h_f_r_kpts_0), std::ref(h_f_r_kpts_1), std::ref(h_f_r_scene_pts_0), std::ref(h_f_r_scene_pts_1));
            l_matcher.join();
            r_matcher.join();
        }

        if (h_f_r_kpts_0.size() < h_f_l_kpts_0.size())
        {
            assembleCircularMatchings(r_kpts_0, l_kpts_0, r_kpts_1, l_kpts_1, h_f_r_kpts_0, h_f_l_kpts_0, h_f_r_kpts_1, h_f_l_kpts_1,
                                      h_f_r_scene_pts_0, h_f_r_scene_pts_1, h_f_r_cluster_ids_0, h_f_r_cluster_ids_1, 
                                      f_r_kpts_0, f_l_kpts_0, f_r_kpts_1, f_l_kpts_1, f_scene_pts_0, f_scene_pts_1, f_cluster_ids_0, f_cluster_ids_1);
        }
        else
        {
            assembleCircularMatchings(l_kpts_0, r_kpts_0, l_kpts_1, r_kpts_1, h_f_l_kpts_0, h_f_r_kpts_0, h_f_l_kpts_1, h_f_r_kpts_1,
                                      h_f_l_scene_pts_0, h_f_l_scene_pts_1, h_f_l_cluster_ids_0, h_f_l_cluster_ids_1,
                                      f_l_kpts_0, f_r_kpts_0, f_l_kpts_1, f_r_kpts_1, f_scene_pts_0, f_scene_pts_1, f_cluster_ids_0, f_cluster_ids_1);
        }
    }
    else
    {
        std::cerr << "Unforeseen type of matching!" << std::endl;
    }
}


std::vector<std::vector<cv::DMatch>> KeypointMatcher::matching(const cv::Mat& descs_0,
                                                               const cv::Mat& descs_1)
{
    const int knn = 2;
    cv::Mat match_mask;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create("BruteForce");
    descriptor_matcher->knnMatch(descs_0, descs_1, knn_matches, knn, match_mask);
    return knn_matches;
}


void KeypointMatcher::ratioMatching(const cv::Mat& descs_0,
                                    const cv::Mat& descs_1,
                                    const std::vector<int>& cluster_ids_0,
                                    const std::vector<int>& cluster_ids_1,
                                    const std::vector<cv::KeyPoint>& kpts_0,
                                    const std::vector<cv::KeyPoint>& kpts_1,
                                    const std::vector<cv::Point3f>& scene_pts_0,
                                    const std::vector<cv::Point3f>& scene_pts_1,
                                    cv::Mat& f_descs_0,
                                    cv::Mat& f_descs_1,
                                    std::vector<int>& f_cluster_ids_0,
                                    std::vector<int>& f_cluster_ids_1,
                                    std::vector<cv::KeyPoint>& f_kpts_0,
                                    std::vector<cv::KeyPoint>& f_kpts_1,
                                    std::vector<cv::Point3f>& f_scene_pts_0,
                                    std::vector<cv::Point3f>& f_scene_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_scene_pts_0.clear();
    f_scene_pts_1.clear();
    f_cluster_ids_0.clear();
    f_cluster_ids_1.clear();
    if (descs_0.rows < 10 || descs_1.rows < 10)
        return;

    // During feature matching of the same stereo pair there are no 3D points.
    bool fill_3d;
    if (scene_pts_0.size() > 0)
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
    for (unsigned int i = 0; i < knn_matches.size(); i++)
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
                f_scene_pts_0.push_back(scene_pts_0[idx_0]);
                f_scene_pts_1.push_back(scene_pts_1[idx_1]);
            }
            if (fill_clusters)
            {
                f_cluster_ids_0.push_back(cluster_ids_0[idx_0]);
                f_cluster_ids_1.push_back(cluster_ids_1[idx_1]);
            }
        }
    }
}


void KeypointMatcher::geometricalMatching(const cv::Mat& descs_0,
                                          const cv::Mat& descs_1,
                                          const std::vector<int>& cluster_ids_0,
                                          const std::vector<int>& cluster_ids_1,
                                          const std::vector<cv::KeyPoint>& kpts_0,
                                          const std::vector<cv::KeyPoint>& kpts_1,
                                          const std::vector<cv::Point3f>& scene_pts_0,
                                          const std::vector<cv::Point3f>& scene_pts_1,
                                          cv::Mat& f_descs_0,
                                          cv::Mat& f_descs_1,
                                          std::vector<int>& f_cluster_ids_0,
                                          std::vector<int>& f_cluster_ids_1,
                                          std::vector<cv::KeyPoint>& f_kpts_0,
                                          std::vector<cv::KeyPoint>& f_kpts_1,
                                          std::vector<cv::Point3f>& f_scene_pts_0,
                                          std::vector<cv::Point3f>& f_scene_pts_1)
{
    // Sanity check
    f_kpts_0.clear();
    f_kpts_1.clear();
    f_descs_0.release();
    f_descs_1.release();
    f_scene_pts_0.clear();
    f_scene_pts_1.clear();
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
    std::vector<cv::Point3f> m_scene_pts_0, m_scene_pts_1;
    for (unsigned int i = 0; i < matches.size(); i++)
    {
        idx_0 = matches[i][0].queryIdx;
        idx_1 = matches[i][0].trainIdx;
        m_kpts_0.push_back(kpts_0[idx_0]);
        m_kpts_1.push_back(kpts_1[idx_1]);
        m_descs_0.push_back(descs_0.row(idx_0));
        m_descs_1.push_back(descs_1.row(idx_1));
        m_scene_pts_0.push_back(scene_pts_0[idx_0]);
        m_scene_pts_1.push_back(scene_pts_1[idx_1]);
        if (fill_clusters)
        {
            m_cluster_ids_0.push_back(cluster_ids_0[idx_0]);
            m_cluster_ids_1.push_back(cluster_ids_1[idx_1]);
        }
    }

    // Compute homography
    cv::Mat M, mask;
    std::vector<cv::Point2f> m_pts_0, m_pts_1;
    cv::KeyPoint::convert(m_kpts_0, m_pts_0);
    cv::KeyPoint::convert(m_kpts_1, m_pts_1);
    try
    {
        M = cv::findHomography(m_pts_0, m_pts_1, mask, cv::RANSAC, 1.0);
    }
    catch (cv::Exception &e)
    {
        const char* err_msg = e.what();
        std::cerr << "Exception caught: " << err_msg << std::endl;
        return;
    }

    // Filter outliers using homography.
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<bool>(i, 0) == 1)
        {
            f_kpts_0.push_back(m_kpts_0[i]);
            f_kpts_1.push_back(m_kpts_1[i]);
            f_descs_0.push_back(m_descs_0.row(i));
            f_descs_1.push_back(m_descs_1.row(i));
            f_scene_pts_0.push_back(m_scene_pts_0[i]);
            f_scene_pts_1.push_back(m_scene_pts_1[i]);
            if (fill_clusters)
            {
                f_cluster_ids_0.push_back(m_cluster_ids_0[i]);
                f_cluster_ids_1.push_back(m_cluster_ids_1[i]);
            }
        }
    }
}


void KeypointMatcher::assembleCircularMatchings(const std::vector<cv::KeyPoint>& stereo_kpts_0_lowest,
                                                const std::vector<cv::KeyPoint>& stereo_kpts_0_highest,
                                                const std::vector<cv::KeyPoint>& stereo_kpts_1_lowest,
                                                const std::vector<cv::KeyPoint>& stereo_kpts_1_highest,
                                                const std::vector<cv::KeyPoint>& temporal_kpts_0_lowest,
                                                const std::vector<cv::KeyPoint>& temporal_kpts_0_highest,
                                                const std::vector<cv::KeyPoint>& temporal_kpts_1_lowest,
                                                const std::vector<cv::KeyPoint>& temporal_kpts_1_highest,
                                                const std::vector<cv::Point3f>& temporal_scene_pts_0,
                                                const std::vector<cv::Point3f>& temporal_scene_pts_1,
                                                const std::vector<int>& temporal_cluster_ids_0,
                                                const std::vector<int>& temporal_cluster_ids_1,
                                                std::vector<cv::KeyPoint>& f_kpts_0_lowest,
                                                std::vector<cv::KeyPoint>& f_kpts_0_highest,
                                                std::vector<cv::KeyPoint>& f_kpts_1_lowest,
                                                std::vector<cv::KeyPoint>& f_kpts_1_highest,
                                                std::vector<cv::Point3f>& f_scene_pts_0,
                                                std::vector<cv::Point3f>& f_scene_pts_1,
                                                std::vector<int>& f_cluster_ids_0,
                                                std::vector<int>& f_cluster_ids_1)
{
    // If there are no clusters, the filtered clusters vectors must not be filled.
    bool fill_clusters;
    if (temporal_cluster_ids_0.size() > 0)
        fill_clusters = true;
    else
        fill_clusters = false;

    // Assembles
    unsigned int j;
    unsigned int k;
    unsigned int l;
    for (unsigned int i = 0; i < temporal_kpts_1_lowest.size(); i++)
    {
        for (j = 0; j < stereo_kpts_1_lowest.size(); j++)
        {
            if (temporal_kpts_1_lowest[i].pt == stereo_kpts_1_lowest[j].pt)
            {
                for (k = 0; k < temporal_kpts_1_highest.size(); k++)
                {
                    if (stereo_kpts_1_highest[j].pt == temporal_kpts_1_highest[k].pt)
                    {
                        for (l = 0; l < stereo_kpts_0_highest.size(); l++)
                        {
                            if (temporal_kpts_0_highest[k].pt == stereo_kpts_0_highest[l].pt)
                            {
                                if (stereo_kpts_0_lowest[l].pt == temporal_kpts_0_lowest[i].pt)
                                {
                                    f_kpts_0_lowest.push_back(temporal_kpts_0_lowest[i]);
                                    f_kpts_0_highest.push_back(temporal_kpts_0_highest[k]);
                                    f_kpts_1_lowest.push_back(temporal_kpts_1_lowest[i]);
                                    f_kpts_1_highest.push_back(temporal_kpts_1_highest[k]);
                                    f_scene_pts_0.push_back(temporal_scene_pts_0[i]);
                                    f_scene_pts_1.push_back(temporal_scene_pts_1[i]);
                                    if (fill_clusters)
                                    {
                                        f_cluster_ids_0.push_back(temporal_cluster_ids_0[i]);
                                        f_cluster_ids_1.push_back(temporal_cluster_ids_1[i]);
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