/**
 *  \file frame.cpp
 *  \brief Frame class representing a stereo camera frame.
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  \author Original: Pep Lluís Negre-Carrasco (2013)
 *  \author Modified: Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon). Both the original 
 *  work and the modifications are released under the BSD 4-Clause License. 
 *  See the top-level LICENSE file for details.
 */


#include "frame.hpp"


Frame::Frame(cv::Mat l_descs,
             cv::Mat r_descs,
             ros::Time stamp,
             std::vector<cv::KeyPoint> l_kpts,
             std::vector<cv::KeyPoint> r_kpts,
             std::vector<cv::Point3f> l_cam_pts):
             stamp_{stamp}, l_descs_{l_descs}, r_descs_{r_descs}, 
             l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


Frame::Frame(cv::Mat l_img,
             cv::Mat r_img,
             cv::Mat l_descs,
             cv::Mat r_descs,
             ros::Time stamp,
             std::vector<cv::KeyPoint> l_kpts,
             std::vector<cv::KeyPoint> r_kpts,
             std::vector<cv::Point3f> l_cam_pts):
             stamp_{stamp}, l_img_{l_img}, r_img_{r_img}, l_descs_{l_descs}, r_descs_{r_descs}, 
             l_kpts_{l_kpts}, r_kpts_{r_kpts}, l_cam_pts_{l_cam_pts} {}


// FROM: http://codereview.stackexchange.com/questions/23966/density-based-clustering-of-image-keypoints
void Frame::regionClustering(int c_global_cluster_id)
{
    int c;
    const float eps = 50.0;
    const int min_pts = 20;
    std::vector<bool> visited, clustered;
    std::vector<int> noise, neighbor_pts, neighbor_pts_2;
    std::vector<std::vector<int>> tmp_clusters, all_clusters;

    const size_t total_size = l_kpts_.size();

    // Init visited and clustered.
    for (size_t k = 0; k < total_size; ++k)
    {
        visited.push_back(false);
        clustered.push_back(false);
    }

    c = -1;

    // For each unvisited point P in dataset keypoints.
    for (size_t i = 0; i < total_size; ++i)
    {
        if (!visited[i])
        {
            // Mark P as visited
            visited[i] = true;
            neighbor_pts = regionQuery(eps, &l_kpts_[i], &l_kpts_);
            if (neighbor_pts.size() < min_pts)
            {
                // Mark P as Noise
                noise.push_back(i);
                clustered[i] = true;
            }
            else
            {
                c++;
                tmp_clusters.emplace_back();

                // Expand cluster, add P to cluster c
                clustered[i] = true;
                tmp_clusters[c].push_back(i);

                // For each point P' in neighbor_pts
                for (size_t j = 0; j < neighbor_pts.size(); ++j)
                {
                    // If P' is not visited
                    if (!visited[neighbor_pts[j]])
                    {
                        // Mark P' as visited
                        visited[neighbor_pts[j]] = true;
                        neighbor_pts_2 = regionQuery(eps, &l_kpts_.at(neighbor_pts[j]), &l_kpts_);
                        if (neighbor_pts_2.size() >= min_pts)
                            neighbor_pts.insert(neighbor_pts.end(), neighbor_pts_2.begin(), neighbor_pts_2.end());
                    }

                    // If P' is not yet a member of any cluster, add P' to cluster c
                    if (!clustered[neighbor_pts[j]])
                    {
                        tmp_clusters[c].push_back(neighbor_pts[j]);
                        clustered[neighbor_pts[j]] = true;
                    }
                }
            }
        }
    }

    // Discard small clusters
    for (size_t i = 0; i < tmp_clusters.size(); ++i)
    {
        if (tmp_clusters[i].size() >= min_pts)
            all_clusters.push_back(tmp_clusters[i]);
        else
        {
            for (size_t j = 0; j < tmp_clusters[i].size(); ++j)
                noise.push_back(tmp_clusters[i][j]);
        }
    }

    // Refine points treat as noise.
    int idx;
    float dist;
    size_t size_a;
    cv::KeyPoint p_n, p_c;
    bool iterate = true, found;
    std::vector<int> noise_tmp;
    while (iterate && noise.size() > 0)
    {
        noise_tmp.clear();
        size_a = noise.size();
        for (size_t n = 0; n < noise.size(); ++n)
        {
            idx = -1;
            found = false;
            p_n = l_kpts_[noise[n]];
            for (size_t i = 0; i < all_clusters.size(); i++)
            {
                for (size_t j = 0; j < all_clusters[i].size(); j++)
                {
                    p_c = l_kpts_[all_clusters[i][j]];
                    dist = std::sqrt(std::pow((p_c.pt.x - p_n.pt.x), 2) + std::pow((p_c.pt.y - p_n.pt.y), 2));
                    if (dist <= eps && dist != 0.0)
                    {
                        idx = i;
                        found = true;
                        break;
                    }
                }
                if (found)
                    break;
            }
             
            if (found && idx >= 0)
                all_clusters[idx].push_back(noise[n]);
            else
                noise_tmp.push_back(noise[n]);
        }

        if (noise_tmp.empty() || noise_tmp.size() == size_a)
            iterate = false;

        noise = noise_tmp;
    }

    // If 1 cluster, add all keypoints
    if (all_clusters.size() <= 1)
    {
        std::vector<int> tmp_cluster;
        for (size_t i = 0; i < l_kpts_.size(); ++i)
            tmp_cluster.push_back(static_cast<int>(i));
        all_clusters.clear();
        all_clusters.push_back(tmp_cluster);
    }

    // Compute the clusters centroids
    idx = -1;
    cv::Point3f centroid, cluster_cam_pt;
    std::vector<cv::Point3f> cluster_l_cam_pts;
    for (size_t i = 0; i < all_clusters.size(); ++i)
    {
        cluster_l_cam_pts.clear();
        for (size_t j = 0; j < all_clusters[i].size(); ++j)
        {
            idx = all_clusters[i][j];
            cluster_cam_pt = l_cam_pts_[idx];
            cluster_l_cam_pts.push_back(cluster_cam_pt);
        }

        cluster_centroids_.push_back(compute3DCentroid(cluster_l_cam_pts));
    }

    // Pre-allocates memory.
    std::vector<int> cluster_ids;
    std::vector<cv::Point3f> l_cam_pts;
    std::vector<cv::KeyPoint> l_kpts, r_kpts;
    l_kpts.reserve(total_size);
    r_kpts.reserve(total_size);
    l_cam_pts.reserve(total_size);
    cluster_ids.reserve(total_size);

    // Sorts clusters.
    idx = -1;
    cv::Mat l_descs, r_descs;
    for (size_t i = 0; i < all_clusters.size(); ++i)
    {
        for (size_t j = 0; j < all_clusters[i].size(); ++j)
        {
            cluster_ids.push_back(c_global_cluster_id);

            idx = all_clusters[i][j];
            l_kpts.emplace_back(l_kpts_[idx]);
            r_kpts.emplace_back(r_kpts_[idx]);
            l_cam_pts.emplace_back(l_cam_pts_[idx]);
            l_descs.push_back(l_descs_.row(idx));
            r_descs.push_back(r_descs_.row(idx));
        }
        c_global_cluster_id++;
    }

    // Stores sorted clusters.
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    l_cam_pts_ = l_cam_pts;
    cluster_ids_ = cluster_ids;
    clusters_num_ = static_cast<int>(all_clusters.size());
}


void Frame::increaseClustersIds(int increase)
{
    for (size_t i = 0; i < cluster_ids_.size(); ++i)
        cluster_ids_[i] += increase;
}


std::vector<int> Frame::regionQuery(float eps, 
                                    cv::KeyPoint *kpt, 
                                    std::vector<cv::KeyPoint> *kpts)
{
    float dist;
    std::vector<int> ret_keys;
    for (size_t i = 0; i < kpts->size(); ++i)
    {
        dist = std::sqrt(std::pow((kpt->pt.x - kpts->at(i).pt.x),2) + std::pow((kpt->pt.y - kpts->at(i).pt.y),2));
        if (dist <= eps && dist != 0.0)
            ret_keys.push_back(i);
    }
    return ret_keys;
}


cv::Point3f Frame::compute3DCentroid(const std::vector<cv::Point3f>& pts)
{
    cv::Point3f sum = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < pts.size(); ++i)
        sum += pts[i];

    cv::Point3f centroid = {sum.x / pts.size(), sum.y / pts.size(), sum.z / pts.size()};
    return centroid;
}