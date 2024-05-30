#include "frame.hpp"


Frame::Frame(cv::Mat l_descs,
             cv::Mat r_descs,
             ros::Time stamp,
             std::vector<cv::Point2f> l_kpts,
             std::vector<cv::Point2f> r_kpts,
             std::vector<cv::Point3f> obj_pts)
{
    stamp_ = stamp;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    obj_pts_ = obj_pts;
}


Frame::Frame(cv::Mat l_img,
             cv::Mat r_img,
             cv::Mat l_descs,
             cv::Mat r_descs,
             ros::Time stamp,
             std::vector<cv::Point2f> l_kpts,
             std::vector<cv::Point2f> r_kpts,
             std::vector<cv::Point3f> obj_pts)
{
    l_img_ = l_img;
    r_img_ = r_img;
    stamp_ = stamp;
    l_kpts_ = l_kpts;
    r_kpts_ = r_kpts;
    l_descs_ = l_descs;
    r_descs_ = r_descs;
    obj_pts_ = obj_pts;
}


// FROM: http://codereview.stackexchange.com/questions/23966/density-based-clustering-of-image-keypoints
void Frame::regionClustering()
{
    clusters_.clear();
    int c;
    const float eps = 50.0;
    const int min_pts = 20;
    std::vector<int> noise;
    std::vector<int> neighbor_pts;
    std::vector<int> neighbor_pts_;
    std::vector<bool> visited;
    std::vector<bool> clustered;
    std::vector<std::vector<int>> clusters;

    uint no_keys = l_kpts_.size();

    // Init visited and clustered.
    for (uint k = 0; k < no_keys; k++)
    {
        visited.push_back(false);
        clustered.push_back(false);
    }
    clusters_ = clusters;

    c = -1;

    // For each unvisited point P in dataset keypoints.
    for (uint i = 0; i < no_keys; i++)
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
                clusters.push_back(std::vector<int>());

                // Expand cluster, add P to cluster c
                clustered[i] = true;
                clusters[c].push_back(i);

                // For each point P' in neighbor_pts
                for (uint j = 0; j < neighbor_pts.size(); j++)
                {
                    // If P' is not visited
                    if (!visited[neighbor_pts[j]])
                    {
                        // Mark P' as visited
                        visited[neighbor_pts[j]] = true;
                        neighbor_pts_ = regionQuery(eps, &l_kpts_.at(neighbor_pts[j]), &l_kpts_);
                        if (neighbor_pts_.size() >= min_pts)
                            neighbor_pts.insert(neighbor_pts.end(), neighbor_pts_.begin(), neighbor_pts_.end());
                    }

                    // If P' is not yet a member of any cluster, add P' to cluster c
                    if (!clustered[neighbor_pts[j]])
                    {
                        clusters[c].push_back(neighbor_pts[j]);
                        clustered[neighbor_pts[j]] = true;
                    }
                }
            }
        }
    }

    // Discard small clusters
    for (uint i = 0; i < clusters.size(); i++)
    {
        if (clusters[i].size() >= min_pts)
            clusters_.push_back(clusters[i]);
        else
        {
            for (uint j = 0; j < clusters[i].size(); j++)
                noise.push_back(clusters[i][j]);
        }
    }

    // Refine points treat as noise.
    int idx;
    float dist;
    uint size_a;
    cv::Point2f p_n, p_c;
    bool iterate = true, found;
    std::vector<int> noise_tmp;
    while (iterate && noise.size() > 0)
    {
        noise_tmp.clear();
        size_a = noise.size();
        for (uint n = 0; n < noise.size(); n++)
        {
            idx = -1;
            found = false;
            p_n = l_kpts_[noise[n]];
            for (uint i = 0; i < clusters_.size(); i++)
            {
                for (uint j = 0; j < clusters_[i].size(); j++)
                {
                    p_c = l_kpts_[clusters_[i][j]];
                    dist = sqrt(pow((p_c.x - p_n.x), 2) + pow((p_c.y - p_n.y), 2));
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
                clusters_[idx].push_back(noise[n]);
            else
                noise_tmp.push_back(noise[n]);
        }

        if (noise_tmp.size() == 0 || noise_tmp.size() == size_a)
            iterate = false;

        noise = noise_tmp;
    }

    // If 1 cluster, add all keypoints
    if (clusters_.size() <= 1)
    {
        std::vector<int> cluster_tmp;
        for (uint i = 0; i < l_kpts_.size(); i++)
            cluster_tmp.push_back(int(i));
        clusters_.clear();
        clusters_.push_back(cluster_tmp);
    }

    // Compute the clusters centroids
    idx = -1;
    cv::Point3f centroid, cluster_obj_pt;
    std::vector<cv::Point3f> cluster_obj_pts;
    for (uint i = 0; i < clusters_.size(); i++)
    {
        cluster_obj_pts.clear();
        for (uint j = 0; j < clusters_[i].size(); j++)
        {
            idx = clusters_[i][j];
            cluster_obj_pt = obj_pts_[idx];
            cluster_obj_pts.push_back(cluster_obj_pt);
        }

        cluster_centroids_.push_back(compute3DCentroid(cluster_obj_pts));
    }
}


std::vector<int> Frame::regionQuery(float eps, 
                                    cv::Point2f *keypoint, 
                                    std::vector<cv::Point2f> *keypoints)
{
    float dist;
    std::vector<int> ret_keys;
    for (uint i = 0; i < keypoints->size(); i++)
    {
        dist = sqrt(pow((keypoint->x - keypoints->at(i).x),2)+pow((keypoint->y - keypoints->at(i).y),2));
        if (dist <= eps && dist != 0.0)
            ret_keys.push_back(i);
    }
    return ret_keys;
}


cv::Point3f Frame::compute3DCentroid(const std::vector<cv::Point3f>& pts)
{
    cv::Point3f sum = {0.0, 0.0, 0.0};
    for (uint i = 0; i < pts.size(); i++)
        sum += pts[i];

    cv::Point3f centroid = {sum.x / pts.size(), sum.y / pts.size(), sum.z / pts.size()};
    return centroid;
}