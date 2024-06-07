#include "loop_closing.hpp"


namespace slam
{


    LoopClosing::LoopClosing(int lc_neighbors,
                             int min_lc_inliers,
                             int lc_discard_window,
                             bool enable_debugging,
                             const std::string& w_path)
    {
        // Initialises parameters.
        w_path_ = w_path;
        lc_neighbors = lc_neighbors_;
        min_lc_inliers_ = min_lc_inliers;
        enable_debugging_ = enable_debugging;
        lc_discard_window_ = lc_discard_window;

        // Creates execution directory
        execution_path_ = w_path_ + "haloc/";
        if (std::filesystem::exists(execution_path_))
            std::filesystem::remove_all(execution_path_);
        std::filesystem::create_directory(execution_path_);

        // Creates debugging directories.
        if (enable_debugging_)
        {
            keyframe_path_ = w_path_ + "keyframes/";
            lc_path_ = w_path_ + "loop_closures/";
            if (std::filesystem::exists(lc_path_))
                std::filesystem::remove_all(lc_path_);
            std::filesystem::create_directory(lc_path_);
        }
    }


    void LoopClosing::run()
    {
        // Loop
        ros::Rate r(2);
        Cluster c_cluster;
        while (ros::ok())
        {
            // Search loop closing in the new cluster.
            if (checkNewClusterInQueue())
            {
                processNewCluster(c_cluster);

                ROS_INFO_STREAM("[LC->run]: Cluster id: " << c_cluster.getId());

                searchByHash(c_cluster);

                // searchByProximity();
            }
            
            r.sleep();
        }
    }


    void LoopClosing::addClusterToQueue(Cluster cluster)
    {
        mutex_cluster_queue_.lock();
        cluster_queue_.push_back(cluster);
        mutex_cluster_queue_.unlock();
    }


    bool LoopClosing::checkNewClusterInQueue()
    {
        mutex_cluster_queue_.lock();
        bool full = !cluster_queue_.empty();
        mutex_cluster_queue_.unlock();
        return full;   
    }


    void LoopClosing::processNewCluster(Cluster& c_cluster)
    {
        mutex_cluster_queue_.lock();
        c_cluster = cluster_queue_.front();
        cluster_queue_.pop_front();
        mutex_cluster_queue_.unlock();

        // Initialise hash
        if (!haloc_.isInitialized())
            haloc_.init(c_cluster.getLeftDescs());

        // Save hash to table
        hash_table_.push_back(std::make_pair(c_cluster.getId(), haloc_.getHash(c_cluster.getLeftDescs())));

        // Store
        cv::FileStorage fs(execution_path_ + convertTo5Digits(c_cluster.getId()) + ".yml", cv::FileStorage::WRITE);
        cv::write(fs, "frame_id", c_cluster.getFrameId());
        cv::write(fs, "l_kpts", c_cluster.getLeftKpts());
        cv::write(fs, "r_kpts", c_cluster.getRightKpts());
        cv::write(fs, "l_descs", c_cluster.getLeftDescs());
        cv::write(fs, "r_descs", c_cluster.getRightDescs());
        cv::write(fs, "obj_pts", c_cluster.getObjectPoints());
        fs.release();

        cluster_id_ = c_cluster.getId();
    }


    void LoopClosing::searchByHash(const Cluster& c_cluster)
    {
        // Get the candidates to close loop
        std::vector<std::pair<int, float>> candidates;
        getHashCandidates(c_cluster.getId(), candidates);
    }


    void LoopClosing::getHashCandidates(int c_cluster_id,
                                        std::vector<std::pair<int, float>>& best_candidates)
    {
        // Init
        best_candidates.clear();

        // Check if enough neighbors
        if ((int)hash_table_.size() <= lc_discard_window_)
            return;

        // Create a list with the non-possible candidates (because they are already loop closings).
        std::vector<int> no_candidates;
        for (uint i = 0; i < cluster_lc_found_.size(); i++)
        {
            if (cluster_lc_found_[i].first == c_cluster_id)
                no_candidates.push_back(cluster_lc_found_[i].second);
            if (cluster_lc_found_[i].second == c_cluster_id)
                no_candidates.push_back(cluster_lc_found_[i].first);
        }

        // Query hash.
        std::vector<float> q_hash = hash_table_[cluster_id_].second;

        // Loop over all the hashes stored.
        int cand_id;
        float match_dist;
        std::vector<float> cand_hash;
        std::vector<std::pair<int, float>> candidates;
        for (uint i = 0; i < hash_table_.size(); i++)
        {
            cand_id = hash_table_[i].first;

            // Discard window.
            if (cand_id > c_cluster_id - lc_discard_window_ && cand_id < c_cluster_id + lc_discard_window_)
                continue;

            // Do not compute the hash matching with itself.
            if (cand_id == c_cluster_id)
                continue;

            // Continue if candidate is in the no_candidates list.
            if (std::find(no_candidates.begin(), no_candidates.end(), hash_table_[i].first) != no_candidates.end())
                continue;

            // Hash matching.
            cand_hash = hash_table_[i].second;
            match_dist = haloc_.match(q_hash, cand_hash);
            candidates.push_back(std::make_pair(cand_id, match_dist));
        }
        
        // Sort the hash matchings
        std::sort(candidates.begin(), candidates.end(), sortByDistance);

        // Get best matches (minnor distance)
        uint max_size = 5;
        if (max_size > candidates.size())
            max_size = candidates.size();
        for (uint i = 0; i < max_size; i++)
            best_candidates.push_back(candidates[i]);
    }


}