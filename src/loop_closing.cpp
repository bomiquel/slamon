#include "loop_closing.hpp"


namespace slam
{


    LoopClosing::LoopClosing(double ratio,
                             int lc_neighbors,
                             int min_lc_inliers,
                             int lc_discard_window,
                             bool enable_debugging,
                             const std::string& w_path,
                             std::string& matching_type,
                             std::string& motion_estimation_type):
                             StereoModules(ratio, min_lc_inliers, matching_type, motion_estimation_type)
    {
        // Initialises parameters.
        w_path_ = w_path;
        lc_neighbors = lc_neighbors_;
        min_lc_inliers_ = min_lc_inliers;
        enable_debugging_ = enable_debugging;
        lc_discard_window_ = lc_discard_window;

        // Set "uninitialised" to the loop closing.
        init_loop_closing_ = false;

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
        // Create ROS node.
        ros::NodeHandle nh;

        // Client services.
        // proj_mat_client_ = nh.serviceClient<slamon::

        // Loop
        ros::Rate r(5);
        Cluster c_cluster;
        while (ros::ok())
        {
            // Search loop closing in the new cluster.
            if (checkNewClusterInQueue())
            {
                // if (init_loop_closing_)


                processNewCluster(c_cluster);

                ROS_INFO_STREAM("[LC->run]: Cluster id: " << c_cluster.getId());

                searchByProximity(c_cluster);

                searchByHash(c_cluster);
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
    }


    // TODO: no_candidates?
    void LoopClosing::searchByProximity(const Cluster& c_cluster)
    {
        // Get the candidates to close loop
        std::vector<int> candidates;
        graph_->findClosestVertices(c_cluster.getId(), c_cluster.getId(), 
                                    lc_discard_window_, 5, candidates);
        if (candidates.size() == 0)
            return;

        // Loop over candidates.
        Cluster cand_cluster;
        for (uint i = 0; i < candidates.size(); i++)
        {
            cand_cluster = readCluster(candidates[i]);
            if (cand_cluster.getLeftDescs().rows == 0)
            {
                ROS_WARN_STREAM("[LC->searchByProximity->readCluster:] It is not possible to read the cluster " << candidates[i] << " from the disk!");
                continue;
            }
        }
    }


    void LoopClosing::searchByHash(const Cluster& c_cluster)
    {
        // Get the candidates to close loop
        std::vector<int> candidates;
        getHashCandidates(c_cluster.getId(), 5, candidates);
        if (candidates.size() == 0)
            return;
        
        // Loop over candidates.
        Cluster cand_cluster;
        for (uint i = 0; i < candidates.size(); i++)
        {
            cand_cluster = readCluster(candidates[i]);
            if (cand_cluster.getLeftDescs().rows == 0)
            {
                ROS_WARN_STREAM("[LC->searchByHash->readCluster:] It is not possible to read the cluster " << candidates[i] << " from the disk!");
                continue;
            }
        }
    }


    void LoopClosing::getHashCandidates(int c_cluster_id,
                                        uint num_closest_candidates,
                                        std::vector<int>& closest_candidates)
    {
        // Init.
        closest_candidates.clear();

        // Check if enough neighbors.
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
        std::vector<float> q_hash = hash_table_[c_cluster_id].second;

        // Loop over all the hashes stored.
        int cand_id;
        float match_dist;
        std::vector<float> cand_hash;
        std::vector<std::pair<int, float>> candidates;
        for (uint i = 0; i < hash_table_.size(); i++)
        {
            cand_id = hash_table_[i].first;

            // Do not compute the hash matching with itself.
            if (cand_id == c_cluster_id)
                continue;

            // Discards window.
            if ((cand_id > c_cluster_id - lc_discard_window_) && (cand_id < c_cluster_id + lc_discard_window_))
                continue;

            // Continue if candidate is in the no_candidates list.
            if (std::find(no_candidates.begin(), no_candidates.end(), hash_table_[i].first) != no_candidates.end())
                continue;

            // Hash matching.
            cand_hash = hash_table_[i].second;
            match_dist = haloc_.match(q_hash, cand_hash);
            candidates.push_back(std::make_pair(cand_id, match_dist));
        }
        
        // Exit if no candidates.
        if (candidates.size() == 0)
            return;

        // Sorts the candidates from least to furthest matching distance to the query neighbour.
        std::sort(candidates.begin(), candidates.end(), sortByMatching);

        // Gets closest matches (minnor mmatching distance)
        if (num_closest_candidates > candidates.size())
            num_closest_candidates = candidates.size();
        for (uint i = 0; i < num_closest_candidates; i++)
            closest_candidates.push_back(candidates[i].first);
    }


    Cluster LoopClosing::readCluster(int id)
    {
        Cluster cluster_tmp;
        std::string cluster_path = execution_path_ + convertTo5Digits(id) + ".yml";

        // Sanity check
        if (!std::filesystem::exists(cluster_path))
            return cluster_tmp;

        // Opens cluster file.
        cv::FileStorage fs;
        fs.open(cluster_path, cv::FileStorage::READ);

        // Sanity check x2
        if (!fs.isOpened())
            return cluster_tmp;

        // Reads cluster data.
        int frame_id;
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> l_kpts, r_kpts;
        fs["l_kpts"] >> l_kpts;
        fs["r_kpts"] >> r_kpts;
        fs["obj_pts"] >> obj_pts;
        fs["l_descs"] >> l_descs;
        fs["r_descs"] >> r_descs;
        fs["frame_id"] >> frame_id;

        // Sets the properties of the cluster.
        tf::Transform camera_pose = graph_->getVertexCameraPose(id, true);
        Cluster cluster(id, frame_id, l_descs, r_descs, camera_pose, l_kpts, r_kpts, obj_pts);
        return cluster;
    }


} // End namespace