#include "loop_closer.hpp"


namespace slam
{


    LoopCloser::LoopCloser(double ratio,
                           int lc_neighbors,
                           int lc_candidates,
                           int min_lc_inliers,
                           int lc_discard_window,
                           bool enable_debugging,
                           const std::string& w_path,
                           std::string& matching_type,
                           std::string& motion_estimation_type):
                           VisionMotionModules(ratio, matching_type, motion_estimation_type)
    {
        // Initialises parameters.
        w_path_ = w_path;
        lc_neighbors_ = lc_neighbors;
        lc_candidates_ = lc_candidates;
        min_lc_inliers_ = min_lc_inliers;
        enable_debugging_ = enable_debugging;
        lc_discard_window_ = lc_discard_window;

        // Set "uninitialised" to the loop closer.
        init_loop_closer_ = false;

        // Initialises number of closed loops.
        loop_closure_num_ = 0; 

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


    void LoopCloser::run()
    { 
        // Create ROS node.
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");

        // Client services.
        proj_mat_client_ = nh.serviceClient<slamon::GetProjectionMatrices>("get_projection_matrices");

        // Publishers.
        run_times_pub_ = nhp.advertise<slamon::Performance>("loop_closer_performance", 1);
        cluster_num_pub_ = nhp.advertise<std_msgs::Int32>("cluster_num", 1);
        loop_closure_num_pub_ = nhp.advertise<std_msgs::Int32>("loop_closure_num", 1);
        loop_closure_img_pub_ = nhp.advertise<sensor_msgs::Image>("loop_closure_img", 1);

        // Loop
        ros::Rate r(10);
        Cluster c_cluster;
        int p_loop_closures_num;
        double start_module_time;
        double start_iteration_time;
        while (ros::ok())
        {
            // Search for a loop closure in the new cluster.
            if (checkNewClusterInQueue())
            {
                // Time at which the program starts the iteration.
                start_iteration_time = ros::Time::now().toSec();

                // Initialisation of the variables necessary to measure thread performance.
                p_loop_closures_num = loop_closure_num_;
                run_times_.run_time_3 = 0.0; // Read cluster candidates run time.
                run_times_.run_time_4 = 0.0; // Loop detection run time.
                run_times_.run_time_5 = 0.0; // Close loop run time.

                if (!init_loop_closer_)
                    getProjectionMatrices();
                
                start_module_time = ros::Time::now().toSec();
                processNewCluster(c_cluster);
                run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Process current cluster run time.

                if (enable_debugging_)
                    ROS_INFO_STREAM("[LC->run]: Cluster: " << c_cluster.getId() << " from keyframe: " << c_cluster.getKeyframeId());

                searchByProximity(c_cluster);

                searchByHash(c_cluster);

                if (cluster_num_pub_.getNumSubscribers() > 0)
                    publishCounter(c_cluster.getId() + 1, cluster_num_pub_);
                if (loop_closure_num_pub_.getNumSubscribers() > 0)
                    publishCounter(loop_closure_num_, loop_closure_num_pub_);

                // Publishes last iteration performance.
                run_times_.total_run_time = ros::Time::now().toSec() - start_iteration_time;
                if (run_times_pub_.getNumSubscribers() > 0)
                    publishPerformance(ros::Time::now(), loop_closure_num_ - p_loop_closures_num, run_times_, run_times_pub_);
            }
            r.sleep();
        }
    }


    void LoopCloser::addClusterToQueue(Cluster cluster)
    {
        std::scoped_lock lock(mutex_cluster_queue_);
        cluster_queue_.push_back(cluster);
    }


    bool LoopCloser::checkNewClusterInQueue()
    {
        std::scoped_lock lock(mutex_cluster_queue_);
        return (!cluster_queue_.empty());
    }


    void LoopCloser::getProjectionMatrices()
    {
        slamon::GetProjectionMatrices srv;
        if (proj_mat_client_.call(srv))
        {
            // Gets projection matrix
            l_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(srv.response.l_proj_mat.data())).clone();
            r_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(srv.response.r_proj_mat.data())).clone();

            ROS_WARN_STREAM("[LC->Projection matrices:] GetProjectionMatrices service call successful." << std::endl <<
                            "The left projection is as follow:" << std::endl <<
                            l_proj_mat_.at<double>(0, 0) << " " << l_proj_mat_.at<double>(0, 1) << " " << l_proj_mat_.at<double>(0, 2) << " " << l_proj_mat_.at<double>(0, 3) << std::endl <<
                            l_proj_mat_.at<double>(1, 0) << " " << l_proj_mat_.at<double>(1, 1) << " " << l_proj_mat_.at<double>(1, 2) << " " << l_proj_mat_.at<double>(1, 3) << std::endl <<
                            l_proj_mat_.at<double>(2, 0) << " " << l_proj_mat_.at<double>(2, 1) << " " << l_proj_mat_.at<double>(2, 2) << " " << l_proj_mat_.at<double>(2, 3) << std::endl <<
                            "The right projection is as follow:" << std::endl <<
                            r_proj_mat_.at<double>(0, 0) << " " << r_proj_mat_.at<double>(0, 1) << " " << r_proj_mat_.at<double>(0, 2) << " " << r_proj_mat_.at<double>(0, 3) << std::endl <<
                            r_proj_mat_.at<double>(1, 0) << " " << r_proj_mat_.at<double>(1, 1) << " " << r_proj_mat_.at<double>(1, 2) << " " << r_proj_mat_.at<double>(1, 3) << std::endl <<
                            r_proj_mat_.at<double>(2, 0) << " " << r_proj_mat_.at<double>(2, 1) << " " << r_proj_mat_.at<double>(2, 2) << " " << r_proj_mat_.at<double>(2, 3));
        }
        else
        {
            ROS_WARN_STREAM("[LC->Projection matrices:] Failed to call GetProjectionMatrices service. It is assumed that the projection matrices are the identity!");
            l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);
            r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);
        }
        init_loop_closer_ = true;
    }


    void LoopCloser::processNewCluster(Cluster& c_cluster)
    {
        // Get the cluster
        {
            std::scoped_lock lock(mutex_cluster_queue_);
            c_cluster = cluster_queue_.front();
            cluster_queue_.pop_front();
        }

        // Initialise hash
        if (!haloc_.isInitialized())
            haloc_.init(c_cluster.getLeftDescs());

        // Save hash to table
        hash_table_.push_back(std::make_pair(c_cluster.getId(), haloc_.getHash(c_cluster.getLeftDescs())));

        // Store
        cv::FileStorage fs(execution_path_ + convertTo5Digits(c_cluster.getId()) + ".yml", cv::FileStorage::WRITE);
        cv::write(fs, "l_kpts", c_cluster.getLeftKpts());
        cv::write(fs, "r_kpts", c_cluster.getRightKpts());
        cv::write(fs, "l_descs", c_cluster.getLeftDescs());
        cv::write(fs, "r_descs", c_cluster.getRightDescs());
        cv::write(fs, "cam_pts", c_cluster.getCameraPoints());
        cv::write(fs, "keyframe_id", c_cluster.getKeyframeId());
        fs.release();
    }


    // TODO: no_candidates?
    void LoopCloser::searchByProximity(const Cluster& c_cluster)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Get the candidates to close loop
        start_module_time = ros::Time::now().toSec();
        std::vector<int> candidates;
        map_->findClosestVertices(c_cluster.getId(), lc_discard_window_, c_cluster.getId(), lc_candidates_, candidates);
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Search for candidates to close loops by proximity.
        if (candidates.size() == 0)
            return;

        // Loop over candidates.
        Cluster cand_cluster;
        for (uint i = 0; i < candidates.size(); i++)
        {
            // Reads cluster candidate from disk.
            start_module_time = ros::Time::now().toSec();
            cand_cluster = readCluster(candidates[i]);
            if (cand_cluster.getLeftDescs().rows == 0)
            {
                ROS_WARN_STREAM("[LC->searchByProximity->readCluster:] It is not possible to read the cluster " << candidates[i] << " from the disk!");
                continue;
            }
            run_times_.run_time_3 += (ros::Time::now().toSec() - start_module_time); // Read cluster candidate run time.

            // Tries to close loop between current/query cluster and a candidate cluster.
            closeLoopBetween(c_cluster, cand_cluster, "proximity");
        }
    }


    void LoopCloser::searchByHash(const Cluster& c_cluster)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Get the candidates to close loop
        start_module_time = ros::Time::now().toSec();
        std::vector<int> candidates;
        getHashCandidates(c_cluster.getId(), lc_candidates_, candidates);
        run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Search for candidates to close loops by hash.
        if (candidates.size() == 0)
            return;

        // Loop over candidates.
        bool valid;
        Cluster cand_cluster;
        for (uint i = 0; i < candidates.size(); i++)
        {
            // Reads cluster candidate from disk.
            start_module_time = ros::Time::now().toSec();
            cand_cluster = readCluster(candidates[i]);
            if (cand_cluster.getLeftDescs().rows == 0)
            {
                ROS_WARN_STREAM("[LC->searchByHash->readCluster:] It is not possible to read the cluster " << candidates[i] << " from the disk!");
                continue;
            }
            run_times_.run_time_3 += (ros::Time::now().toSec() - start_module_time); // Read cluster candidate run time.

            // Tries to close loop between current/query cluster and a candidate cluster.
            valid = closeLoopBetween(c_cluster, cand_cluster, "hash");
            if (valid)
                break;
        }
    }


    void LoopCloser::getHashCandidates(int c_cluster_id,
                                       uint closest_candidates_num,
                                       std::vector<int>& closest_candidates)
    {
        // Init.
        closest_candidates.clear();

        // Check if enough neighbors.
        if ((int)hash_table_.size() <= lc_discard_window_)
            return;

        // Create a list with the non-possible candidates (because they are already loop closures).
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
        if (closest_candidates_num > candidates.size())
            closest_candidates_num = candidates.size();
        for (uint i = 0; i < closest_candidates_num; i++)
            closest_candidates.push_back(candidates[i].first);
    }


    Cluster LoopCloser::readCluster(int id)
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
        int keyframe_id;
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> cam_pts;
        std::vector<cv::Point2f> l_kpts, r_kpts;
        fs["l_kpts"] >> l_kpts;
        fs["r_kpts"] >> r_kpts;
        fs["cam_pts"] >> cam_pts;
        fs["l_descs"] >> l_descs;
        fs["r_descs"] >> r_descs;
        fs["keyframe_id"] >> keyframe_id;
        fs.release();

        // Sets the properties of the cluster.
        tf::Transform camera_pose = map_->getVertexCameraPose(id, true);
        Cluster cluster(id, keyframe_id, l_descs, r_descs, camera_pose, l_kpts, r_kpts, cam_pts);
        return cluster;
    }


    bool LoopCloser::closeLoopBetween(Cluster q_cluster,
                                      Cluster cand_cluster,
                                      const std::string& search_method)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time = ros::Time::now().toSec();

        // Cluster ids list.
        std::vector<int> q_cluster_ids, cand_cluster_ids, f_q_cluster_ids, f_cand_cluster_ids;

        // First descriptor matching.
        std::vector<cv::Point3f> f_q_cam_pts, f_cand_cam_pts;
        std::vector<cv::Point2f> f_q_l_kpts, f_q_r_kpts, f_cand_l_kpts, f_cand_r_kpts;
        matchNonStereoKeypoints(q_cluster.getLeftDescs(), q_cluster.getRightDescs(), 
                                cand_cluster.getLeftDescs(), cand_cluster.getRightDescs(),
                                q_cluster_ids, cand_cluster_ids, 
                                q_cluster.getLeftKpts(), q_cluster.getRightKpts(),
                                cand_cluster.getLeftKpts(), cand_cluster.getRightKpts(),
                                q_cluster.getCameraPoints(), cand_cluster.getCameraPoints(),
                                f_q_cluster_ids, f_cand_cluster_ids,
                                f_q_l_kpts, f_q_r_kpts, 
                                f_cand_l_kpts, f_cand_r_kpts,
                                f_q_cam_pts, f_cand_cam_pts);

        // Gets the neighbor clusters if enough matching percentage.
        if (f_q_l_kpts.size() >= (uint)(min_lc_inliers_ / 2))
        {
            // Inits accumulated data. Sanity check.
            q_cluster_ids.clear();
            f_q_cluster_ids.clear();
            cand_cluster_ids.clear();
            f_cand_cluster_ids.clear();

            // Inits accumulated data. Candidate data.
            cv::Mat all_cand_l_descs = cand_cluster.getLeftDescs();
            cv::Mat all_cand_r_descs = cand_cluster.getRightDescs();
            std::vector<cv::Point2f> all_cand_l_kpts = cand_cluster.getLeftKpts();
            std::vector<cv::Point2f> all_cand_r_kpts = cand_cluster.getRightKpts();
            std::vector<cv::Point3f> all_cand_world_pts = cand_cluster.getWorldPoints();

            // Inits accumulated data. Query data.
            cv::Mat all_q_l_descs = q_cluster.getLeftDescs();
            cv::Mat all_q_r_descs = q_cluster.getRightDescs();
            std::vector<cv::Point2f> all_q_l_kpts = q_cluster.getLeftKpts();
            std::vector<cv::Point2f> all_q_r_kpts = q_cluster.getRightKpts();
            std::vector<cv::Point3f> all_q_cam_pts = q_cluster.getCameraPoints();

            // Temporal variables.
            Cluster tmp_cluster;
            cv::Mat tmp_l_descs, tmp_r_descs;
            std::vector<cv::Point3f> tmp_3d_pts;
            std::vector<cv::Point2f> tmp_l_kpts, tmp_r_kpts;

            // Inits the cluster candidate list.
            for (uint i = 0; i < all_cand_l_kpts.size(); i++)
                cand_cluster_ids.push_back(cand_cluster.getId());

            // Increases the probability to close loop by extracting the candidate neighbors.
            std::vector<int> cand_cluster_neighbours;
            map_->findClosestVertices(cand_cluster.getId(), lc_discard_window_, q_cluster.getId(), lc_neighbors_, cand_cluster_neighbours);
            for (uint i = 0; i < cand_cluster_neighbours.size(); i++)
            {
                // Reads from the disk the candidate cluster's ith neighbour.
                tmp_cluster = readCluster(cand_cluster_neighbours[i]);

                // Sanity check.
                tmp_l_descs = tmp_cluster.getLeftDescs();
                if (tmp_l_descs.rows == 0)
                    continue;

                // Gets data from the candidate cluster's ith neighbour.
                tmp_l_kpts = tmp_cluster.getLeftKpts();
                tmp_r_kpts = tmp_cluster.getRightKpts();
                tmp_r_descs = tmp_cluster.getRightDescs();
                tmp_3d_pts = tmp_cluster.getWorldPoints();

                // Concatenates candidate data.
                cv::vconcat(all_cand_l_descs, tmp_l_descs, all_cand_l_descs);
                cv::vconcat(all_cand_r_descs, tmp_r_descs, all_cand_r_descs);
                all_cand_l_kpts.insert(all_cand_l_kpts.end(), tmp_l_kpts.begin(), tmp_l_kpts.end());
                all_cand_r_kpts.insert(all_cand_r_kpts.end(), tmp_r_kpts.begin(), tmp_r_kpts.end());
                all_cand_world_pts.insert(all_cand_world_pts.end(), tmp_3d_pts.begin(), tmp_3d_pts.end());

                // Save the candidate cluster's ith neighbour id for these points.
                for (uint j = 0; j < tmp_l_kpts.size(); j++)
                    cand_cluster_ids.push_back(tmp_cluster.getId());

                // Sanity check.
                tmp_l_kpts.clear();
                tmp_r_kpts.clear();
                tmp_3d_pts.clear();
                tmp_l_descs.release();
                tmp_r_descs.release();
            }

            // Inits the query cluster frame list.
            for (uint i = 0; i < all_q_l_kpts.size(); i++)
                q_cluster_ids.push_back(q_cluster.getId());

            // Extracts all the clusters corresponding to the current cluster frame
            std::vector<int> q_cluster_neighbours;
            map_->getKeyframeVertices(q_cluster.getKeyframeId(), q_cluster_neighbours);
            for (uint i = 0; i < q_cluster_neighbours.size(); i++)
            {
                // Avoids itself.
                if (q_cluster.getId() == q_cluster_neighbours[i])
                    continue;

                // Reads from the disk the query cluster's ith neighbour.
                tmp_cluster = readCluster(q_cluster_neighbours[i]);

                // Sanity check.
                tmp_l_descs = tmp_cluster.getLeftDescs();
                if (tmp_l_descs.rows == 0)
                    continue;

                // Gets data from the query cluster's ith neighbour.
                tmp_l_kpts = tmp_cluster.getLeftKpts();
                tmp_r_kpts = tmp_cluster.getRightKpts();
                tmp_r_descs = tmp_cluster.getRightDescs();
                tmp_3d_pts = tmp_cluster.getCameraPoints();

                // Concatenates query data.
                cv::vconcat(all_q_l_descs, tmp_l_descs, all_q_l_descs);
                cv::vconcat(all_q_r_descs, tmp_r_descs, all_q_r_descs);
                all_q_l_kpts.insert(all_q_l_kpts.end(), tmp_l_kpts.begin(), tmp_l_kpts.end());
                all_q_r_kpts.insert(all_q_r_kpts.end(), tmp_r_kpts.begin(), tmp_r_kpts.end());
                all_q_cam_pts.insert(all_q_cam_pts.end(), tmp_3d_pts.begin(), tmp_3d_pts.end());

                // Save the query cluster's ith neighbour id for these points.
                for (uint j = 0; j < tmp_l_kpts.size(); j++)
                    q_cluster_ids.push_back(tmp_cluster.getId());

                // Sanity check.
                tmp_l_kpts.clear();
                tmp_r_kpts.clear();
                tmp_3d_pts.clear();
                tmp_l_descs.release();
                tmp_r_descs.release();
            }

            // Reuses variables.
            f_q_l_kpts.clear();
            f_q_r_kpts.clear();
            f_q_cam_pts.clear();
            f_cand_l_kpts.clear();
            f_cand_r_kpts.clear();
            f_cand_cam_pts.clear();

            // Second descriptor matching.
            std::vector<cv::Point3f> f_cand_world_pts;
            matchNonStereoKeypoints(all_q_l_descs, all_q_r_descs, 
                                    all_cand_l_descs, all_cand_r_descs,
                                    q_cluster_ids, cand_cluster_ids,
                                    all_q_l_kpts, all_q_r_kpts, 
                                    all_cand_l_kpts, all_cand_r_kpts,
                                    all_q_cam_pts, all_cand_world_pts,
                                    f_q_cluster_ids, f_cand_cluster_ids,
                                    f_q_l_kpts, f_q_r_kpts,
                                    f_cand_l_kpts, f_cand_r_kpts,
                                    f_q_cam_pts, f_cand_world_pts);

            if (f_q_l_kpts.size() >= (uint)(min_lc_inliers_))
            {
                // Estimates motion.
                cv::Mat estimated_rotation, estimated_translation;
                std::vector<int> i_q_cluster_ids, i_cand_cluster_ids;
                std::vector<cv::Point3f> i_q_cam_pts, i_cand_world_pts;
                std::vector<cv::Point2f> i_q_l_kpts, i_q_r_kpts, i_cand_l_kpts, i_cand_r_kpts;
                estimateMotion(f_q_cluster_ids, f_cand_cluster_ids, f_q_l_kpts, f_q_r_kpts, f_cand_l_kpts, f_cand_r_kpts, f_q_cam_pts, f_cand_world_pts,
                               estimated_rotation, estimated_translation,
                               i_q_cluster_ids, i_cand_cluster_ids, i_q_l_kpts, i_q_r_kpts, i_cand_l_kpts, i_cand_r_kpts, i_q_cam_pts, i_cand_world_pts);

                // Loop found!
                if (i_q_l_kpts.size() >= (uint)(min_lc_inliers_))
                {
                    // Finished loop detection, starting loop closure.
                    run_times_.run_time_4 += (ros::Time::now().toSec() - start_module_time); // Loop detection run time.
                    start_module_time = ros::Time::now().toSec();

                    // From cv::Mat to tf::Transform.
                    tf::Transform estimated_motion;
                    fromMatToTf(estimated_rotation, estimated_translation, estimated_motion);
                    
                    // From {q_cam}^T_{O} to {O}^T_{q_cam}.
                    estimated_motion = estimated_motion.inverse();

                    // Gets the inliers per cluster pair.
                    std::pair<int, int> relation;
                    bool cluster_pair_found, cand_keyframe_found;
                    std::vector<int> inliers_per_pair, cand_keyframe_ids;
                    std::vector<std::pair<int, int>> cluster_pairs, cand_cluster_keyframe_relation;
                    int q_cluster_id, cand_cluster_id, cand_keyframe_id, last_keyframe_id = -1, last_cluster_id = -1;
                    for (uint i = 0; i < i_q_l_kpts.size(); i++)
                    {
                        q_cluster_id = i_q_cluster_ids[i];
                        cand_cluster_id = i_cand_cluster_ids[i];
                        cand_keyframe_id = map_->getVertexKeyframeId(cand_cluster_id);

                        // Builds the unique candidate keyframe vector.
                        cand_keyframe_found = false;
                        for (uint j = 0; j < cand_keyframe_ids.size(); j++)
                        {
                            if (cand_keyframe_ids[j] == cand_keyframe_id)
                            {
                                cand_keyframe_found = true;
                                break;
                            }
                        }
                        if (!cand_keyframe_found)
                            cand_keyframe_ids.push_back(cand_keyframe_id);

                        // Builds candidate keyframe-cluster relation.
                        if ((last_keyframe_id != cand_keyframe_id) || (last_cluster_id != cand_cluster_id))
                        {
                            last_cluster_id = cand_cluster_id;
                            last_keyframe_id = cand_keyframe_id;
                            cand_cluster_keyframe_relation.push_back(std::make_pair(cand_cluster_id, cand_keyframe_id));
                        }

                        // Searches if this pair already exists.
                        cluster_pair_found = false;
                        for (uint j = 0; j < cluster_pairs.size(); j++)
                        {
                            if ((cluster_pairs[j].first == q_cluster_id) && (cluster_pairs[j].second == cand_cluster_id))
                            {
                                inliers_per_pair[j]++;
                                cluster_pair_found = true;
                                break;
                            }
                        }
                        if (!cluster_pair_found)
                        {
                            cluster_pairs.push_back(std::make_pair(q_cluster_id, cand_cluster_id));
                            inliers_per_pair.push_back(1);
                        }
                    }

                    // Adds the corresponding edges.
                    bool lc_already_exists, some_edge_added = false;
                    std::vector<int> definitive_inliers_per_pair;
                    std::vector<std::pair<int, int>> definitive_cluster_pairs; // q_cluster_id - cand_cluster_id
                    tf::Transform edge, cand_cluster_pose, q_cluster_pose_relative_to_camera;
                    for (uint i = 0; i < inliers_per_pair.size(); i++)
                    {
                        if (inliers_per_pair[i] >= 5)
                        {
                            // Checks if this loop closure exists.
                            lc_already_exists = false;
                            for (uint j = 0; j < cluster_lc_found_.size(); j++)
                            {
                                if ((cluster_lc_found_[j].first == cluster_pairs[i].first && cluster_lc_found_[j].second == cluster_pairs[i].second) ||
                                    (cluster_lc_found_[j].first == cluster_pairs[i].second && cluster_lc_found_[j].second == cluster_pairs[i].first))
                                {
                                    lc_already_exists = true;
                                    break;
                                }
                            }
                            if (lc_already_exists)
                                continue;
                            
                            // Computes correct transform between edges.
                            cand_cluster_pose = map_->getVertexPose(cluster_pairs[i].second, true); // {O}^T_{cand_cluster}
                            q_cluster_pose_relative_to_camera = map_->getVertexPoseRelativeToCamera(cluster_pairs[i].first); // {q_cam}^T_{q_cluster}
                            edge = cand_cluster_pose.inverse() * estimated_motion * q_cluster_pose_relative_to_camera; // {cand_cluster}^T_{q_cluster} = {cand_cluster}^T_{O} * {O}^T_{q_cam} * {q_cam}^T_{q_cluster}

                            // Adds this edge to the graph.
                            some_edge_added = true;
                            map_->addEdge(cluster_pairs[i].second, cluster_pairs[i].first, edge);                           
                            definitive_inliers_per_pair.push_back(inliers_per_pair[i]);
                            definitive_cluster_pairs.push_back(std::make_pair(cluster_pairs[i].first, cluster_pairs[i].second)); // q_cluster_id - cand_cluster_id

                            // Add this edge to the list of clusters that close loops.
                            cluster_lc_found_.push_back(std::make_pair(cluster_pairs[i].first, cluster_pairs[i].second));
                        }
                    }

                    if (some_edge_added)
                    {
                        map_->update();
                        run_times_.run_time_5 += (ros::Time::now().toSec() - start_module_time); // Close loop run time.

                        ROS_INFO_STREAM("[LC->closeLoopBetween:] ------------------------------");
                        ROS_INFO_STREAM("                      LOOP CLOSURE: " << loop_closure_num_);
                        ROS_INFO_STREAM("Between keyframe " << q_cluster.getKeyframeId() << " and " << cand_cluster.getKeyframeId());
                        ROS_INFO_STREAM("Method: " << search_method);
                        ROS_INFO_STREAM("Inliers: " << i_q_l_kpts.size());
                        ROS_INFO_STREAM("[LC->closeLoopBetween:] ------------------------------");

                        if (enable_debugging_)
                        {
                            if (loop_closure_img_pub_.getNumSubscribers() > 0)
                                publishLoopClosure(ros::Time::now(), 
                                                   loop_closure_img_pub_,
                                                   q_cluster.getKeyframeId(),
                                                   keyframe_path_,
                                                   i_q_cluster_ids,
                                                   i_cand_cluster_ids,
                                                   cand_keyframe_ids,
                                                   i_q_l_kpts,
                                                   i_q_r_kpts,
                                                   i_cand_l_kpts,
                                                   i_cand_r_kpts,
                                                   definitive_inliers_per_pair,
                                                   definitive_cluster_pairs,
                                                   cand_cluster_keyframe_relation);

                            writeLoopClosure(lc_path_ + convertTo5Digits(loop_closure_num_),
                                             q_cluster.getKeyframeId(),
                                             keyframe_path_,
                                             i_q_cluster_ids,
                                             i_cand_cluster_ids,
                                             cand_keyframe_ids,
                                             i_q_l_kpts,
                                             i_q_r_kpts,
                                             i_cand_l_kpts,
                                             i_cand_r_kpts,
                                             definitive_inliers_per_pair,
                                             definitive_cluster_pairs,
                                             cand_cluster_keyframe_relation);
                        }
                        loop_closure_num_++;
                        return true;
                    }
                    else
                    {
                        run_times_.run_time_5 += (ros::Time::now().toSec() - start_module_time); // Close loop run time.
                    }
                }
            }
        }
        run_times_.run_time_4 += (ros::Time::now().toSec() - start_module_time); // Loop detection run time.
        return false;  
    }


} // End namespace