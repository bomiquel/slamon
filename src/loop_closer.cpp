/**
 *  @file loop_closer.cpp
 *  @brief The LoopCloser class detects and closes loops between keypoint 
 *         clusters from multiple keyframes. 
 *  @package slamon
 *  @project SLAMON
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  @author
 *    Original:  Pep Lluís Negre-Carrasco (2013)
 *    Modified:  Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    Both the original work and the modifications are released under the
 *    BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#include "loop_closer.hpp"


namespace slamon
{


    LoopCloser::LoopCloser(Map* map,
                           const int lc_neighbors,
                           const int lc_candidates,
                           const int min_lc_inliers,
                           const int lc_discard_window,
                           const bool enable_debugging,
                           Queue<Cluster>* cluster_queue,
                           const std::string& w_path,
                           const std::string& matching_type,
                           const std::string& motion_estimation_type):
                           KeypointMatcher(matching_type), 
                           MotionEstimator(motion_estimation_type),
                           enable_debugging_{enable_debugging}, init_loop_closer_{false}, lc_neighbors_{lc_neighbors},
                           lc_candidates_{lc_candidates}, min_lc_inliers_{min_lc_inliers}, lc_discard_window_{lc_discard_window},
                           loop_closure_num_{0}, map_{map}, cluster_queue_{cluster_queue},
                           nh_{}, nhp_{"~"}
    {
        // Computes maximum inliers for edges.
        max_edge_inliers_ = min_lc_inliers * 2;

        // Creates execution directory
        execution_path_ = w_path + "haloc/";
        if (std::filesystem::exists(execution_path_))
            std::filesystem::remove_all(execution_path_);
        std::filesystem::create_directory(execution_path_);

        // Creates debugging directories.
        if (enable_debugging_)
        {
            keyframe_path_ = w_path + "keyframes/";
            
            lc_path_ = w_path + "loop_closures/";
            if (std::filesystem::exists(lc_path_))
                std::filesystem::remove_all(lc_path_);
            std::filesystem::create_directory(lc_path_);
        }

        // ROS client services.
        proj_mat_client_ = nh_.serviceClient<slamon::GetProjectionMatrices>("get_projection_matrices");

        // Publishers.
        run_times_pub_ = nhp_.advertise<slamon::Performance>("loop_closer_performance", 1);
        cluster_num_pub_ = nhp_.advertise<std_msgs::Int32>("cluster_num", 1);
        loop_closure_num_pub_ = nhp_.advertise<std_msgs::Int32>("loop_closure_num", 1);
        loop_closure_img_pub_ = nhp_.advertise<sensor_msgs::Image>("loop_closure_img", 1);
        loop_closure_matches_num_pub_ = nhp_.advertise<std_msgs::Int32>("loop_closure_matches_num", 1);
        loop_closure_inliers_num_pub_ = nhp_.advertise<std_msgs::Int32>("loop_closure_inliers_num", 1);
    }


    void LoopCloser::run()
    {   
        // Loop
        Cluster q_cluster;
        int p_loop_closures_num;
        double start_module_time;
        double start_iteration_time;
        while (ros::ok())
        {
            // Search for a loop closure in the new cluster.
            if (!cluster_queue_->isEmpty())
            {
                // Time at which the program starts the iteration.
                start_iteration_time = ros::Time::now().toSec();

                // Initialisation of the variables necessary to measure thread performance.
                run_times_.reset();
                p_loop_closures_num = loop_closure_num_;

                if (!init_loop_closer_)
                    getProjectionMatrices(l_proj_mat_, r_proj_mat_);

                // Process new cluster.
                start_module_time = ros::Time::now().toSec();
                q_cluster = cluster_queue_->getFront();
                processNewCluster(q_cluster);
                run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Process current cluster run time.

                if (enable_debugging_)
                    ROS_INFO_STREAM("[LC->run]: Cluster: " << q_cluster.getGlobalId() << " from keyframe: " << q_cluster.getKeyframeId());

                if (cluster_num_pub_.getNumSubscribers() > 0)
                    publishCounter(q_cluster.getGlobalId() + 1, cluster_num_pub_);
                
                // Candidates.
                std::vector<std::pair<Cluster, std::string>> cand_clusters;
                std::vector<std::vector<Cluster>> neighbours_cand_clusters;

                // Searches by proximity candidates to close loops.
                start_module_time = ros::Time::now().toSec();
                std::vector<int> cand_cluster_ids_by_proximity;
                map_->findClosestVertices(q_cluster.getGlobalId(), lc_discard_window_, q_cluster.getGlobalId(), lc_candidates_, cand_cluster_ids_by_proximity);
                run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Search for candidates to close loops by proximity.
                start_module_time = ros::Time::now().toSec();
                readCandidateClusters("proximity", cand_cluster_ids_by_proximity, cand_clusters, neighbours_cand_clusters);
                run_times_.run_time_3 += (ros::Time::now().toSec() - start_module_time); // Read cluster candidate run time.

                // Searches by hash candidates to close loops.
                start_module_time = ros::Time::now().toSec();
                std::vector<int> cand_cluster_ids_by_hash;
                getHashCandidates(q_cluster.getGlobalId(), lc_candidates_, cand_cluster_ids_by_proximity, cand_cluster_ids_by_hash);
                run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Search for candidates to close loops by hash.
                start_module_time = ros::Time::now().toSec();
                readCandidateClusters("hash", cand_cluster_ids_by_hash, cand_clusters, neighbours_cand_clusters);
                run_times_.run_time_3 += (ros::Time::now().toSec() - start_module_time); // Read cluster candidate run time.

                // Loop over candidates.
                tf::Transform estimated_motion;
                bool loop_detected = false, loop_closed = false;
                std::vector<int> i_q_cluster_ids, i_cand_cluster_ids;
                std::vector<cv::KeyPoint> i_q_l_kpts, i_q_r_kpts, i_cand_l_kpts, i_cand_r_kpts;
                for (size_t i = 0; i < cand_clusters.size(); ++i)
                {
                    // Tries to detect a loop between query cluster and a candidate cluster.
                    start_module_time = ros::Time::now().toSec();
                    loop_detected = detectLoop(q_cluster,
                                               cand_clusters[i].first,
                                               neighbours_cand_clusters[i],
                                               estimated_motion,
                                               i_q_cluster_ids,
                                               i_cand_cluster_ids,
                                               i_q_l_kpts,
                                               i_q_r_kpts,
                                               i_cand_l_kpts,
                                               i_cand_r_kpts);
                    run_times_.run_time_4 += (ros::Time::now().toSec() - start_module_time); // Loop detection run time.

                    // Tries to add edges and correct execute a pose-graph optimization.
                    start_module_time = ros::Time::now().toSec();
                    if (loop_detected)
                    {
                        loop_closed = loopCorrection(q_cluster.getKeyframeId(),
                                                     cand_clusters[i].first.getGlobalId(),
                                                     cand_clusters[i].second,
                                                     estimated_motion,
                                                     i_q_cluster_ids,
                                                     i_cand_cluster_ids,
                                                     i_q_l_kpts,
                                                     i_q_r_kpts,
                                                     i_cand_l_kpts,
                                                     i_cand_r_kpts);
                    }
                    run_times_.run_time_5 += (ros::Time::now().toSec() - start_module_time); // Loop correction run time.

                    if (loop_closed && cand_clusters[i].second == "hash")
                        break;

                    // Clear
                    i_q_l_kpts.clear();
                    i_q_r_kpts.clear();
                    loop_closed = false;
                    loop_detected = false;
                    i_cand_l_kpts.clear();
                    i_cand_r_kpts.clear();
                    i_q_cluster_ids.clear();
                    i_cand_cluster_ids.clear();
                    estimated_motion.setIdentity();
                }
                p_cand_clusters_ = cand_clusters;
                p_q_cluster_neighbours_.emplace_back(q_cluster);
                p_cands_cluster_neighbours_ = neighbours_cand_clusters;

                if (loop_closure_num_pub_.getNumSubscribers() > 0)
                    publishCounter(loop_closure_num_, loop_closure_num_pub_);

                // Publishes last iteration performance.
                run_times_.total_run_time = ros::Time::now().toSec() - start_iteration_time;
                if (run_times_pub_.getNumSubscribers() > 0)
                    publishPerformance(ros::Time::now(), loop_closure_num_ - p_loop_closures_num, run_times_, run_times_pub_);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }


    void LoopCloser::getProjectionMatrices(cv::Mat& l_proj_mat,
                                           cv::Mat& r_proj_mat)
    {
        slamon::GetProjectionMatrices srv;
        if (proj_mat_client_.call(srv))
        {
            // Gets projection matrix
            l_proj_mat = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(srv.response.l_proj_mat.data())).clone();
            r_proj_mat = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(srv.response.r_proj_mat.data())).clone();

            ROS_WARN_STREAM("[LC->Projection matrices:] GetProjectionMatrices service call successful." << std::endl <<
                            "The left projection is as follow:" << std::endl <<
                            l_proj_mat.at<double>(0, 0) << " " << l_proj_mat.at<double>(0, 1) << " " << l_proj_mat.at<double>(0, 2) << " " << l_proj_mat.at<double>(0, 3) << std::endl <<
                            l_proj_mat.at<double>(1, 0) << " " << l_proj_mat.at<double>(1, 1) << " " << l_proj_mat.at<double>(1, 2) << " " << l_proj_mat.at<double>(1, 3) << std::endl <<
                            l_proj_mat.at<double>(2, 0) << " " << l_proj_mat.at<double>(2, 1) << " " << l_proj_mat.at<double>(2, 2) << " " << l_proj_mat.at<double>(2, 3) << std::endl <<
                            "The right projection is as follow:" << std::endl <<
                            r_proj_mat.at<double>(0, 0) << " " << r_proj_mat.at<double>(0, 1) << " " << r_proj_mat.at<double>(0, 2) << " " << r_proj_mat.at<double>(0, 3) << std::endl <<
                            r_proj_mat.at<double>(1, 0) << " " << r_proj_mat.at<double>(1, 1) << " " << r_proj_mat.at<double>(1, 2) << " " << r_proj_mat.at<double>(1, 3) << std::endl <<
                            r_proj_mat.at<double>(2, 0) << " " << r_proj_mat.at<double>(2, 1) << " " << r_proj_mat.at<double>(2, 2) << " " << r_proj_mat.at<double>(2, 3));
        }
        else
        {
            ROS_WARN_STREAM("[LC->Projection matrices:] Failed to call GetProjectionMatrices service. It is assumed that the projection matrices are the identity!");
            l_proj_mat = cv::Mat::eye(3, 4, CV_64FC1);
            r_proj_mat = cv::Mat::eye(3, 4, CV_64FC1);
        }
        init_loop_closer_ = true;
    }


    void LoopCloser::processNewCluster(const Cluster& q_cluster)
    {
        // Initialise hash.
        if (!haloc_.isInitialized())
            haloc_.init(q_cluster.getLeftDescs());

        // Clears the vector of previous clusters if the current cluster belongs to a new frame.
        if (q_cluster.getLocalId() == 0)
            p_q_cluster_neighbours_.clear();

        // Create a locking element.
        std::string cluster_path = execution_path_ + convertTo5Digits(q_cluster.getGlobalId()) + ".yml";
        std::string lock_path = cluster_path + ".lock";
        std::fstream lock_file(lock_path.c_str(), std::ios::out | std::ios::trunc);

        // Store.
        cv::FileStorage fs(cluster_path, cv::FileStorage::WRITE);
        cv::write(fs, "l_kpts", q_cluster.getLeftKpts());
        cv::write(fs, "r_kpts", q_cluster.getRightKpts());
        cv::write(fs, "l_descs", q_cluster.getLeftDescs());
        cv::write(fs, "r_descs", q_cluster.getRightDescs());
        cv::write(fs, "keyframe_id", q_cluster.getKeyframeId());
        cv::write(fs, "l_cam_pts", q_cluster.getLeftCameraPoints());
        fs.release();

        // Unlock.
        bool ok = std::filesystem::remove(lock_path.c_str());
        if (!ok)
            ROS_ERROR("[LC->processNewCluster]: Error deleting the locking file!");

        // Save hash to table.
        hash_table_.emplace_back(q_cluster.getGlobalId(), haloc_.getHash(q_cluster.getLeftDescs()));
    }


    void LoopCloser::getHashCandidates(const int q_cluster_id,
                                       const size_t cand_num,
                                       std::vector<int>& no_cand,
                                       std::vector<int>& closest_cand)
    {
        // Init.
        closest_cand.clear();

        // Check if enough neighbors.
        if (hash_table_.size() <= static_cast<size_t>(lc_discard_window_))
            return;

        // Fills the non-candidates list with those clusters that have already closed the loop.
        for (size_t i = 0; i < cluster_lc_found_.size(); ++i)
        {
            if (cluster_lc_found_[i].first == q_cluster_id)
                no_cand.push_back(cluster_lc_found_[i].second);
            if (cluster_lc_found_[i].second == q_cluster_id)
                no_cand.push_back(cluster_lc_found_[i].first);
        }

        // Query hash.
        std::vector<float> q_hash = hash_table_[q_cluster_id].second;

        // Loop over all the hashes stored.
        int cand_id;
        float match_dist;
        std::vector<float> cand_hash;
        std::vector<std::pair<int, float>> candidates;
        const int lower_limit = q_cluster_id - lc_discard_window_;
        const int upper_limit = q_cluster_id + lc_discard_window_;
        for (size_t i = 0; i < hash_table_.size(); ++i)
        {
            cand_id = hash_table_[i].first;

            // Do not compute the hash matching with itself.
            if (cand_id == q_cluster_id)
                continue;

            // Discards window.
            if ((cand_id > lower_limit) && (cand_id < upper_limit))
                continue;

            // Continue if candidate is in the no_cand list.
            if (std::find(no_cand.begin(), no_cand.end(), hash_table_[i].first) != no_cand.end())
                continue;

            // Hash matching.
            cand_hash = hash_table_[i].second;
            match_dist = haloc_.match(q_hash, cand_hash);
            candidates.emplace_back(cand_id, match_dist);
        }

        // Exit if no candidates.
        if (candidates.empty())
            return;

        // Sorts the candidates from least to furthest matching distance to the query neighbour.
        std::sort(candidates.begin(), candidates.end(), sortByMatching);

        // Gets closest matches (minnor matching distance).
        const size_t final_cand_num = std::min(cand_num, candidates.size());
        for (size_t i = 0; i < final_cand_num; ++i)
            closest_cand.emplace_back(candidates[i].first);
    }


    Cluster LoopCloser::readCluster(const int id)
    {
        Cluster tmp_cluster;
        std::string cluster_path = execution_path_ + convertTo5Digits(id) + ".yml";

        // Sanity check
        if (!std::filesystem::exists(cluster_path))
            return tmp_cluster;

        // Wait until lock file has been released.
        std::string lock_path = cluster_path + ".lock";
        while (std::filesystem::exists(lock_path));

        // Opens cluster file.
        cv::FileStorage fs;
        fs.open(cluster_path, cv::FileStorage::READ);

        // Sanity check x2
        if (!fs.isOpened())
            return tmp_cluster;

        // Reads cluster data.
        int keyframe_id;
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> l_cam_pts;
        std::vector<cv::KeyPoint> l_kpts, r_kpts;
        fs["l_kpts"] >> l_kpts;
        fs["r_kpts"] >> r_kpts;
        fs["l_descs"] >> l_descs;
        fs["r_descs"] >> r_descs;
        fs["l_cam_pts"] >> l_cam_pts;
        fs["keyframe_id"] >> keyframe_id;
        fs.release();

        // Sets the properties of the cluster.
        tf::Transform camera_map_pose = map_->getVertexCameraPose(id, true);
        Cluster cluster(id, keyframe_id, l_descs, r_descs, camera_map_pose, l_kpts, r_kpts, l_cam_pts);
        return cluster;
    }


    void LoopCloser::readCandidateClusters(const std::string& search_type,
                                           const std::vector<int>& cand_cluster_ids,
                                           std::vector<std::pair<Cluster, std::string>>& cand_clusters,
                                           std::vector<std::vector<Cluster>>& neighbours_cand_clusters)
    {
        bool found;
        for (size_t i = 0; i < cand_cluster_ids.size(); ++i)
        {
            found = false;

            // Checks if the candidate exists with all its neighbors.
            for (size_t j = 0; j < p_cand_clusters_.size(); ++j)
            {
                if ((cand_cluster_ids[i] == p_cand_clusters_[j].first.getGlobalId()))
                {
                    updateClusterPose(p_cand_clusters_[j].first);
                    cand_clusters.emplace_back(p_cand_clusters_[j].first, search_type);
                    for (size_t k = 0; k < p_cands_cluster_neighbours_[j].size(); ++k)
                        updateClusterPose(p_cands_cluster_neighbours_[j][k]);
                    neighbours_cand_clusters.emplace_back(p_cands_cluster_neighbours_[j]);
                    found = true;
                    break;
                }
            }

            // Checks if the candidate exists among the neighbors.
            for (size_t j = 0; j < p_cands_cluster_neighbours_.size(); ++j)
            {
                for (size_t k = 0; k < p_cands_cluster_neighbours_[j].size(); ++k)
                {
                    if (cand_cluster_ids[i] == p_cands_cluster_neighbours_[j][k].getGlobalId())
                    {
                        updateClusterPose(p_cands_cluster_neighbours_[j][k]);
                        cand_clusters.emplace_back(p_cands_cluster_neighbours_[j][k], search_type);
                        neighbours_cand_clusters.emplace_back();
                        found = true;
                        break;
                    }
                }
                if (found)
                    break;
            }

            // Reads if there is no alternative.
            if (!found)
            {
                Cluster tmp_cluster = readCluster(cand_cluster_ids[i]);

                // Sanity check.
                std::vector<cv::KeyPoint> tmp_l_kpts = tmp_cluster.getLeftKpts();
                if (tmp_l_kpts.empty())
                {
                    ROS_WARN_STREAM("[LC->readCandidateClusters:] It is not possible to read the cluster " << cand_cluster_ids[i] << " from the disk!");
                    continue;
                }

                cand_clusters.emplace_back(tmp_cluster, search_type);
                neighbours_cand_clusters.emplace_back();
            }
        }
    }


    Cluster LoopCloser::readCandidateNeighborsClusters(const int neighbor_id,
                                                       std::vector<Cluster>& neighbours_cand_cluster)
    {
        // Searches among current neighbors
        for (size_t i = 0; i < neighbours_cand_cluster.size(); ++i)
        {
            if (neighbor_id == neighbours_cand_cluster[i].getGlobalId())
            {
                updateClusterPose(neighbours_cand_cluster[i]);
                return neighbours_cand_cluster[i];
            }
        }

        // Expands the search
        for (size_t i = 0; i < p_cand_clusters_.size(); ++i)
        {
            if (neighbor_id == p_cand_clusters_[i].first.getGlobalId())
            {
                updateClusterPose(p_cand_clusters_[i].first);
                neighbours_cand_cluster.emplace_back(p_cand_clusters_[i].first);
                return p_cand_clusters_[i].first;
            }
            for (size_t j = 0; j < p_cands_cluster_neighbours_[i].size(); ++j)
            {
                if (neighbor_id == p_cands_cluster_neighbours_[i][j].getGlobalId())
                {
                    updateClusterPose(p_cands_cluster_neighbours_[i][j]);
                    neighbours_cand_cluster.emplace_back(p_cands_cluster_neighbours_[i][j]);
                    return p_cands_cluster_neighbours_[i][j];
                }
            }
        }

        // Reads from disk as a last resort.
        Cluster tmp_cluster = readCluster(neighbor_id);
        neighbours_cand_cluster.emplace_back(tmp_cluster);
        return tmp_cluster;
    }


    void LoopCloser::updateClusterPose(Cluster& cluster)
    {
        tf::Transform camera_map_pose = map_->getVertexCameraPose(cluster.getGlobalId(), true);
        cluster.updateCameraPose(camera_map_pose);
    }


    bool LoopCloser::detectLoop(Cluster& q_cluster,
                                Cluster& cand_cluster,
                                std::vector<Cluster>& neighbours_cand_cluster,
                                tf::Transform& estimated_motion,
                                std::vector<int>& i_q_cluster_ids,
                                std::vector<int>& i_cand_cluster_ids,
                                std::vector<cv::KeyPoint>& i_q_l_kpts,
                                std::vector<cv::KeyPoint>& i_q_r_kpts,
                                std::vector<cv::KeyPoint>& i_cand_l_kpts,
                                std::vector<cv::KeyPoint>& i_cand_r_kpts)
    {
        // Cluster ids lists.
        std::vector<int> q_cluster_ids, cand_cluster_ids, f_q_cluster_ids, f_cand_cluster_ids;

        // First descriptor matching.
        std::vector<cv::Point3f> f_q_l_cam_pts, f_cand_l_cam_pts;
        std::vector<cv::KeyPoint> f_q_l_kpts, f_q_r_kpts, f_cand_l_kpts, f_cand_r_kpts;
        matchTemporalClusters(q_cluster.getLeftDescs(), q_cluster.getRightDescs(), 
                              cand_cluster.getLeftDescs(), cand_cluster.getRightDescs(),
                              q_cluster_ids, cand_cluster_ids, 
                              q_cluster.getLeftKpts(), q_cluster.getRightKpts(),
                              cand_cluster.getLeftKpts(), cand_cluster.getRightKpts(),
                              q_cluster.getLeftCameraPoints(), cand_cluster.getLeftCameraPoints(),
                              f_q_cluster_ids, f_cand_cluster_ids,
                              f_q_l_kpts, f_q_r_kpts, 
                              f_cand_l_kpts, f_cand_r_kpts,
                              f_q_l_cam_pts, f_cand_l_cam_pts);

        // Gets the neighbor clusters if enough matching percentage.
        if (f_q_l_kpts.size() >= static_cast<size_t>(min_lc_inliers_ / 2))
        {
            // Inits accumulated data. Sanity check.
            q_cluster_ids.clear();
            f_q_cluster_ids.clear();
            cand_cluster_ids.clear();
            f_cand_cluster_ids.clear();

            // Temporal variables.
            size_t tmp_size;
            Cluster tmp_cluster;
            cv::Mat tmp_l_descs, tmp_r_descs;
            std::vector<cv::Point3f> tmp_3d_pts;
            std::vector<cv::KeyPoint> tmp_l_kpts, tmp_r_kpts;

            // Inits accumulated data. Candidate data.
            cv::Mat all_cand_l_descs = cand_cluster.getLeftDescs();
            cv::Mat all_cand_r_descs = cand_cluster.getRightDescs();
            std::vector<cv::KeyPoint> all_cand_l_kpts = cand_cluster.getLeftKpts();
            std::vector<cv::KeyPoint> all_cand_r_kpts = cand_cluster.getRightKpts();
            std::vector<cv::Point3f> all_cand_world_pts = cand_cluster.getWorldPoints();

            // Inits accumulated data. Query data.
            cv::Mat all_q_l_descs = q_cluster.getLeftDescs();
            cv::Mat all_q_r_descs = q_cluster.getRightDescs();
            std::vector<cv::KeyPoint> all_q_l_kpts = q_cluster.getLeftKpts();
            std::vector<cv::KeyPoint> all_q_r_kpts = q_cluster.getRightKpts();
            std::vector<cv::Point3f> all_q_l_cam_pts = q_cluster.getLeftCameraPoints();

            // Inits the cluster candidate list.
            tmp_size = all_cand_l_kpts.size();
            cand_cluster_ids.reserve(tmp_size);
            for (size_t i = 0; i < tmp_size; ++i)
                cand_cluster_ids.push_back(cand_cluster.getGlobalId());

            // Inits the query cluster frame list.
            tmp_size = all_q_l_kpts.size();
            q_cluster_ids.reserve(tmp_size);
            for (size_t i = 0; i < tmp_size; ++i)
                q_cluster_ids.push_back(q_cluster.getGlobalId());

            // Increases the probability to close loop by extracting the candidate neighbors.
            std::vector<int> cand_cluster_neighbours;
            map_->findClosestVertices(cand_cluster.getGlobalId(), lc_discard_window_, q_cluster.getGlobalId(), lc_neighbors_, cand_cluster_neighbours);
            for (size_t i = 0; i < cand_cluster_neighbours.size(); ++i)
            {
                // Checks if the cluster has been read in the last iteration and if not, reads from disk.
                tmp_cluster = readCandidateNeighborsClusters(cand_cluster_neighbours[i], neighbours_cand_cluster);

                // Sanity check.
                tmp_l_kpts = tmp_cluster.getLeftKpts();
                if (tmp_l_kpts.empty())
                {
                    ROS_WARN_STREAM("[LC->closeLoopBetween:] It is not possible to read the cluster " << cand_cluster_neighbours[i] << " from the disk!");
                    continue;
                }

                // Gets data from the candidate cluster's ith neighbour.
                tmp_r_kpts = tmp_cluster.getRightKpts();
                tmp_l_descs = tmp_cluster.getLeftDescs();
                tmp_r_descs = tmp_cluster.getRightDescs();
                tmp_3d_pts = tmp_cluster.getWorldPoints();

                // Pre-allocates memory.
                tmp_size = tmp_l_kpts.size();
                all_cand_l_kpts.reserve(all_cand_l_kpts.size() + tmp_size);
                all_cand_r_kpts.reserve(all_cand_r_kpts.size() + tmp_size);
                all_cand_world_pts.reserve(all_cand_world_pts.size() + tmp_size);
                cand_cluster_ids.reserve(cand_cluster_ids.size() + tmp_size);

                // Concatenates candidate data.
                cv::vconcat(all_cand_l_descs, tmp_l_descs, all_cand_l_descs);
                cv::vconcat(all_cand_r_descs, tmp_r_descs, all_cand_r_descs);
                all_cand_l_kpts.insert(all_cand_l_kpts.end(), tmp_l_kpts.begin(), tmp_l_kpts.end());
                all_cand_r_kpts.insert(all_cand_r_kpts.end(), tmp_r_kpts.begin(), tmp_r_kpts.end());
                all_cand_world_pts.insert(all_cand_world_pts.end(), tmp_3d_pts.begin(), tmp_3d_pts.end());

                // Save the candidate cluster's ith neighbour id for these points.
                for (size_t j = 0; j < tmp_size; ++j)
                    cand_cluster_ids.push_back(tmp_cluster.getGlobalId());

                // Sanity check.
                tmp_l_kpts.clear();
                tmp_r_kpts.clear();
                tmp_3d_pts.clear();
                tmp_l_descs.release();
                tmp_r_descs.release();
            }

            // Retrieve all processed clusters corresponding to the current cluster frame.
            for (size_t i = 0; i < p_q_cluster_neighbours_.size(); ++i)
            {
                // Just in case, avoid itself.
                if (q_cluster.getGlobalId() == p_q_cluster_neighbours_[i].getGlobalId())
                    continue;

                // Sanity check.
                tmp_l_kpts = p_q_cluster_neighbours_[i].getLeftKpts();
                if (tmp_l_kpts.empty())
                {
                    ROS_WARN_STREAM("[-LC->closeLoopBetween:] Problems with query neighbours.");
                    continue;
                }

                // Gets data from the query cluster's ith neighbour.
                tmp_r_kpts = p_q_cluster_neighbours_[i].getRightKpts();
                tmp_l_descs = p_q_cluster_neighbours_[i].getLeftDescs();
                tmp_r_descs = p_q_cluster_neighbours_[i].getRightDescs();
                tmp_3d_pts = p_q_cluster_neighbours_[i].getLeftCameraPoints();

                // Pre-allocates memory.
                tmp_size = tmp_l_kpts.size();
                all_q_l_kpts.reserve(all_q_l_kpts.size() + tmp_size);
                all_q_r_kpts.reserve(all_q_r_kpts.size() + tmp_size);
                all_q_l_cam_pts.reserve(all_q_l_cam_pts.size() + tmp_size);
                q_cluster_ids.reserve(q_cluster_ids.size() + tmp_size);

                // Concatenates query data.
                cv::vconcat(all_q_l_descs, tmp_l_descs, all_q_l_descs);
                cv::vconcat(all_q_r_descs, tmp_r_descs, all_q_r_descs);
                all_q_l_kpts.insert(all_q_l_kpts.end(), tmp_l_kpts.begin(), tmp_l_kpts.end());
                all_q_r_kpts.insert(all_q_r_kpts.end(), tmp_r_kpts.begin(), tmp_r_kpts.end());
                all_q_l_cam_pts.insert(all_q_l_cam_pts.end(), tmp_3d_pts.begin(), tmp_3d_pts.end());

                // Save the query cluster's ith neighbour id for these points.
                for (size_t j = 0; j < tmp_size; ++j)
                    q_cluster_ids.push_back(p_q_cluster_neighbours_[i].getGlobalId());

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
            f_q_l_cam_pts.clear();
            f_cand_l_kpts.clear();
            f_cand_r_kpts.clear();

            // Second descriptor matching.
            std::vector<cv::Point3f> f_cand_world_pts;
            matchTemporalClusters(all_q_l_descs, all_q_r_descs, 
                                  all_cand_l_descs, all_cand_r_descs,
                                  q_cluster_ids, cand_cluster_ids,
                                  all_q_l_kpts, all_q_r_kpts, 
                                  all_cand_l_kpts, all_cand_r_kpts,
                                  all_q_l_cam_pts, all_cand_world_pts,
                                  f_q_cluster_ids, f_cand_cluster_ids,
                                  f_q_l_kpts, f_q_r_kpts,
                                  f_cand_l_kpts, f_cand_r_kpts,
                                  f_q_l_cam_pts, f_cand_world_pts);

            // Loop closure matches debugging.
            if (loop_closure_matches_num_pub_.getNumSubscribers() > 0)
                publishCounter(f_q_l_kpts.size(), loop_closure_matches_num_pub_);

            if (f_q_l_kpts.size() >= static_cast<size_t>(min_lc_inliers_))
            {
                // Estimates motion.
                cv::Mat estimated_rotation, estimated_translation;
                std::vector<cv::Point3f> i_q_l_cam_pts, i_cand_world_pts;
                estimateRefinedMotion(l_proj_mat_, r_proj_mat_,
                                      f_q_cluster_ids, f_cand_cluster_ids, f_q_l_kpts, f_q_r_kpts, f_cand_l_kpts, f_cand_r_kpts, f_q_l_cam_pts, f_cand_world_pts,
                                      estimated_rotation, estimated_translation,
                                      i_q_cluster_ids, i_cand_cluster_ids, i_q_l_kpts, i_q_r_kpts, i_cand_l_kpts, i_cand_r_kpts, i_q_l_cam_pts, i_cand_world_pts);

                // Loop closure inliers debugging.
                if (loop_closure_inliers_num_pub_.getNumSubscribers() > 0)
                    publishCounter(i_q_l_kpts.size(), loop_closure_inliers_num_pub_);

                // Loop found!
                if (i_q_l_kpts.size() >= static_cast<size_t>(min_lc_inliers_))
                {
                    // From cv::Mat to tf::Transform.
                    fromMatToTf(estimated_rotation, estimated_translation, estimated_motion);

                    // From {q_cam}^T_{O} to {O}^T_{q_cam}.
                    estimated_motion = estimated_motion.inverse();

                    return true;
                }
            }
        }
        return false;  
    }


    bool LoopCloser::loopCorrection(const int q_keyframe_id,
                                    const int cand_keyframe_id,
                                    const std::string& search_method,
                                    const tf::Transform& estimated_motion,
                                    const std::vector<int>& i_q_cluster_ids,
                                    const std::vector<int>& i_cand_cluster_ids,
                                    const std::vector<cv::KeyPoint>& i_q_l_kpts,
                                    const std::vector<cv::KeyPoint>& i_q_r_kpts,
                                    const std::vector<cv::KeyPoint>& i_cand_l_kpts,
                                    const std::vector<cv::KeyPoint>& i_cand_r_kpts)
    {
        // Gets the inliers per cluster pair.
        std::vector<int> inliers_per_pair, cand_keyframe_ids;
        std::vector<std::pair<int, int>> cluster_pairs, cand_cluster_keyframe_relations;
        getInliersPerClusterPair(i_q_cluster_ids, i_cand_cluster_ids, i_q_l_kpts, inliers_per_pair,
                                 cand_keyframe_ids, cluster_pairs, cand_cluster_keyframe_relations);

        // Pre-allocates memory.
        size_t inliers_per_pair_size = inliers_per_pair.size();
        std::vector<int> definitive_inliers_per_pair;
        std::vector<std::pair<int, int>> definitive_cluster_pairs; // q_cluster_id - cand_cluster_id
        definitive_inliers_per_pair.reserve(inliers_per_pair_size);
        definitive_cluster_pairs.reserve(inliers_per_pair_size);

        // Adds the corresponding edges.
        bool lc_already_exists, some_edge_added = false;
        tf::Transform relative_movement, cand_cluster_map_pose, q_cluster_camera_pose;
        for (size_t i = 0; i < inliers_per_pair_size; ++i)
        {
            if (inliers_per_pair[i] >= static_cast<int>(max_edge_inliers_ * 0.1))
            {
                // Checks if this loop closure exists.
                lc_already_exists = false;
                for (size_t j = 0; j < cluster_lc_found_.size(); ++j)
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
                cand_cluster_map_pose = map_->getVertexPose(cluster_pairs[i].second, true); // {O}^T_{cand_cluster}
                q_cluster_camera_pose = map_->getVertexPoseRelativeToCamera(cluster_pairs[i].first); // {q_cam}^T_{q_cluster}
                relative_movement = cand_cluster_map_pose.inverse() * estimated_motion * q_cluster_camera_pose; // {cand_cluster}^T_{q_cluster} = {cand_cluster}^T_{O} * {O}^T_{q_cam} * {q_cam}^T_{q_cluster}

                // Adds this edge to the graph.
                some_edge_added = true;
                map_->addEdge(cluster_pairs[i].second, cluster_pairs[i].first, normalisation(max_edge_inliers_, inliers_per_pair[i]), relative_movement);
                definitive_inliers_per_pair.push_back(inliers_per_pair[i]);
                definitive_cluster_pairs.emplace_back(cluster_pairs[i].first, cluster_pairs[i].second); // q_cluster_id - cand_cluster_id

                // Add this edge to the list of clusters that close loops.
                cluster_lc_found_.emplace_back(cluster_pairs[i].first, cluster_pairs[i].second);
            }
        }

        if (some_edge_added)
        {
            map_->optimiseGraphPoses();
            loop_closure_num_++;

            ROS_INFO_STREAM("[LC->loopCorrection:] ------------------------------");
            ROS_INFO_STREAM("                      LOOP CLOSURE: " << loop_closure_num_);
            ROS_INFO_STREAM("Between keyframe " << q_keyframe_id << " and " << cand_keyframe_id);
            ROS_INFO_STREAM("Method: " << search_method);
            ROS_INFO_STREAM("Inliers: " << i_q_l_kpts.size());
            ROS_INFO_STREAM("[LC->loopCorrection:] ------------------------------");

            if (enable_debugging_)
            {
                if (loop_closure_img_pub_.getNumSubscribers() > 0)
                    publishImgLoopClosure(ros::Time::now(), 
                                        loop_closure_img_pub_,
                                        q_keyframe_id,
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
                                        cand_cluster_keyframe_relations);

                writeImgLoopClosure(lc_path_ + convertTo5Digits(loop_closure_num_),
                                    q_keyframe_id,
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
                                    cand_cluster_keyframe_relations);
            }
            return true;
        }
        return false;
    }


    void LoopCloser::getInliersPerClusterPair(const std::vector<int>& i_q_cluster_ids,
                                              const std::vector<int>& i_cand_cluster_ids,
                                              const std::vector<cv::KeyPoint>& i_q_l_kpts,
                                              std::vector<int>& inliers_per_pair,
                                              std::vector<int>& cand_keyframe_ids,
                                              std::vector<std::pair<int, int>>& cluster_pairs,
                                              std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations)
    {
        std::pair<int, int> relation;
        bool cluster_pair_found, cand_keyframe_found;
        int q_cluster_id, cand_cluster_id, cand_keyframe_id, last_keyframe_id = -1, last_cluster_id = -1;
        for (size_t i = 0; i < i_q_l_kpts.size(); ++i)
        {
            q_cluster_id = i_q_cluster_ids[i];
            cand_cluster_id = i_cand_cluster_ids[i];
            cand_keyframe_id = map_->getVertexKeyframeId(cand_cluster_id);

            // Builds the unique candidate keyframe vector.
            cand_keyframe_found = false;
            for (size_t j = 0; j < cand_keyframe_ids.size(); ++j)
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
                cand_cluster_keyframe_relations.emplace_back(cand_cluster_id, cand_keyframe_id);
            }

            // Searches if this pair already exists.
            cluster_pair_found = false;
            for (size_t j = 0; j < cluster_pairs.size(); ++j)
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
                cluster_pairs.emplace_back(q_cluster_id, cand_cluster_id);
                inliers_per_pair.push_back(1);
            }
        }
    }


} // End namespace