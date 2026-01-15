/**
 *  @file mapmaker.cpp
 *  @brief The Mapmaker constructs a topological map where nodes are clusters of keypoints 
 *         and edges are the relative movement between them and the number of keypoint matchings.
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

 
#include "mapmaker.hpp"


namespace slamon
{

    
    Mapmaker::Mapmaker(Map* map,
                       const int min_lc_inliers,
                       const std::string& w_path,
                       const bool enable_debugging,
                       Queue<Cluster>* cluster_queue,
                       const std::string& robot_frame_id):
                       initialised_map_{false}, enable_debugging_{enable_debugging}, 
                       robot_frame_id_{robot_frame_id}, map_{map}, cluster_queue_{cluster_queue},
                       nh_{}, nhp_{"~"}
    {
        // Computes maximum inliers for edges.
        max_edge_inliers_ = min_lc_inliers * 2;

        // Create debugging directories.
        if (enable_debugging_)
        {
            keyframe_path_ = w_path + "keyframes/";
            if (std::filesystem::exists(keyframe_path_))
                std::filesystem::remove_all(keyframe_path_);
            std::filesystem::create_directory(keyframe_path_);

            clusters_path_ = w_path + "clusters/";
            if (std::filesystem::exists(clusters_path_))
                std::filesystem::remove_all(clusters_path_);
            std::filesystem::create_directory(clusters_path_);

            poses_file_ = w_path + "keyframes_poses.txt";
            if (std::filesystem::exists(poses_file_))
                std::filesystem::remove(poses_file_.c_str());

            edges_file_ = w_path + "loop_closure_edges.txt";
            if (std::filesystem::exists(edges_file_))
                std::filesystem::remove(edges_file_.c_str());

            std::string lock_file = w_path + "keyframes_poses.txt.lock";
            if (std::filesystem::exists(lock_file))
                std::filesystem::remove(lock_file.c_str());
            lock_file = w_path + "loop_closure_edges.txt.lock";
            if (std::filesystem::exists(lock_file))
                std::filesystem::remove(lock_file.c_str());

            writeMapResults();
        }

        // Client services.
        robot2camera_client_ = nh_.serviceClient<slamon::GetTransform>("get_robot_to_camera_tf");

        // Publishers.
        robot_map_pub_ = nhp_.advertise<nav_msgs::Odometry>("robot_map", 1);
        camera_map_pub_ = nhp_.advertise<nav_msgs::Odometry>("camera_map", 1);
        run_times_pub_ = nhp_.advertise<slamon::Performance>("mapmaker_performance", 1);
        keyframes_num_pub_ = nhp_.advertise<std_msgs::Int32>("keyframe_num", 1);
        clustering_img_pub_ = nhp_.advertise<sensor_msgs::Image>("keypoints_clustering_img", 1);

        // Subscribers.
        odom_sub_.subscribe(nh_, "odom", 5);
        keyframe_sub_.subscribe(nh_, "keyframe", 5);

        // Message sync.
        sync_.reset(new Sync_(SyncPolicy_(10), odom_sub_, keyframe_sub_));
        sync_->registerCallback(boost::bind(&Mapmaker::callback, this, _1, _2));

        // ROS timer.
        timer_ = nh_.createWallTimer(ros::WallDuration(15.0), &Mapmaker::timerCallback, this);
    }


    void Mapmaker::run()
    {
        ROS_INFO("Mapmaker started.");
        while (ros::ok())
        {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ROS_INFO("Mapmaker stopped.");
    }


    void Mapmaker::callback(const nav_msgs::OdometryConstPtr& odom_msg,
                            const slamon::KeyframeConstPtr& keyframe_msg)
    {
        // Stop timer.
        timer_.stop();

        // Time at which the program enters into the callback.
        double start_callback_time = ros::Time::now().toSec();
        
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Reset run times.
        run_times_.reset();

        // From odom ROS msg to tf2::Transform.
        tf2::Transform c_robot_odom;
        tf2::fromMsg(odom_msg->pose.pose, c_robot_odom);

        // Gets current keyframe id.
        int keyframe_id = keyframe_msg->id;

        // Gets current keyframe stamp.
        ros::Time stamp = keyframe_msg->header.stamp;

        if (enable_debugging_)
            ROS_INFO_STREAM("[MM->Includer:] Adding keyframe: " << keyframe_id);

        // Updates pose.
        start_module_time = ros::Time::now().toSec();
        tf2::Transform c_camera_map_pose;
        if (initialised_map_)
        {
            // {wo}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
            tf2::Transform c_camera_odom = c_robot_odom * robot2camera_;

            // {wo}^T_{p_cam} = {wo}^T_{p_robot} * {robot}^T_{cam}.
            tf2::Transform p_camera_odom = p_robot_odom_ * robot2camera_;

            // {p_cam}^T_{c_cam} = {p_cam}^T_{wo} * {wo}^T_{c_cam}.
            tf2::Transform camera_delta_odom = p_camera_odom.inverse() * c_camera_odom;

            // Gets {wm}^T_{p_cam}.
            tf2::Transform p_camera_map_pose = map_->getKeyframePose(keyframe_id - 1);

            // {wm}^T_{c_cam} = {wm}^T_{p_cam} * {p_cam}^T_{c_cam}.
            c_camera_map_pose = p_camera_map_pose * camera_delta_odom;
        }
        else
        {
            // Gets robot to camera transformation.
            getRobot2Camera();

            // The first SLAM position is the same as the odometry.
            // {wm}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
            c_camera_map_pose = c_robot_odom * robot2camera_;

            initialised_map_ = true;
        }
        p_robot_odom_ = c_robot_odom;
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Pose update run time.

        // Saves the keyframe timestamp.
        map_->addTimestamp(stamp.toSec());

        // Loop over frame clusters.
        start_module_time = ros::Time::now().toSec();
        int cluster_id;
        std::vector<int> c_clusters_ids;
        tf2::Transform local_cluster_pose, cluster_map_pose;
        std::vector<std::vector<cv::KeyPoint>> clustered_l_kpts;
        const size_t clusters_num = keyframe_msg->batches.size();
        c_clusters_ids.reserve(clusters_num);
        clustered_l_kpts.reserve(clusters_num);
        for (size_t i = 0; i < clusters_num; ++i)
        {
            // Gets cluster id.
            cluster_id = keyframe_msg->batches[i].global_id;

            // Gets {c_cam}^T_{c_cluster}.
            local_cluster_pose.setIdentity();
            local_cluster_pose.setOrigin(tf2::Vector3(keyframe_msg->batches[i].centroid.x,
                                                      keyframe_msg->batches[i].centroid.y,
                                                      keyframe_msg->batches[i].centroid.z));

            // {wm}^T_{c_cluster} = {wm}^T_{c_cam} * {c_cam}^T_{c_cluster}.
            cluster_map_pose = c_camera_map_pose * local_cluster_pose;

            // Adds cluster to the graph.
            map_->addVertex(cluster_id, cluster_map_pose);

            // Stores information.
            c_clusters_ids.push_back(cluster_id);
            map_->addLocalClusterPose(local_cluster_pose);
            map_->addClusterKeyframeRelation(std::make_pair(cluster_id, keyframe_id));

            // Creates cluster.
            Cluster cluster = fromBatchToCluster(keyframe_id,
                                                 i,
                                                 keyframe_msg->batches[i]);

            // Sends the new clusters to the loop closing thread.
            cluster_queue_->emplaceBack(cluster);

            // Accumulate clustered left keypoints.
            clustered_l_kpts.emplace_back(cluster.getLeftKpts());
        }
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Vertex insertion run time.

        // Adds edges between clusters of the same frame.
        start_module_time = ros::Time::now().toSec();
        if (clusters_num > 1)
        {
            // Retrieve all possible combinations.
            std::vector<std::vector<int>> combinations = createCombinations(c_clusters_ids.size(), 2);

            int cluster_0_id, cluster_1_id;
            tf2::Transform relative_movement, cluster_0_map_pose, cluster_1_map_pose;
            for (size_t i = 0; i < combinations.size(); ++i)
            {
                cluster_0_id = c_clusters_ids[combinations[i][0]];
                cluster_1_id = c_clusters_ids[combinations[i][1]];

                cluster_0_map_pose = map_->getVertexPose(cluster_0_id, true);
                cluster_1_map_pose = map_->getVertexPose(cluster_1_id, true);

                relative_movement = cluster_0_map_pose.inverse() * cluster_1_map_pose;
                map_->addEdge(cluster_0_id, cluster_1_id, 1.0, relative_movement);
            }
        }
        run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Co-framed edges insertion run time.

        // Connects this keyframe with the previous.
        start_module_time = ros::Time::now().toSec();
        if (keyframe_id > 0)
        {
            if (keyframe_msg->cluster_pairs.empty()) // Connects only the closest clusters between the two keyframes.
            {
                // Retrieves the clusters ids of the previous keyframe.
                std::vector<int> p_clusters_ids;
                map_->getKeyframeVertices(keyframe_id - 1, p_clusters_ids);
                
                // Searches for the nearest clusters.
                std::pair<int, int> closest_clusters;
                tf2::Transform c_cluster_map_pose, p_cluster_map_pose;
                std::pair<tf2::Transform, tf2::Transform> closest_poses;
                double dist, min_dist = std::numeric_limits<double>::max();
                for (size_t i = 0; i < c_clusters_ids.size(); ++i)
                {
                    // Current ith vertex pose. 
                    c_cluster_map_pose = map_->getVertexPose(c_clusters_ids[i], true);

                    for (size_t j = 0; j < p_clusters_ids.size(); ++j)
                    {
                        // Previous ith vertex pose. 
                        p_cluster_map_pose = map_->getVertexPose(p_clusters_ids[j], true);

                        dist = poseDiff3D(c_cluster_map_pose, p_cluster_map_pose);
                        if (dist < min_dist)
                        {
                            min_dist = dist;

                            closest_clusters.first = c_clusters_ids[i];
                            closest_clusters.second = p_clusters_ids[j];

                            closest_poses.first = c_cluster_map_pose;
                            closest_poses.second = p_cluster_map_pose;
                        }
                    }
                }

                // Adds edge.
                if (!(closest_clusters.first == 0 && closest_clusters.second == 0))
                    map_->addEdge(closest_clusters.first, closest_clusters.second, normalisation(max_edge_inliers_, 0.0),
                                  closest_poses.first.inverse() * closest_poses.second);
                else
                    ROS_ERROR_STREAM("[MM->Vertex linker:] Unable to connect current and previous keyframe (when there are no inliers). Graph will have non-connected vertices!");
            }
            else // Connects the keyframe clusters with the number of inliers between clusters as confidence.
            {
                std::vector<int> inliers_per_pair = keyframe_msg->inliers_per_pair;
                std::vector<int> cluster_pairs = keyframe_msg->cluster_pairs;
                if (cluster_pairs.size() == inliers_per_pair.size() * 2)
                {
                    int c_cluster_id, p_cluster_id;
                    tf2::Transform c_cluster_map_pose, p_cluster_map_pose;
                    for (size_t i = 0; i < inliers_per_pair.size(); ++i)
                    {
                        // Gets clusters ids.
                        c_cluster_id = cluster_pairs[i * 2];
                        p_cluster_id = cluster_pairs[i * 2 + 1];

                        // Gets clusters poses.
                        c_cluster_map_pose = map_->getVertexPose(c_cluster_id, true);
                        p_cluster_map_pose = map_->getVertexPose(p_cluster_id, true);

                        // Adds edge.
                        map_->addEdge(c_cluster_id, p_cluster_id, normalisation(max_edge_inliers_, inliers_per_pair[i]),
                                      c_cluster_map_pose.inverse() * p_cluster_map_pose);
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("[MM->Vertex linker:] Cluster_pairs and inliers_per_pair have a different size. Unable to connect current and previous keyframe. Graph will have non-connected vertices!");
                }
            }
        }
        run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Inter-frames edges insertion run time.

        // Publishes to ROS.
        start_module_time = ros::Time::now().toSec();

        // Publishes updated pose.
        tf2::Transform updated_camera_map_pose = map_->getVertexCameraPose(map_->getCurrentVertexId(), true);
        publishPose(stamp, updated_camera_map_pose, false, camera_map_pub_, camera_frame_id_);
        tf2::Transform updated_robot_map_pose = updated_camera_map_pose * robot2camera_.inverse();
        publishPose(stamp, updated_robot_map_pose, false, robot_map_pub_, robot_frame_id_);

        // Publishes clustered keyframe.
        cv::Mat l_img, r_img;
        sensor_msgs::ImageConstPtr img_msg;
        img_msg = boost::make_shared<const sensor_msgs::Image>(keyframe_msg->l_img);
        bool l_img_ok = fromImgMsgToMat(img_msg, l_img);
        img_msg = boost::make_shared<const sensor_msgs::Image>(keyframe_msg->r_img);
        bool r_img_ok = fromImgMsgToMat(img_msg, r_img);
        if (clustering_img_pub_.getNumSubscribers() > 0)
        {
            if (l_img_ok && r_img_ok)
                publishImgKeypointsClustering(l_img, stamp, clustering_img_pub_, clustered_l_kpts);
        }

        // Publishes number of keyframes.
        if (keyframes_num_pub_.getNumSubscribers() > 0)
            publishCounter(keyframe_id + 1, keyframes_num_pub_);
        run_times_.run_time_4 = ros::Time::now().toSec() - start_module_time; // Publishing run time.

        // Writes data to disk.
        start_module_time = ros::Time::now().toSec();
        if (enable_debugging_)
        {
            // Saves keyframe map poses and loop closure edges to files.
            writeMapResults();

            if (l_img_ok && r_img_ok)
            {
                std::string keyframe_name = convertTo5Digits(keyframe_id);

                // Saves keyframe images to disk.
                cv::imwrite(keyframe_path_ + keyframe_name + "_left.png", l_img);
                cv::imwrite(keyframe_path_ + keyframe_name + "_right.png", r_img);

                // Saves clustered image.
                writeImgKeypointsClustering(l_img, clusters_path_ + keyframe_name, clustered_l_kpts);
            }
        }
        run_times_.run_time_5 = ros::Time::now().toSec() - start_module_time; // Data write run time.

        run_times_.total_run_time = ros::Time::now().toSec() - start_callback_time;
        if (run_times_pub_.getNumSubscribers() > 0)
            publishPerformance(ros::Time::now(), 1, run_times_, run_times_pub_);

        // Start timer.
        timer_.start();
    }


    void Mapmaker::timerCallback(const ros::WallTimerEvent& event)
    {
        if (enable_debugging_)
        {
            ROS_WARN_STREAM("[MM->timerCallback:] Writing files!");

            // Saves keyframe map poses and loop closure edges to files.
            writeMapResults();
        }
    }


    void Mapmaker::getRobot2Camera()
    {
        slamon::GetTransform srv;
        if (robot2camera_client_.call(srv))
        {
            camera_frame_id_ = srv.response.tf.child_frame_id;
            tf2::fromMsg(srv.response.tf.transform, robot2camera_);

            ROS_WARN_STREAM("[MM->Static transform:] GetTransform service call successful. The robot2camera transform is as follows:" << std::endl <<
                            "tx: " << robot2camera_.getOrigin().getX() << std::endl << 
                            "ty: " << robot2camera_.getOrigin().getY() << std::endl << 
                            "tz: " << robot2camera_.getOrigin().getZ() << std::endl << 
                            "qx: " << robot2camera_.getRotation().getX() << std::endl << 
                            "qy: " << robot2camera_.getRotation().getY() << std::endl << 
                            "qz: " << robot2camera_.getRotation().getZ() << std::endl << 
                            "qw: " << robot2camera_.getRotation().getW());
        }
        else
        {
            ROS_WARN_STREAM("[MM->Static transform:] Failed to call GetTransform service. The robot2camera transform will be assumed as the identity.");
            robot2camera_.setIdentity();
        }
    }


    Cluster Mapmaker::fromBatchToCluster(const int keyframe_id,
                                         const int cluster_local_id,
                                         const slamon::Batch& batch_msg)
    {
        // Pre-allocates.
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> l_cam_pts;
        std::vector<cv::KeyPoint> l_kpts, r_kpts;
        size_t batch_size = batch_msg.l_cam_pts.size();
        l_kpts.reserve(batch_size);
        r_kpts.reserve(batch_size);
        l_cam_pts.reserve(batch_size);

        // Collect all batch data.
        cv::KeyPoint kpt;
        cv::Point3f cam_pt;
        std::vector<float> desc_aux;
        for (size_t i = 0; i < batch_size; ++i)
        {
            kpt.pt.x = batch_msg.l_kpts[i].pt.x;
            kpt.pt.y = batch_msg.l_kpts[i].pt.y;
            l_kpts.emplace_back(kpt);
            desc_aux = batch_msg.l_kpts[i].desc;
            l_descs.push_back(cv::Mat(1, desc_aux.size(), CV_32F, desc_aux.data()));
            kpt.pt.x = batch_msg.r_kpts[i].pt.x;
            kpt.pt.y = batch_msg.r_kpts[i].pt.y;
            r_kpts.emplace_back(kpt);
            desc_aux = batch_msg.r_kpts[i].desc;
            r_descs.push_back(cv::Mat(1, desc_aux.size(), CV_32F, desc_aux.data()));
            cam_pt.x = batch_msg.l_cam_pts[i].x;
            cam_pt.y = batch_msg.l_cam_pts[i].y;
            cam_pt.z = batch_msg.l_cam_pts[i].z;
            l_cam_pts.emplace_back(cam_pt);
        }

        Cluster cluster(batch_msg.global_id, cluster_local_id, keyframe_id,
                        l_descs, r_descs, l_kpts, r_kpts, l_cam_pts);
        return cluster;
    }


    std::vector<std::vector<int>> Mapmaker::createCombinations(const size_t n,
                                                               const size_t r)
    {
        std::string bitmask(r, 1);
        bitmask.resize(n, 0);

        std::vector<int> combination;
        std::vector<std::vector<int>> combinations;
        do
        {
            for (size_t i = 0; i < n; ++i)
            {
                if (bitmask[i])
                    combination.push_back(i);
            }
            combinations.push_back(combination);
            combination.clear();
        }
        while (std::prev_permutation(bitmask.begin(), bitmask.end()));

        return combinations;
    }


    void Mapmaker::writeMapResults()
    {
        // Saves keyframe map poses to file.
        std::vector<double> stamps;
        std::vector<tf2::Transform> keyframe_poses;
        map_->getKeyframePoses(stamps, keyframe_poses);
        for (size_t i = 0; i < keyframe_poses.size(); ++i)
            keyframe_poses[i] = keyframe_poses[i] * robot2camera_.inverse();
        writePoses(poses_file_, stamps, keyframe_poses);

        // Saves loop closure edges to file.
        std::vector<double> edges_weights;
        std::vector<std::pair<int, int>> edges_keyframe_ids;
        std::vector<std::pair<tf2::Transform, tf2::Transform>> edges_keyframe_poses;
        map_->getLoopClosureEdges(edges_weights, edges_keyframe_ids, edges_keyframe_poses);
        for (size_t i = 0; i < edges_keyframe_poses.size(); ++i)
        {
            edges_keyframe_poses[i].first = edges_keyframe_poses[i].first * robot2camera_.inverse();
            edges_keyframe_poses[i].second = edges_keyframe_poses[i].second * robot2camera_.inverse();
        }
        writeLoopClosureEdges(edges_file_, edges_weights, edges_keyframe_ids, edges_keyframe_poses);
    }


}