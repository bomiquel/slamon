#include "mapmaker.hpp"


namespace slamon
{

    Mapmaker::Mapmaker(Map* map,
                       int min_lc_inliers,
                       bool enable_debugging,
                       const std::string& w_path,
                       Queue<Cluster>* cluster_queue,
                       const std::string& robot_frame_id):
                       initialised_map_{false}, enable_debugging_{enable_debugging}, 
                       robot_frame_id_{robot_frame_id}, map_{map}, cluster_queue_{cluster_queue}
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
        }
    }


    void Mapmaker::run()
    {
        // Create ROS node.
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");

        // Client services.
        robot2camera_client_ = nh.serviceClient<slamon::GetTransform>("get_robot_to_camera_tf");

        // Publishers.
        robot_map_pub_ = nhp.advertise<nav_msgs::Odometry>("robot_map", 1);
        camera_map_pub_ = nhp.advertise<nav_msgs::Odometry>("camera_map", 1);
        run_times_pub_ = nhp.advertise<slamon::Performance>("mapmaker_performance", 1);
        keyframes_num_pub_ = nhp.advertise<std_msgs::Int32>("keyframe_num", 1);
        clustering_img_pub_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering_img", 1);

        // Subscribers.
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<slamon::Keyframe> keyframe_sub;

        // Message sync.
        boost::shared_ptr<Sync_> sync;
        odom_sub.subscribe(nh, "odom", 5);
        keyframe_sub.subscribe(nh, "keyframe", 5);
        sync.reset(new Sync_(SyncPolicy_(10), odom_sub, keyframe_sub));
        sync->registerCallback(boost::bind(&Mapmaker::callback, this, _1, _2));

        // Calls callback when synchronised messages arrive
        ros::spin();
    }


    void Mapmaker::callback(const nav_msgs::OdometryConstPtr& odom_msg,
                            const slamon::KeyframeConstPtr& keyframe_msg)
    {
        // Time at which the program enters into the callback.
        double start_callback_time = ros::Time::now().toSec();
        
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Reset run times.
        run_times_.reset();

        // From odom ROS msg to tf::Transform.
        tf::Transform c_robot_odom;
        tf::poseMsgToTF(odom_msg->pose.pose, c_robot_odom);

        // Gets current keyframe id.
        int keyframe_id = keyframe_msg->id;

        // Gets current keyframe stamp.
        ros::Time stamp = keyframe_msg->header.stamp;

        if (enable_debugging_)
            ROS_INFO_STREAM("[MM->Includer]: Adding keyframe: " << keyframe_id);

        // Updates pose.
        start_module_time = ros::Time::now().toSec();
        tf::Transform c_camera_map_pose;
        if (initialised_map_)
        {
            // {wo}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
            tf::Transform c_camera_odom = c_robot_odom * robot2camera_;

            ROS_INFO_STREAM("{wo}^T_{c_cam}: " << c_camera_odom.getOrigin().getX() << ", "
                                               << c_camera_odom.getOrigin().getY() << ", "
                                               << c_camera_odom.getOrigin().getZ() << ", "
                                               << c_camera_odom.getRotation().getX() << ", "
                                               << c_camera_odom.getRotation().getY() << ", "
                                               << c_camera_odom.getRotation().getZ() << ", "
                                               << c_camera_odom.getRotation().getW());

            // {wo}^T_{p_cam} = {wo}^T_{p_robot} * {robot}^T_{cam}.
            tf::Transform p_camera_odom = p_robot_odom_ * robot2camera_;

            ROS_INFO_STREAM("{wo}^T_{p_cam}: " << p_camera_odom.getOrigin().getX() << ", "
                                               << p_camera_odom.getOrigin().getY() << ", "
                                               << p_camera_odom.getOrigin().getZ() << ", "
                                               << p_camera_odom.getRotation().getX() << ", "
                                               << p_camera_odom.getRotation().getY() << ", "
                                               << p_camera_odom.getRotation().getZ() << ", "
                                               << p_camera_odom.getRotation().getW());

            // {p_cam}^T_{c_cam} = {p_cam}^T_{wo} * {wo}^T_{c_cam}.
            tf::Transform camera_delta_odom = p_camera_odom.inverse() * c_camera_odom;

            ROS_INFO_STREAM("{p_cam}^T_{c_cam}: " << camera_delta_odom.getOrigin().getX() << ", "
                                                  << camera_delta_odom.getOrigin().getY() << ", "
                                                  << camera_delta_odom.getOrigin().getZ() << ", "
                                                  << camera_delta_odom.getRotation().getX() << ", "
                                                  << camera_delta_odom.getRotation().getY() << ", "
                                                  << camera_delta_odom.getRotation().getZ() << ", "
                                                  << camera_delta_odom.getRotation().getW());

            // Gets {wm}^T_{p_cam}.
            tf::Transform p_camera_map_pose = map_->getKeyframePose(keyframe_id - 1);

            ROS_INFO_STREAM("{wm}^T_{p_cam}: " << p_camera_map_pose.getOrigin().getX() << ", "
                                               << p_camera_map_pose.getOrigin().getY() << ", "
                                               << p_camera_map_pose.getOrigin().getZ() << ", "
                                               << p_camera_map_pose.getRotation().getX() << ", "
                                               << p_camera_map_pose.getRotation().getY() << ", "
                                               << p_camera_map_pose.getRotation().getZ() << ", "
                                               << p_camera_map_pose.getRotation().getW());

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
        ROS_INFO_STREAM("{wm}^T_{c_cam}: " << c_camera_map_pose.getOrigin().getX() << ", "
                                           << c_camera_map_pose.getOrigin().getY() << ", "
                                           << c_camera_map_pose.getOrigin().getZ() << ", "
                                           << c_camera_map_pose.getRotation().getX() << ", "
                                           << c_camera_map_pose.getRotation().getY() << ", "
                                           << c_camera_map_pose.getRotation().getZ() << ", "
                                           << c_camera_map_pose.getRotation().getW());
        p_robot_odom_ = c_robot_odom;
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Pose update run time.

        ROS_INFO_STREAM("[MM->Includer]: Hello there Loop over frame clusters.");

        // Saves the keyframe timestamp.
        map_->addTimestamp(stamp.toSec());

        // Loop over frame clusters.
        start_module_time = ros::Time::now().toSec();
        int cluster_id;
        std::vector<int> c_clusters_ids;
        tf::Transform local_cluster_pose, cluster_map_pose;
        std::vector<std::vector<cv::KeyPoint>> clustered_l_kpts;
        const size_t clusters_num = keyframe_msg->batches.size();
        c_clusters_ids.reserve(clusters_num);
        clustered_l_kpts.reserve(clusters_num);
        for (size_t i = 0; i < clusters_num; ++i)
        {
            // Gets {c_cam}^T_{c_cluster}.
            local_cluster_pose.setIdentity();
            local_cluster_pose.setOrigin(tf::Vector3(keyframe_msg->batches[i].centroid.x,
                                                     keyframe_msg->batches[i].centroid.y,
                                                     keyframe_msg->batches[i].centroid.z));

            // {wm}^T_{c_cluster} = {wm}^T_{c_cam} * {c_cam}^T_{c_cluster}.
            cluster_map_pose = c_camera_map_pose * local_cluster_pose;

            // Adds cluster to the graph.
            cluster_id = map_->addVertex(cluster_map_pose);

            // Stores information.
            c_clusters_ids.push_back(cluster_id);
            map_->addLocalClusterPose(local_cluster_pose);
            map_->addClusterKeyframeRelation(std::make_pair(cluster_id, keyframe_id));

            // Creates cluster.
            Cluster cluster = fromBatchToCluster(keyframe_id,
                                                 i,
                                                 cluster_id,
                                                 keyframe_msg->batches[i]);

            // Sends the new clusters to the loop closing thread.
            cluster_queue_->emplaceBack(cluster);

            // Accumulate clustered left keypoints.
            clustered_l_kpts.emplace_back(cluster.getLeftKpts());
        }
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Vertex insertion run time.

        ROS_INFO_STREAM("[MM->Includer]: Hello there Adds edges between clusters of the same frame.");

        // Adds edges between clusters of the same frame.
        start_module_time = ros::Time::now().toSec();
        if (clusters_num > 1)
        {
            // Retrieve all possible combinations.
            std::vector<std::vector<int>> combinations = createCombinations(c_clusters_ids.size(), 2);

            int cluster_0_id, cluster_1_id;
            tf::Transform relative_movement, cluster_0_map_pose, cluster_1_map_pose;
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

        ROS_INFO_STREAM("[MM->Includer]: Hello there Connects this keyframe with the previous.");

        // Connects this keyframe with the previous.
        start_module_time = ros::Time::now().toSec();
        if (keyframe_id > 0)
        {
            // Retrieves the clusters ids of the previous keyframe.
            std::vector<int> p_clusters_ids;
            map_->getKeyframeVertices(keyframe_id - 1, p_clusters_ids);

            if (keyframe_msg->cluster_pairs.empty())
            {
                // Connects only the closest clusters between the two keyframes.
                std::pair<int, int> closest_clusters;
                tf::Transform c_cluster_map_pose, p_cluster_map_pose;
                std::pair<tf::Transform, tf::Transform> closest_poses;
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
            else
            {
                // Connect the keyframe clusters with the number of inliers between clusters as confidence.
                std::vector<int> inliers_per_pair = keyframe_msg->inliers_per_pair;
                std::vector<int> cluster_pairs = keyframe_msg->cluster_pairs;
                if (cluster_pairs.size() == inliers_per_pair.size() * 2)
                {
                    // Sorts the clusters of the previous keyframe in ascending order.
                    std::sort(p_clusters_ids.begin(), p_clusters_ids.end());

                    int c_cluster_id, p_cluster_id;
                    tf::Transform c_cluster_map_pose, p_cluster_map_pose;
                    for (size_t i = 0; i < inliers_per_pair.size(); ++i)
                    {
                        // Gets clusters ids.
                        c_cluster_id = c_clusters_ids[cluster_pairs[i * 2]];
                        p_cluster_id = p_clusters_ids[cluster_pairs[i * 2 + 1]];

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

        ROS_INFO_STREAM("[MM->Includer]: Hello there Publishes to ROS.");

        // Publishes to ROS.
        start_module_time = ros::Time::now().toSec();

        // Publishes updated pose.
        tf::Transform updated_camera_map_pose = map_->getVertexCameraPose(map_->getCurrentVertexId(), true);
        publishPose(stamp, updated_camera_map_pose, false, camera_map_pub_, camera_frame_id_);
        tf::Transform updated_robot_map_pose = updated_camera_map_pose * robot2camera_.inverse();
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

        ROS_INFO_STREAM("[MM->Includer]: Hello there Writes data to disk.");

        // Writes data to disk.
        start_module_time = ros::Time::now().toSec();
        if (enable_debugging_)
        {
            // Saves keyframe map poses to file.
            std::vector<double> stamps;
            std::vector<tf::Transform> keyframe_poses;
            map_->getKeyframePoses(stamps, keyframe_poses);
            for (size_t i = 0; i < keyframe_poses.size(); ++i)
                keyframe_poses[i] = keyframe_poses[i] * robot2camera_.inverse();
            writePoses(poses_file_, stamps, keyframe_poses);

            // Saves loop closure edges to file.
            std::vector<std::pair<int, int>> edges_keyframe_ids;
            std::vector<std::pair<tf::Transform, tf::Transform>> edges_keyframe_poses;
            map_->getLoopClosureEdges(edges_keyframe_ids, edges_keyframe_poses);
            for (size_t i = 0; i < edges_keyframe_poses.size(); ++i)
            {
                edges_keyframe_poses[i].first = edges_keyframe_poses[i].first * robot2camera_.inverse();
                edges_keyframe_poses[i].second = edges_keyframe_poses[i].second * robot2camera_.inverse();
            }
            writeLoopClosureEdges(edges_file_, edges_keyframe_ids, edges_keyframe_poses);

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
    }

    // void Mapmaker::callback(const nav_msgs::OdometryConstPtr& odom_msg,
    //                         const slamon::StereoImageAttributesConstPtr& stereo_attributes_msg)
    // {
    //     // Time at which the program enters into the callback.
    //     double start_callback_time = ros::Time::now().toSec();
        
    //     // Auxilary variable to set the start time of a module.
    //     double start_module_time;

    //     // From ROS messages to local objects.
    //     start_module_time = ros::Time::now().toSec();
    //     Frame c_frame;
    //     tf::Transform c_odom_robot_pose;
    //     fromStereoAttributesToFrame(stereo_attributes_msg, c_frame);
    //     tf::poseMsgToTF(odom_msg->pose.pose, c_odom_robot_pose);
    //     run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // From StereoImageAttributes msg to Frame object run time.

    //     // If enable debugging gets images from disk.
    //     std::string c_frame_name;
    //     if (enable_debugging_)
    //     {
    //         c_frame_name = convertTo5Digits(stereo_attributes_msg->header.seq);
    //         c_frame.setLeftImg(cv::imread(frame_path_ + c_frame_name + "_left.png", cv::IMREAD_COLOR));
    //         c_frame.setRightImg(cv::imread(frame_path_ + c_frame_name + "_right.png", cv::IMREAD_COLOR));
    //     }

    //     // Does the map need a new keyframe?
    //     start_module_time = ros::Time::now().toSec();
    //     bool add_keyframe = needNewKeyFrame(c_odom_robot_pose, c_frame);
    //     run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Need new keyframe? run time.

    //     // Includes a keyframe if the map needs one.
    //     if (add_keyframe)
    //     {
    //         tf::Transform c_map_camera_pose;
    //         if (init_map_)
    //         {
    //             // {wo}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
    //             tf::Transform c_odom_camera_pose = c_odom_robot_pose * robot2camera_;

    //             // {wo}^T_{p_cam} = {wo}^T_{p_robot} * {robot}^T_{cam}.
    //             tf::Transform p_odom_camera_pose = p_odom_robot_pose_ * robot2camera_;

    //             // {p_cam}^T_{c_cam} = {p_cam}^T_{wo} * {wo}^T_{c_cam}.
    //             tf::Transform c_camera_motion = p_odom_camera_pose.inverse() * c_odom_camera_pose;

    //             // Gets {wm}^T_{p_cam}.
    //             tf::Transform p_map_camera_pose = map_->getKeyframePose(c_keyframe_id_);

    //             // {wm}^T_{c_cam} = {wm}^T_{p_cam} * {p_cam}^T_{c_cam}.
    //             c_map_camera_pose = p_map_camera_pose * c_camera_motion;
    //         }
    //         else
    //         {
    //             // The first SLAM position is the same as the odometry.
    //             // {wm}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
    //             c_map_camera_pose = c_odom_robot_pose * robot2camera_;
    //         }

    //         // Updates global variables.
    //         init_map_ = true;
    //         c_keyframe_id_++;
    //         p_odom_robot_pose_ = c_odom_robot_pose;
            
    //         // Get frame visual information.
    //         const ros::Time stamp = c_frame.getTimestamp();
    //         const cv::Mat l_descs = c_frame.getLeftDescs();
    //         const cv::Mat r_descs = c_frame.getRightDescs();
    //         const std::vector<int> cluster_ids = c_frame.getClusterIds();
    //         const std::vector<cv::KeyPoint> l_kpts = c_frame.getLeftKpts();
    //         const std::vector<cv::KeyPoint> r_kpts = c_frame.getRightKpts();
    //         const size_t clusters_num = c_frame.getClustersCentroids().size();
    //         const std::vector<cv::Point3f> l_cam_pts = c_frame.getLeftCameraPoints();
    //         const std::vector<cv::Point3f> cluster_centroids = c_frame.getClustersCentroids();
    //         const size_t total_keyframe_size = l_cam_pts.size();

    //         // Debug keyframe.
    //         if (enable_debugging_)
    //         {
    //             std::string c_keyframe_name = convertTo5Digits(c_keyframe_id_);
    //             cv::imwrite(keyframe_path_ + c_keyframe_name + "_left.png", c_frame.getLeftImg());
    //             cv::imwrite(keyframe_path_ + c_keyframe_name + "_right.png", c_frame.getRightImg());
    //             if (clustering_img_pub_.getNumSubscribers() > 0)
    //                 publishImgKeypointsClustering(c_frame.getLeftImg(), stamp, clusters_num, clustering_img_pub_, cluster_ids, l_kpts);
    //             writeImgKeypointsClustering(c_frame.getLeftImg(), clusters_num, clusters_path_ + c_keyframe_name, cluster_ids, l_kpts);
    //             ROS_INFO_STREAM("[MM->Includer]: Adding keyframe: " << c_keyframe_id_);
    //         }

    //         // Save the frame timestamp
    //         map_->addTimestamp(stamp.toSec());

    //         // Loop over frame clusters.
    //         size_t idx = 0;
    //         int cluster_id;
    //         std::vector<int> c_vertex_ids;
    //         tf::Transform local_cluster_pose, c_cluster_pose;
    //         for (size_t i = 0; i < clusters_num; ++i)
    //         {
    //             // Gets {c_cam}^T_{c_cluster}.
    //             local_cluster_pose = fromPoint3DToTf(cluster_centroids[i]);

    //             // {w}^T_{c_cluster} = {w}^T_{c_cam} * {c_cam}^T_{c_cluster}.
    //             c_cluster_pose = c_map_camera_pose * local_cluster_pose;

    //             // Add cluster to the graph.
    //             cluster_id = map_->addVertex(c_cluster_pose);

    //             // Store information.
    //             c_vertex_ids.push_back(cluster_id);
    //             map_->addLocalClusterPose(local_cluster_pose);
    //             map_->addClusterKeyframeRelation(std::make_pair(cluster_id, c_keyframe_id_));

    //             while (idx < total_keyframe_size)
    //             {
                    
    //             }
                
    //         }




    //         // Pre-allocates memory.
    //         start_module_time = ros::Time::now().toSec();
    //         size_t clusters_num = clusters.size();
    //         std::vector<int> c_vertex_ids;
    //         std::vector<Cluster> clusters_to_close_loop;
    //         c_vertex_ids.reserve(clusters_num);
    //         clusters_to_close_loop.reserve(clusters_num);
            
    //         // Loop of frame clusters.
    //         size_t cluster_size;
    //         int cluster_id, idx;
    //         cv::Mat cluster_l_descs, cluster_r_descs;
    //         std::vector<cv::Point3f> cluster_l_cam_pts;
    //         std::vector<cv::KeyPoint> cluster_l_kpts, cluster_r_kpts;
    //         tf::Transform local_cluster_pose, c_cluster_pose;
    //         for (size_t i = 0; i < clusters_num; ++i)
    //         {
    //             // Gets {c_cam}^T_{c_cluster}.
    //             local_cluster_pose = fromPoint3DToTf(cluster_centroids[i]);

    //             // {w}^T_{c_cluster} = {w}^T_{c_cam} * {c_cam}^T_{c_cluster}.
    //             c_cluster_pose = c_map_camera_pose * local_cluster_pose; 

    //             // Store current cluster pose.
    //             initial_cluster_pose_history_.emplace_back(c_cluster_pose);

    //             // Add cluster to the graph.
    //             cluster_id = map_->addVertex(c_cluster_pose);

    //             // Store information.
    //             c_vertex_ids.push_back(cluster_id);
    //             map_->addLocalClusterPose(local_cluster_pose);
    //             map_->addClusterKeyframeRelation(std::make_pair(cluster_id, c_keyframe_id_));

    //             // Pre-allocates memory.
    //             cluster_size = clusters[i].size();
    //             cluster_l_kpts.reserve(cluster_size);
    //             cluster_r_kpts.reserve(cluster_size);
    //             cluster_l_cam_pts.reserve(cluster_size);

    //             // Build cluster
    //             for (size_t j = 0; j < clusters[i].size(); ++j)
    //             {
    //                 idx = clusters[i][j];
    //                 cluster_l_kpts.emplace_back(l_kpts[idx]);
    //                 cluster_r_kpts.emplace_back(r_kpts[idx]);
    //                 cluster_l_cam_pts.emplace_back(l_cam_pts[idx]);
    //                 cluster_l_descs.push_back(l_descs.row(idx));
    //                 cluster_r_descs.push_back(r_descs.row(idx));
    //             }
    //             Cluster cluster(cluster_id, i, c_keyframe_id_, 
    //                             cluster_l_descs, cluster_r_descs, c_map_camera_pose, 
    //                             cluster_l_kpts, cluster_r_kpts, cluster_l_cam_pts);
    //             clusters_to_close_loop.emplace_back(cluster);

    //             // Clear variables.
    //             cluster_l_kpts.clear();
    //             cluster_r_kpts.clear();
    //             cluster_l_cam_pts.clear();
    //             cluster_l_descs.release();
    //             cluster_r_descs.release();
    //         }

    //         // Send the new clusters to the loop closing thread.
    //         for (size_t i = 0; i < clusters_to_close_loop.size(); ++i)
    //             cluster_queue_->emplaceBack(clusters_to_close_loop[i]);
    //         run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Add vertex (cluster) to graph run time.

    //         // Add edges between clusters of the same frame.
    //         start_module_time = ros::Time::now().toSec();
    //         if (clusters_num > 1)
    //         {
    //             // Retrieve all possible combinations.
    //             std::vector<std::vector<int>> combinations = createCombinations(c_vertex_ids.size(), 2);

    //             int vertex_0_id, vertex_1_id;
    //             tf::Transform edge, vertex_0_pose, vertex_1_pose;
    //             for (size_t i = 0; i < combinations.size(); ++i)
    //             {
    //                 vertex_0_id = c_vertex_ids[combinations[i][0]];
    //                 vertex_1_id = c_vertex_ids[combinations[i][1]];

    //                 vertex_0_pose = map_->getVertexPose(vertex_0_id, true);
    //                 vertex_1_pose = map_->getVertexPose(vertex_1_id, true);

    //                 edge = vertex_0_pose.inverse() * vertex_1_pose;
    //                 map_->addEdge(vertex_0_id, vertex_1_id, edge);
    //             }
    //         }

    //         // Connect this keyframe with the previous.
    //         if (c_keyframe_id_ > 0)
    //         {
    //             // Retrieves the vertices of the previous keyframe.
    //             std::vector<int> p_vertex_ids;
    //             map_->getKeyframeVertices(c_keyframe_id_ - 1, p_vertex_ids);

    //             // Connect only the closest vertices between the two keyframes.
    //             std::vector<int> closest_vertices;
    //             std::vector<tf::Transform> closest_poses;
    //             tf::Transform c_vertex_pose, p_vertex_pose;
    //             double dist, min_dist = std::numeric_limits<double>::max();
    //             for (size_t i = 0; i < c_vertex_ids.size(); ++i)
    //             {
    //                 // Current ith vertex pose. 
    //                 c_vertex_pose = map_->getVertexPose(c_vertex_ids[i], true);

    //                 for (size_t j = 0; j < p_vertex_ids.size(); ++j)
    //                 {
    //                     // Previous ith vertex pose. 
    //                     p_vertex_pose = map_->getVertexPose(p_vertex_ids[j], true);

    //                     dist = poseDiff3D(c_vertex_pose, p_vertex_pose);
    //                     if (dist < min_dist)
    //                     {
    //                         min_dist = dist;

    //                         closest_vertices.clear();
    //                         closest_vertices.push_back(c_vertex_ids[i]);
    //                         closest_vertices.push_back(p_vertex_ids[j]);

    //                         closest_poses.clear();
    //                         closest_poses.emplace_back(c_vertex_pose);
    //                         closest_poses.emplace_back(p_vertex_pose);
    //                     }
    //                 }
    //             }

    //             if (!closest_vertices.empty())
    //             {
    //                 tf::Transform edge = closest_poses[0].inverse() * closest_poses[1];
    //                 map_->addEdge(closest_vertices[0], closest_vertices[1], edge);
    //             }
    //             else
    //             {
    //                 ROS_ERROR("[MM->Includer]: Impossible to connect current and previous keyframe. Graph will have non-connected parts!");
    //             }
    //         }
    //         run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Add edges to graph run time.

    //         // Save map poses to file.
    //         start_module_time = ros::Time::now().toSec();
    //         if (enable_debugging_)
    //         {
    //             std::vector<double> stamps;
    //             std::vector<tf::Transform> keyframe_poses;
    //             map_->getKeyframePoses(stamps, keyframe_poses);
    //             for (size_t i = 0; i < keyframe_poses.size(); ++i)
    //                 keyframe_poses[i] = keyframe_poses[i] * robot2camera_.inverse();
    //             writePoses(poses_file_, stamps, keyframe_poses);

    //             std::vector<std::pair<int, int>> edges_keyframe_ids;
    //             std::vector<std::pair<tf::Transform, tf::Transform>> edges_keyframe_poses;
    //             map_->getLoopClosureEdges(edges_keyframe_ids, edges_keyframe_poses);
    //             for (size_t i = 0; i < edges_keyframe_poses.size(); ++i)
    //             {
    //                 edges_keyframe_poses[i].first = edges_keyframe_poses[i].first * robot2camera_.inverse();
    //                 edges_keyframe_poses[i].second = edges_keyframe_poses[i].second * robot2camera_.inverse();
    //             }
    //             writeLoopClosureEdges(edges_file_, edges_keyframe_ids, edges_keyframe_poses);
    //         }

    //         // Publish updated robot pose.
    //         tf::Transform updated_map_camera_pose = map_->getVertexCameraPose(map_->getCurrentVertexId(), true);
    //         publishPose(stamp, updated_map_camera_pose, camera_map_pub_, camera_frame_id_);
    //         tf::Transform updated_map_robot_pose = updated_map_camera_pose * robot2camera_.inverse();
    //         publishPose(stamp, updated_map_robot_pose, robot_map_pub_, robot_frame_id_);
    //         run_times_.run_time_4 = ros::Time::now().toSec() - start_module_time; // Write graph to disk and publish updated robot pose run time.

    //         // Publish number of keyframes.
    //         if (keyframes_num_pub_.getNumSubscribers() > 0)
    //             publishCounter(c_keyframe_id_ + 1, keyframes_num_pub_);
    //     }
    //     else
    //     {
    //         run_times_.run_time_2 = 0.0;
    //         run_times_.run_time_3 = 0.0;
    //         run_times_.run_time_4 = 0.0;
    //     }

    //     if (enable_debugging_)
    //     {
    //         std::filesystem::remove(frame_path_ + c_frame_name + "_left.png");
    //         std::filesystem::remove(frame_path_ + c_frame_name + "_right.png");
    //     }

    //     // Publishes last iteration performance.
    //     run_times_.run_time_5 = 0.0; // Nothing.
    //     run_times_.total_run_time = ros::Time::now().toSec() - start_callback_time;
    //     if (run_times_pub_.getNumSubscribers() > 0)
    //         publishPerformance(ros::Time::now(), (int)add_keyframe, run_times_, run_times_pub_);
    // }


    void Mapmaker::getRobot2Camera()
    {
        slamon::GetTransform srv;
        if (robot2camera_client_.call(srv))
        {
            camera_frame_id_ = srv.response.tf.header.frame_id;
            tf::poseMsgToTF(srv.response.tf.pose, robot2camera_);

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
                                         const int cluster_global_id,
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

        Cluster cluster(cluster_global_id, cluster_local_id, keyframe_id,
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

    
}