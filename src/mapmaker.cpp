#include "mapmaker.hpp"


namespace slamon
{

    Mapmaker::Mapmaker(Map* map,
                       int min_lc_inliers,
                       bool enable_debugging,
                       double min_dist_keyframe,
                       const std::string& w_path,
                       Queue<Cluster>* cluster_queue,
                       const std::string& robot_frame_id)
    {
        // Inilitialises parameters.
        map_ = map;
        cluster_queue_ = cluster_queue;
        min_lc_inliers_ = min_lc_inliers;
        robot_frame_id_ = robot_frame_id;
        enable_debugging_ = enable_debugging;
        min_dist_keyframe_ = min_dist_keyframe;

        // Set "uninitialised" to the map.
        init_map_ = false;

        // Initialised keyframe counter.
        c_keyframe_id_ = -1;

        // Create debugging directories.
        if (enable_debugging_)
        {
            frame_path_ = w_path + "frames/";

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
        message_filters::Subscriber<slamon::StereoImageAttributes> stereo_attributes_sub;

        // Message sync.
        boost::shared_ptr<Sync_> sync;
        odom_sub.subscribe(nh, "odom", 1);
        stereo_attributes_sub.subscribe(nh, "stereo_attributes", 1);
        sync.reset(new Sync_(SyncPolicy_(10), odom_sub, stereo_attributes_sub));
        sync->registerCallback(boost::bind(&Mapmaker::callback, this, _1, _2));

        // Calls callback when synchronised messages arrive
        ros::spin();
    }


    void Mapmaker::callback(const nav_msgs::OdometryConstPtr& odom_msg,
                            const slamon::StereoImageAttributesConstPtr& stereo_attributes_msg)
    {
        // Time at which the program enters into the callback.
        double start_callback_time = ros::Time::now().toSec();
        
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // From ROS messages to local objects.
        start_module_time = ros::Time::now().toSec();
        Frame c_frame;
        tf::Transform c_odom_robot_pose;
        fromStereoAttributesToFrame(stereo_attributes_msg, c_frame);
        tf::poseMsgToTF(odom_msg->pose.pose, c_odom_robot_pose);
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // From StereoImageAttributes msg to Frame object run time.

        // If enable debugging gets images from disk.
        std::string c_frame_name;
        if (enable_debugging_)
        {
            c_frame_name = convertTo5Digits(stereo_attributes_msg->header.seq);
            c_frame.setLeftImg(cv::imread(frame_path_ + c_frame_name + "_left.png", cv::IMREAD_COLOR));
            c_frame.setRightImg(cv::imread(frame_path_ + c_frame_name + "_right.png", cv::IMREAD_COLOR));
        }

        // Does the map need a new keyframe?
        start_module_time = ros::Time::now().toSec();
        bool add_keyframe = needNewKeyFrame(c_odom_robot_pose, c_frame);
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Need new keyframe? run time.

        // Includes a keyframe if the map needs one.
        if (add_keyframe)
        {
            tf::Transform c_map_camera_pose;
            if (init_map_)
            {
                // {wo}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
                tf::Transform c_odom_camera_pose = c_odom_robot_pose * robot2camera_;

                // {wo}^T_{p_cam} = {wo}^T_{p_robot} * {robot}^T_{cam}.
                tf::Transform p_odom_camera_pose = p_odom_robot_pose_ * robot2camera_;

                // {p_cam}^T_{c_cam} = {p_cam}^T_{wo} * {wo}^T_{c_cam}.
                tf::Transform c_camera_motion = p_odom_camera_pose.inverse() * c_odom_camera_pose;

                // Gets {wm}^T_{p_cam}.
                tf::Transform p_map_camera_pose = map_->getKeyframePose(c_keyframe_id_);

                // {wm}^T_{c_cam} = {wm}^T_{p_cam} * {p_cam}^T_{c_cam}.
                c_map_camera_pose = p_map_camera_pose * c_camera_motion;
            }
            else
            {
                // The first SLAM position is the same as the odometry.
                // {wm}^T_{c_cam} = {wo}^T_{c_robot} * {robot}^T_{cam}.
                c_map_camera_pose = c_odom_robot_pose * robot2camera_;
            }

            // Updates global variables.
            init_map_ = true;
            c_keyframe_id_++;
            p_odom_robot_pose_ = c_odom_robot_pose;
            
            // Get frame visual information.
            ros::Time stamp = c_frame.getTimestamp();
            cv::Mat l_descs = c_frame.getLeftDescs();
            cv::Mat r_descs = c_frame.getRightDescs();
            std::vector<cv::KeyPoint> l_kpts = c_frame.getLeftKpts();
            std::vector<cv::KeyPoint> r_kpts = c_frame.getRightKpts();
            std::vector<cv::Point3f> l_cam_pts = c_frame.getLeftCameraPoints();
            std::vector<cv::Point3f> cluster_centroids = c_frame.getClustersCentroids();
            std::vector<std::vector<int>> clusters = c_frame.getClusters();

            // Debug keyframe.
            if (enable_debugging_)
            {
                std::string c_keyframe_name = convertTo5Digits(c_keyframe_id_);
                cv::imwrite(keyframe_path_ + c_keyframe_name + "_left.png", c_frame.getLeftImg());
                cv::imwrite(keyframe_path_ + c_keyframe_name + "_right.png", c_frame.getRightImg());
                if (clustering_img_pub_.getNumSubscribers() > 0)
                    publishImgKeypointsClustering(c_frame.getLeftImg(), stamp, clustering_img_pub_, l_kpts, clusters);
                writeImgKeypointsClustering(c_frame.getLeftImg(), clusters_path_ + c_keyframe_name, l_kpts, clusters);
                ROS_INFO_STREAM("[MM->Includer]: Adding keyframe: " << c_keyframe_id_);
            }

            // Save the frame timestamp
            map_->addTimestamp(stamp.toSec());
            
            // Loop of frame clusters.
            start_module_time = ros::Time::now().toSec();
            int cluster_id, idx;
            std::vector<int> c_vertex_ids;
            cv::Mat cluster_l_descs, cluster_r_descs;
            std::vector<cv::Point3f> cluster_l_cam_pts;
            std::vector<cv::KeyPoint> cluster_l_kpts, cluster_r_kpts;
            std::vector<Cluster> clusters_to_close_loop;
            tf::Transform local_cluster_pose, c_cluster_pose;
            for (uint i = 0; i < clusters.size(); i++)
            {
                // Gets {c_cam}^T_{c_cluster}.
                local_cluster_pose = fromPoint3DToTf(cluster_centroids[i]);

                // {w}^T_{c_cluster} = {w}^T_{c_cam} * {c_cam}^T_{c_cluster}.
                c_cluster_pose = c_map_camera_pose * local_cluster_pose; 

                // Store current cluster pose.
                initial_cluster_pose_history_.push_back(c_cluster_pose);

                // Add cluster to the graph.
                cluster_id = map_->addVertex(c_cluster_pose);

                // Store information.
                c_vertex_ids.push_back(cluster_id);
                map_->addLocalClusterPose(local_cluster_pose);
                map_->addClusterKeyframeRelation(std::make_pair(cluster_id, c_keyframe_id_));

                // Build cluster
                for (uint j = 0; j < clusters[i].size(); j++)
                {
                    idx = clusters[i][j];
                    cluster_l_kpts.push_back(l_kpts[idx]);
                    cluster_r_kpts.push_back(r_kpts[idx]);
                    cluster_l_cam_pts.push_back(l_cam_pts[idx]);
                    cluster_l_descs.push_back(l_descs.row(idx));
                    cluster_r_descs.push_back(r_descs.row(idx));
                }
                Cluster cluster(cluster_id, i, c_keyframe_id_, 
                                cluster_l_descs, cluster_r_descs, c_map_camera_pose, 
                                cluster_l_kpts, cluster_r_kpts, cluster_l_cam_pts);
                clusters_to_close_loop.push_back(cluster);

                // Clear variables
                cluster_l_kpts.clear();
                cluster_r_kpts.clear();
                cluster_l_cam_pts.clear();
                cluster_l_descs.release();
                cluster_r_descs.release();
            }

            // Send the new clusters to the loop closing thread.
            for (uint i = 0; i < clusters_to_close_loop.size(); i++)
                cluster_queue_->pushBack(clusters_to_close_loop[i]);
            run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Add vertex (cluster) to graph run time.

            // Add edges between clusters of the same frame.
            start_module_time = ros::Time::now().toSec();
            if (clusters.size() > 1)
            {
                // Retrieve all possible combinations.
                std::vector<std::vector<int>> combinations = createCombinations(c_vertex_ids.size(), 2);

                int vertex_0_id, vertex_1_id;
                tf::Transform edge, vertex_0_pose, vertex_1_pose;
                for (uint i = 0; i < combinations.size(); i++)
                {
                    vertex_0_id = c_vertex_ids[combinations[i][0]];
                    vertex_1_id = c_vertex_ids[combinations[i][1]];

                    vertex_0_pose = map_->getVertexPose(vertex_0_id, true);
                    vertex_1_pose = map_->getVertexPose(vertex_1_id, true);

                    edge = vertex_0_pose.inverse() * vertex_1_pose;
                    map_->addEdge(vertex_0_id, vertex_1_id, edge);
                }
            }
                
            // Connect this keyframe with the previous
            if (c_keyframe_id_ > 0)
            {
                // Retrieves the vertices of the previous keyframe.
                std::vector<int> p_vertex_ids;
                map_->getKeyframeVertices(c_keyframe_id_ - 1, p_vertex_ids);

                // Connect only the closest vertices between the two keyframes.
                std::vector<int> closest_vertices;
                std::vector<tf::Transform> closest_poses;
                tf::Transform c_vertex_pose, p_vertex_pose;
                double dist, min_dist = std::numeric_limits<double>::max();
                for (uint i = 0; i < c_vertex_ids.size(); i++)
                {
                    // Current ith vertex pose. 
                    c_vertex_pose = map_->getVertexPose(c_vertex_ids[i], true);

                    for (uint j = 0; j < p_vertex_ids.size(); j++)
                    {
                        // Previous ith vertex pose. 
                        p_vertex_pose = map_->getVertexPose(p_vertex_ids[j], true);

                        dist = poseDiff3D(c_vertex_pose, p_vertex_pose);
                        if (dist < min_dist)
                        {
                            min_dist = dist;

                            closest_vertices.clear();
                            closest_vertices.push_back(c_vertex_ids[i]);
                            closest_vertices.push_back(p_vertex_ids[j]);

                            closest_poses.clear();
                            closest_poses.push_back(c_vertex_pose);
                            closest_poses.push_back(p_vertex_pose);
                        }
                    }
                }

                if (closest_vertices.size() > 0)
                {
                    tf::Transform edge = closest_poses[0].inverse() * closest_poses[1];
                    map_->addEdge(closest_vertices[0], closest_vertices[1], edge);
                }
                else
                {
                    ROS_ERROR("[MM->Includer]: Impossible to connect current and previous keyframe. Graph will have non-connected parts!");
                }
            }
            run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Add edges to graph run time.

            // Save map poses to file.
            start_module_time = ros::Time::now().toSec();
            if (enable_debugging_)
            {
                std::vector<double> stamps;
                std::vector<int> keyframe_ids;
                std::vector<tf::Transform> keyframe_poses;
                map_->getKeyframePoses(stamps, keyframe_ids, keyframe_poses);
                for (uint i = 0; i < keyframe_poses.size(); i++)
                    keyframe_poses[i] = keyframe_poses[i] * robot2camera_.inverse();
                writePoses(poses_file_, keyframe_ids, stamps, keyframe_poses);

                std::vector<std::pair<int, int>> edges_keyframe_ids;
                std::vector<std::pair<tf::Transform, tf::Transform>> edges_keyframe_poses;
                map_->getLoopClosureEdges(edges_keyframe_ids, edges_keyframe_poses);
                for (uint i = 0; i < edges_keyframe_poses.size(); i++)
                {
                    edges_keyframe_poses[i].first = edges_keyframe_poses[i].first * robot2camera_.inverse();
                    edges_keyframe_poses[i].second = edges_keyframe_poses[i].second * robot2camera_.inverse();
                }
                writeLoopClosureEdges(edges_file_, edges_keyframe_ids, edges_keyframe_poses);
            }

            // Publish updated robot pose.
            tf::Transform updated_map_camera_pose = map_->getVertexCameraPose(map_->getCurrentVertexId(), true);
            publishPose(stamp, updated_map_camera_pose, camera_map_pub_, camera_frame_id_);
            tf::Transform updated_map_robot_pose = updated_map_camera_pose * robot2camera_.inverse();
            publishPose(stamp, updated_map_robot_pose, robot_map_pub_, robot_frame_id_);
            run_times_.run_time_4 = ros::Time::now().toSec() - start_module_time; // Write graph to disk and publish updated robot pose run time.

            // Publish number of keyframes.
            if (keyframes_num_pub_.getNumSubscribers() > 0)
                publishCounter(c_keyframe_id_ + 1, keyframes_num_pub_);
        }
        else
        {
            run_times_.run_time_2 = 0.0;
            run_times_.run_time_3 = 0.0;
            run_times_.run_time_4 = 0.0;
        }

        if (enable_debugging_)
        {
            std::filesystem::remove(frame_path_ + c_frame_name + "_left.png");
            std::filesystem::remove(frame_path_ + c_frame_name + "_right.png");
        }

        // Publishes last iteration performance.
        run_times_.run_time_5 = 0.0; // Nothing.
        run_times_.total_run_time = ros::Time::now().toSec() - start_callback_time;
        if (run_times_pub_.getNumSubscribers() > 0)
            publishPerformance(ros::Time::now(), (int)add_keyframe, run_times_, run_times_pub_);
    }


    bool Mapmaker::needNewKeyFrame(const tf::Transform& c_odom_robot_pose,
                                   Frame& c_frame)
    {
        if (!init_map_)
        {
            if (addFrameToMap(c_frame))
            {
                getRobot2Camera();
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (poseDiff2D(p_odom_robot_pose_, c_odom_robot_pose) > min_dist_keyframe_)
        {
            return addFrameToMap(c_frame);
        }
        else
        {
            return false;
        }
    }


    bool Mapmaker::addFrameToMap(Frame& c_frame)
    {
        if (c_frame.getLeftKpts().size() > (uint)(min_lc_inliers_/2))
        {
            c_frame.regionClustering();

            // Check if enough clusters
            std::vector<std::vector<int>> clusters = c_frame.getClusters();
            if (clusters.size() > 0)
            {
                return true;
            }
            else
            {
                ROS_WARN_STREAM("[MM->Includer]: Image produces 0 clusters.");
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("[MM->Includer]: Not enough keypoints in this frame (" << c_frame.getLeftKpts().size() << ")");
            return false;
        }
    }


    void Mapmaker::getRobot2Camera()
    {
        slamon::GetTransform srv;
        if (robot2camera_client_.call(srv))
        {
            camera_frame_id_ = srv.response.tf.header.frame_id;
            double tx = srv.response.tf.pose.position.x;
            double ty = srv.response.tf.pose.position.y;
            double tz = srv.response.tf.pose.position.z;
            double qx = srv.response.tf.pose.orientation.x;
            double qy = srv.response.tf.pose.orientation.y;
            double qz = srv.response.tf.pose.orientation.z;
            double qw = srv.response.tf.pose.orientation.w;

            ROS_WARN_STREAM("[MM->Static transform:] GetTransform service call successful. The robot2camera transform is as follows:" << std::endl <<
                            "tx: " << tx << std::endl << 
                            "ty: " << ty << std::endl << 
                            "tz: " << tz << std::endl << 
                            "qx: " << qx << std::endl << 
                            "qy: " << qy << std::endl << 
                            "qz: " << qz << std::endl << 
                            "qw: " << qw);

            tf::Vector3 translation(tx, ty, tz);
            tf::Quaternion rotation(qx, qy, qz, qw);
            robot2camera_.setOrigin(translation);
            robot2camera_.setRotation(rotation);
        }
        else
        {
            ROS_WARN_STREAM("[MM->Static transform:] Failed to call GetTransform service. The robot2camera transform will be assumed as the identity.");
            robot2camera_.setIdentity();
        }
    }


    std::vector<std::vector<int>> Mapmaker::createCombinations(const uint n,
                                                               const uint r)
    {
        std::string bitmask(r, 1);
        bitmask.resize(n, 0);

        std::vector<int> combination;
        std::vector<std::vector<int>> combinations;
        do
        {
            for (uint i = 0; i < n; i++)
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


    void Mapmaker::fromStereoAttributesToFrame(const slamon::StereoImageAttributesConstPtr& msg,
                                               Frame& frame)
    {
        cv::KeyPoint kpt;
        cv::Point3f cam_pt;
        std::vector<float> desc;
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> l_cam_pts;
        std::vector<cv::KeyPoint> l_kpts, r_kpts;
        for (uint i = 0; i < msg->l_kpts.size(); i++)
        {
            kpt.pt.x = msg->l_kpts[i].pt.x;
            kpt.pt.y = msg->l_kpts[i].pt.y;
            l_kpts.push_back(kpt);
            kpt.pt.x = msg->r_kpts[i].pt.x;
            kpt.pt.y = msg->r_kpts[i].pt.y;
            r_kpts.push_back(kpt);
            desc = msg->l_kpts[i].desc;
            l_descs.push_back(cv::Mat(1, desc.size(), CV_32F, desc.data()));
            desc = msg->r_kpts[i].desc;
            r_descs.push_back(cv::Mat(1, desc.size(), CV_32F, desc.data()));
            cam_pt.x = msg->l_cam_pts[i].x;
            cam_pt.y = msg->l_cam_pts[i].y;
            cam_pt.z = msg->l_cam_pts[i].z;
            l_cam_pts.push_back(cam_pt);
        }

        frame = Frame(l_descs, r_descs, msg->header.stamp, l_kpts, r_kpts, l_cam_pts);
    }


    tf::Transform Mapmaker::fromPoint3DToTf(const cv::Point3f& pt)
    {
        tf::Transform out;
        out.setIdentity();
        tf::Vector3 translation(pt.x, pt.y, pt.z);
        out.setOrigin(translation);
        return out;
    }

    
}