#include "map2.hpp"


namespace slam
{

    Map2::Map2(int min_lc_inliers,
               bool enable_debugging,
               double min_dist_keyframe,
               const std::string& w_path,
               LoopClosing* loop_closing,
               const std::string& robot_frame_id)
    {
        // Inilitialises parameters.
        w_path_ = w_path;
        loop_closing_ = loop_closing;
        min_lc_inliers_ = min_lc_inliers;
        robot_frame_id_ = robot_frame_id;
        enable_debugging_ = enable_debugging;
        min_dist_keyframe_ = min_dist_keyframe;

        // Set "uninitialised" to the map.
        init_map_ = false;

        // Initialised keyframe counter.
        c_keyframe_id_ = -1;

        // Inilitialises the g2o graph optimizer.
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr (new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>());
        std::unique_ptr<g2o::BlockSolverX> solver_ptr (new g2o::BlockSolverX(std::move(linear_solver_ptr)));
        g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
        graph_.setAlgorithm(solver);

        // Create debugging directories.
        if (enable_debugging_)
        {
            frame_path_ = w_path_ + "frames/";
            keyframe_path_ = w_path_ + "keyframes/";
            clusters_path_ = w_path_ + "clusters/";
            std::string poses_file = w_path_ + "keyframes_poses.txt";
            std::string lock_file = w_path_ + "keyframes_poses.txt.lock";
            if (std::filesystem::exists(lock_file))
                std::filesystem::remove(lock_file.c_str());
            if (std::filesystem::exists(poses_file))
                std::filesystem::remove(poses_file.c_str());
            if (std::filesystem::exists(keyframe_path_))
                std::filesystem::remove_all(keyframe_path_);
            if (std::filesystem::exists(clusters_path_))
                std::filesystem::remove_all(clusters_path_);
            std::filesystem::create_directory(keyframe_path_);
            std::filesystem::create_directory(clusters_path_);
        }
    }


    void Map2::run()
    {
        // Create ROS node.
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");

        // Client services.
        robot2camera_client_ = nh.serviceClient<slamon::GetTransform>("get_robot_to_camera_tf");

        // Publishers.
        map_pub_ = nhp.advertise<nav_msgs::Odometry>("map", 1);
        run_times_pub_ = nhp.advertise<slamon::Performance>("map_performance", 1);
        keyframes_num_pub_ = nhp.advertise<std_msgs::Int32>("keyframe_num", 1);
        clustering_img_pub_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering_img", 1);

        // Subscribers.
        message_filters::Subscriber<slamon::Frame> frame_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

        // Message sync.
        boost::shared_ptr<Sync_> sync;
        frame_sub.subscribe(nh, "frame", 5);
        odom_sub.subscribe(nh, "odom", 5);
        sync.reset(new Sync_(SyncPolicy_(10), frame_sub, odom_sub));
        sync->registerCallback(boost::bind(&Map2::frameCallback, this, _1, _2));

        // Calls callback when synchronised messages arrive
        ros::spin();
    }


    int Map2::getVertexKeyframeId(int vertex_id)
    {
        int frame_id = -1;
        for (uint i = 0; i < cluster_keyframe_relation_.size(); i++)
        {
            if (cluster_keyframe_relation_[i].first == vertex_id)
            {
                frame_id = cluster_keyframe_relation_[i].second;
                break;
            }
        }
        return frame_id;
    }
    

    tf::Transform Map2::getVertexPose(int vertex_id, bool lock = true)
    {
        if (lock)
        {
            std::scoped_lock lock(mutex_graph_);

            tf::Transform vertex_pose;
            if (vertex_id >= 0)
            {
                g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
                vertex_pose = fromVertexToTf(vertex);
            }
            else
            {
                vertex_pose.setIdentity();
            }
            return vertex_pose;
        }
        else
        {
            tf::Transform vertex_pose;
            if (vertex_id >= 0)
            {
                g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
                vertex_pose = fromVertexToTf(vertex);
            }
            else
            {
                vertex_pose.setIdentity();
            }
            return vertex_pose;
        }
    }


    tf::Transform Map2::getVertexPoseRelativeToCamera(int vertex_id)
    {
        return local_cluster_poses_[vertex_id];
    }


    tf::Transform Map2::getVertexCameraPose(int vertex_id, bool lock = true)
    {
        tf::Transform vertex_pose = getVertexPose(vertex_id, lock);
        return vertex_pose * local_cluster_poses_[vertex_id].inverse();
    }


    void Map2::getKeyframeVertices(int keyframe_id, std::vector<int>& vertices)
    {
        vertices.clear();
        for (uint i = 0; i < cluster_keyframe_relation_.size(); i++)
        {
            if (cluster_keyframe_relation_[i].second == keyframe_id)
                vertices.push_back(cluster_keyframe_relation_[i].first);
        }
    }


    void Map2::getKeyframePoses(std::vector<double>& stamps,
                               std::vector<tf::Transform>& keyframe_poses)
    {
        // Locks graph
        std::scoped_lock lock(mutex_graph_);

        // Get keyframe poses
        bool found;
        int keyframe_id;
        std::vector<int> processed_keyframes;
        for (uint vertex_id = 0; vertex_id < graph_.vertices().size(); vertex_id++)
        {
            // Is this keyframe processed?
            found = false;
            keyframe_id = getVertexKeyframeId(vertex_id);
            // for (uint processed_keyframe_id = processed_keyframes.size() - 1; processed_keyframe_id >= 0; processed_keyframe_id--)
            for (uint processed_keyframe_id = 0; processed_keyframe_id < processed_keyframes.size(); processed_keyframe_id++)
            {
                if (processed_keyframes[processed_keyframe_id] == keyframe_id)
                {
                    found = true;
                    break;
                }
            }
            if (found)
                continue;

            processed_keyframes.push_back(keyframe_id);
            stamps.push_back(keyframe_stamps_[keyframe_id]);
            keyframe_poses.push_back(getVertexCameraPose(vertex_id, false) * robot2camera_.inverse());
        }
    }


    void Map2::addEdge(int vertex_0_id,
                       int vertex_1_id,
                       tf::Transform tf_edge)
    {
        std::scoped_lock lock(mutex_graph_);

        // Get the graph vertices.
        g2o::VertexSE3* vertex_0 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_0_id]);
        g2o::VertexSE3* vertex_1 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_1_id]);

        // Add graph edge to graph.
        g2o::EdgeSE3* graph_edge = new g2o::EdgeSE3();
        Eigen::Isometry3d iso_edge = fromTfToIsometry(tf_edge);
        graph_edge->setVertex(0, vertex_0);
        graph_edge->setVertex(1, vertex_1);
        graph_edge->setMeasurement(iso_edge);

        graph_.addEdge(graph_edge);
    }


    void Map2::findClosestVertices(int vertex_id,
                                  int window_center,
                                  int discard_window,
                                  uint closest_neighbors_num,
                                  std::vector<int>& closest_neighbors)
    {
        // Init.
        closest_neighbors.clear();
        tf::Transform q_vertex_pose = getVertexPose(vertex_id);

        // Loop thought all the other nodes.
        double dist;
        tf::Transform neighbor_vertex_pose;
        std::vector<std::pair<int, double>> neighbors;
        for (int i = 0; i < (int)graph_.vertices().size(); i++)
        {
            // Avoids itself.
            if (i == vertex_id)
                continue;

            // Discards window.
            if ((i > window_center - discard_window) && (i < window_center + discard_window))
                continue;

            // Gets the distance between the queried vertex and the ith vertex.
            neighbor_vertex_pose = getVertexPose(i);
            dist = poseDiff2D(neighbor_vertex_pose, q_vertex_pose);
            neighbors.push_back(std::make_pair(i, dist));
        }

        // Exits if no neighbors.
        if (neighbors.size() == 0)
            return;

        // Sorts the neighbours from sleast to furthest distance to the query neighbour.
        std::sort(neighbors.begin(), neighbors.end(), sortByDistance);

        // Gets closest vertices (minnor distance)
        if (closest_neighbors_num > neighbors.size())
            closest_neighbors_num = neighbors.size();
        for (uint i = 0; i < closest_neighbors_num; i++)
            closest_neighbors.push_back(neighbors[i].first);
    }


    void Map2::update()
    {
        std::scoped_lock lock(mutex_graph_);

        // Optimize!
        graph_.initializeOptimization();
        graph_.optimize(20);

        ROS_INFO_STREAM("[Map->updater:] Optimization done in graph with: " << graph_.vertices().size() << " vertices.");
    }


    void Map2::frameCallback(const slamon::FrameConstPtr& frame_msg,
                             const nav_msgs::OdometryConstPtr& odom_msg)
    {
        // Time at which the program enters into the callback.
        double start_callback_time = ros::Time::now().toSec();
        
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // From ROS messages to local objects.
        start_module_time = ros::Time::now().toSec();
        Frame c_frame;
        tf::Transform c_robot_pose;
        fromFrameMsgToFrameObject(frame_msg, c_frame);
        tf::poseMsgToTF(odom_msg->pose.pose, c_robot_pose);
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // From Frame msg to Frame object run time.

        // If enable debugging gets images from disk.
        std::string c_frame_name;
        if (enable_debugging_)
        {
            c_frame_name = convertTo5Digits(frame_msg->header.seq);
            c_frame.setLeftImg(cv::imread(frame_path_ + c_frame_name + "_left.png", cv::IMREAD_COLOR));
            c_frame.setRightImg(cv::imread(frame_path_ + c_frame_name + "_right.png", cv::IMREAD_COLOR));
        }

        // Does the map need a new keyframe?
        start_module_time = ros::Time::now().toSec();
        bool add_keyframe = needNewKeyFrame(c_frame, c_robot_pose);
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Need new keyframe? run time.

        // Includes a keyframe if the map needs one.
        if (add_keyframe)
        {
            // Updates global variables.
            init_map_ = true;
            c_keyframe_id_++;
            p_keyframe_robot_pose_ = c_robot_pose;
            
            // Get frame visual information.
            ros::Time stamp = c_frame.getTimestamp();
            cv::Mat l_descs = c_frame.getLeftDescs();
            cv::Mat r_descs = c_frame.getRightDescs();
            std::vector<cv::Point2f> l_kpts = c_frame.getLeftKpts();
            std::vector<cv::Point2f> r_kpts = c_frame.getRightKpts();
            std::vector<cv::Point3f> cam_pts = c_frame.getCameraPoints();
            std::vector<cv::Point3f> cluster_centroids = c_frame.getClustersCentroids();
            std::vector<std::vector<int>> clusters = c_frame.getClusters();

            // Debug keyframe.
            if (enable_debugging_)
            {
                std::string c_keyframe_name = convertTo5Digits(c_keyframe_id_);
                cv::imwrite(keyframe_path_ + c_keyframe_name + "_left.png", c_frame.getLeftImg());
                cv::imwrite(keyframe_path_ + c_keyframe_name + "_right.png", c_frame.getRightImg());
                if (clustering_img_pub_.getNumSubscribers() > 0)
                    publishKeypointsClustering(c_frame.getLeftImg(), stamp, clustering_img_pub_, l_kpts, clusters);
                writeKeypointsClustering(c_frame.getLeftImg(), clusters_path_ + c_keyframe_name, l_kpts, clusters);
                ROS_INFO_STREAM("[MAP->Includer]: Adding keyframe: " << c_keyframe_id_);
            }

            // Save the frame timestamp
            keyframe_stamps_.push_back(stamp.toSec());
            
            // Loop of frame clusters.
            start_module_time = ros::Time::now().toSec();
            int cluster_id, idx;
            std::vector<int> c_vertex_ids;
            cv::Mat cluster_l_descs, cluster_r_descs;
            std::vector<cv::Point3f> cluster_cam_pts;
            std::vector<cv::Point2f> cluster_l_kpts, cluster_r_kpts;
            std::vector<Cluster> clusters_to_close_loop;
            tf::Transform local_cluster_pose, c_cluster_pose;
            tf::Transform c_camera_pose = c_robot_pose * robot2camera_; // {o}^T_{c} = {o}^T_{r} * {r}^T_{c}
            for (uint i = 0; i < clusters.size(); i++)
            {
                local_cluster_pose = fromPoint3DToTf(cluster_centroids[i]); // {c}^T_{cluster}
                c_cluster_pose = c_camera_pose * local_cluster_pose; // {o}^T_{cluster} = {o}^T_{c} * {c}^T_{cluster}
                initial_cluster_pose_history_.push_back(c_cluster_pose);

                // Add cluster to the graph.
                cluster_id = addVertex(c_cluster_pose);

                // Store information.
                c_vertex_ids.push_back(cluster_id);
                local_cluster_poses_.push_back(local_cluster_pose);
                cluster_keyframe_relation_.push_back(std::make_pair(cluster_id, c_keyframe_id_)); // Cluster and Frame id.

                // Build cluster
                for (uint j = 0; j < clusters[i].size(); j++)
                {
                    idx = clusters[i][j];
                    cluster_l_kpts.push_back(l_kpts[idx]);
                    cluster_r_kpts.push_back(r_kpts[idx]);
                    cluster_cam_pts.push_back(cam_pts[idx]);
                    cluster_l_descs.push_back(l_descs.row(idx));
                    cluster_r_descs.push_back(r_descs.row(idx));
                }
                Cluster cluster(cluster_id, c_keyframe_id_, cluster_l_descs, cluster_r_descs, c_camera_pose, cluster_l_kpts, cluster_r_kpts, cluster_cam_pts);
                clusters_to_close_loop.push_back(cluster);

                // Clear variables
                cluster_l_kpts.clear();
                cluster_r_kpts.clear();
                cluster_cam_pts.clear();
                cluster_l_descs.release();
                cluster_r_descs.release();
            }

            // Send the new clusters to the loop closing thread.
            for (uint i = 0; i < clusters_to_close_loop.size(); i++)
                loop_closing_->addClusterToQueue(clusters_to_close_loop[i]);
            run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Add vertex (cluster) to graph run time.

            // Add edges between clusters of the same frame.
            start_module_time = ros::Time::now().toSec();
            if (clusters.size() > 1)
            {
                // Retrieve all possible combinations.
                std::vector<std::vector<int>> combinations = createComb(c_vertex_ids);

                int vertex_0_id, vertex_1_id;
                tf::Transform edge, vertex_0_pose, vertex_1_pose;
                for (uint i = 0; i < combinations.size(); i++)
                {
                    vertex_0_id = c_vertex_ids[combinations[i][0]];
                    vertex_1_id = c_vertex_ids[combinations[i][1]];

                    vertex_0_pose = getVertexPose(vertex_0_id);
                    vertex_1_pose = getVertexPose(vertex_1_id);

                    edge = vertex_0_pose.inverse() * vertex_1_pose;
                    addEdge(vertex_0_id, vertex_1_id, edge);
                }
            }
                
            // Connect this keyframe with the previous
            if (c_keyframe_id_ > 0)
            {
                // Retrieves the vertices of the previous keyframe.
                std::vector<int> p_vertex_ids;
                getKeyframeVertices(c_keyframe_id_ - 1, p_vertex_ids);

                // Connect only the closest vertices between the two keyframes.
                std::vector<int> closest_vertices;
                std::vector<tf::Transform> closest_poses;
                tf::Transform c_vertex_pose, p_vertex_pose;
                double dist, min_dist = std::numeric_limits<double>::max();
                for (uint i = 0; i < c_vertex_ids.size(); i++)
                {
                    // Current ith vertex pose. 
                    c_vertex_pose = getVertexPose(c_vertex_ids[i]);

                    for (uint j = 0; j < p_vertex_ids.size(); j++)
                    {
                        // Previous ith vertex pose. 
                        p_vertex_pose = getVertexPose(p_vertex_ids[j]);

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
                    addEdge(closest_vertices[0], closest_vertices[1], edge);
                }
                else
                {
                    ROS_ERROR("[MAP->Includer]: Impossible to connect current and previous keyframe. Graph will have non-connected parts!");
                }
            }
            run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Add edges to graph run time.

            // Save map poses to file.
            start_module_time = ros::Time::now().toSec();
            if (enable_debugging_)
            {
                std::vector<double> stamps;
                std::vector<tf::Transform> keyframe_poses;
                getKeyframePoses(stamps, keyframe_poses);
                writePoses(w_path_ + "keyframes_poses.txt", stamps, keyframe_poses);
            }

            // Publish updated robot pose.
            int c_vertex_id = -1;
            {
                std::scoped_lock lock(mutex_graph_);
                c_vertex_id = graph_.vertices().size() - 1;
            }
            tf::Transform updated_robot_pose = getVertexCameraPose(c_vertex_id, true) * robot2camera_.inverse();
            publishPose(stamp, map_pub_, "map", robot_frame_id_, updated_robot_pose);
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


    bool Map2::needNewKeyFrame(Frame& c_frame, const tf::Transform& c_robot_pose)
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
        else if (poseDiff2D(p_keyframe_robot_pose_, c_robot_pose) > min_dist_keyframe_)
        {
            return addFrameToMap(c_frame);
        }
        else
        {
            return false;
        }
    }


    bool Map2::addFrameToMap(Frame& c_frame)
    {
        if (c_frame.getLeftKpts().size() > (long unsigned int)(min_lc_inliers_/2))
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
                ROS_WARN_STREAM("[GRAPH->Map includer]: Image produces 0 clusters.");
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("[GRAPH->Map includer]: Not enough keypoints in this frame (" << c_frame.getLeftKpts().size() << ")");
            return false;
        }
    }


    void Map2::getRobot2Camera()
    {
        slamon::GetTransform srv;
        if (robot2camera_client_.call(srv))
        {
            double tx = srv.response.tf.position.x;
            double ty = srv.response.tf.position.y;
            double tz = srv.response.tf.position.z;
            double qx = srv.response.tf.orientation.x;
            double qy = srv.response.tf.orientation.y;
            double qz = srv.response.tf.orientation.z;
            double qw = srv.response.tf.orientation.w;

            ROS_WARN_STREAM("[GRAPH->Static transform:] GetTransform service call successful. The robot2camera transform is as follows:" << std::endl <<
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
            ROS_WARN_STREAM("[GRAPH->Static transform:] Failed to call GetTransform service. The robot2camera transform will be assumed as the identity.");
            robot2camera_.setIdentity();
        }
    }


    int Map2::addVertex(tf::Transform pose)
    {
        // Lock graph.
        std::scoped_lock lock(mutex_graph_);

        // Convert pose for graph
        Eigen::Isometry3d vertex_pose = fromTfToIsometry(pose);

        // Set node id equal to graph size
        int id = graph_.vertices().size();

        // Build the vertex
        g2o::VertexSE3* c_vertex = new g2o::VertexSE3();
        c_vertex->setId(id);
        c_vertex->setEstimate(vertex_pose);
        
        // First time, no edges.
        if (id == 0)
            c_vertex->setFixed(true);

        // Add vertex.
        graph_.addVertex(c_vertex);

        return id;
    }


    std::vector<std::vector<int>> Map2::createComb(const std::vector<int>& vec)
    {
        std::string bitmask(2, 1);
        bitmask.resize(vec.size(), 0);

        std::vector<int> comb;
        std::vector<std::vector<int>> combinations;
        do
        {
            for (uint i = 0; i < vec.size(); i++)
            {
                if (bitmask[i])
                    comb.push_back(i);
            }
            combinations.push_back(comb);
            comb.clear();
        }
        while (std::prev_permutation(bitmask.begin(), bitmask.end()));

        return combinations;
    }


    void Map2::fromFrameMsgToFrameObject(const slamon::FrameConstPtr& msg,
                                         Frame& frame)
    {
        cv::Point2f kpt;
        cv::Point3f cam_pt;
        std::vector<float> desc;
        cv::Mat l_descs, r_descs;
        std::vector<cv::Point3f> cam_pts;
        std::vector<cv::Point2f> l_kpts, r_kpts;
        for (uint i = 0; i < msg->l_kpts.size(); i++)
        {
            kpt.x = msg->l_kpts[i].pt.x;
            kpt.y = msg->l_kpts[i].pt.y;
            l_kpts.push_back(kpt);
            kpt.x = msg->r_kpts[i].pt.x;
            kpt.y = msg->r_kpts[i].pt.y;
            r_kpts.push_back(kpt);
            desc = msg->l_kpts[i].desc;
            l_descs.push_back(cv::Mat(1, desc.size(), CV_32F, desc.data()));
            desc = msg->r_kpts[i].desc;
            r_descs.push_back(cv::Mat(1, desc.size(), CV_32F, desc.data()));
            cam_pt.x = msg->cam_pts[i].x;
            cam_pt.y = msg->cam_pts[i].y;
            cam_pt.z = msg->cam_pts[i].z;
            cam_pts.push_back(cam_pt);
        }

        frame = Frame(l_descs, r_descs, msg->header.stamp, l_kpts, r_kpts, cam_pts);
    }


    tf::Transform Map2::fromPoint3DToTf(const cv::Point3f& pt)
    {
        tf::Transform out;
        out.setIdentity();
        tf::Vector3 translation(pt.x, pt.y, pt.z);
        out.setOrigin(translation);
        return out;
    }


    Eigen::Isometry3d Map2::fromTfToIsometry(const tf::Transform& tf)
    {
        tf::Vector3 t_in = tf.getOrigin();
        tf::Quaternion q_in = tf.getRotation();
        Eigen::Vector3d t_out(t_in.getX(), t_in.getY(), t_in.getZ());
        Eigen::Quaterniond q_out;
        q_out.setIdentity();
        q_out.x() = q_in.getX();
        q_out.y() = q_in.getY();
        q_out.z() = q_in.getZ();
        q_out.w() = q_in.getW();
        Eigen::Isometry3d out = (Eigen::Isometry3d)q_out;
        out.translation() = t_out;
        return out;
    }


    tf::Transform Map2::fromIsometryToTf(const Eigen::Isometry3d& iso)
    {
        Eigen::Vector3d t_in = iso.translation();
        Eigen::Quaterniond q_in = (Eigen::Quaterniond)iso.rotation();
        tf::Vector3 t_out(t_in.x(), t_in.y(), t_in.z());
        tf::Quaternion q_out(q_in.x(), q_in.y(), q_in.z(), q_in.w());
        tf::Transform out(q_out, t_out);
        return out;
    }


    tf::Transform Map2::fromVertexToTf(const g2o::VertexSE3* vertex)
    {
        Eigen::Isometry3d iso = vertex->estimate();
        tf::Transform tf = fromIsometryToTf(iso);
        return tf;
    }

    
}