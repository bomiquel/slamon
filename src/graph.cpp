#include "graph.hpp"


Graph::Graph(int min_lc_inliers,
             double min_dist_kf,
             bool enable_debugging,
             const std::string& w_path)
{
    w_path_ = w_path;
    kf_counter_ = -1;
    init_graph_ = false;
    min_dist_kf_ = min_dist_kf;
    min_lc_inliers_ = min_lc_inliers;
    enable_debugging_ = enable_debugging;
}


void Graph::run()
{
    // Create debugging directories.
    if (enable_debugging_)
    {
        f_path_ = w_path_ + "frames/";
        kf_path_ = w_path_ + "keyframes/";
        clusters_path_ = w_path_ + "clusters/";
        if (std::filesystem::exists(kf_path_))
            std::filesystem::remove_all(kf_path_);
        if (std::filesystem::exists(clusters_path_))
            std::filesystem::remove_all(clusters_path_);
        std::filesystem::create_directory(kf_path_);
        std::filesystem::create_directory(clusters_path_);
    }

    // Create ROS node.
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Client serives.
    robot2camera_client_ = nh.serviceClient<slamon::GetTransform>("get_robot_to_camera_tf");

    // Publishers.
    clustering_img_pub_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering_img", 1);

    // Subscribers.
    message_filters::Subscriber<slamon::Frame> frame_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

    // Message sync.
    boost::shared_ptr<Sync_> sync;
    frame_sub.subscribe(nh, "frame", 5);
    odom_sub.subscribe(nh, "odom", 5);
    sync.reset(new Sync_(SyncPolicy_(10), frame_sub, odom_sub));
    sync->registerCallback(boost::bind(&Graph::frameCallback, this, _1, _2));

    //
    ros::spin();
}


void Graph::frameCallback(const slamon::FrameConstPtr& frame_msg,
                          const nav_msgs::OdometryConstPtr& odom_msg)
{
    // From ROS messages to local objects.
    Frame c_frame;
    tf::Transform c_robot_pose;
    fromFrameMsgToFrameObject(frame_msg, c_frame);
    tf::poseMsgToTF(odom_msg->pose.pose, c_robot_pose);

    // If enable debugging gets images from disk.
    std::string c_frame_name;
    if (enable_debugging_)
    {
        c_frame_name = convertTo5Digits(frame_msg->header.seq);
        c_frame.setLeftImg(cv::imread(f_path_ + c_frame_name + "_left.png"));
        c_frame.setRightImg(cv::imread(f_path_ + c_frame_name + "_right.png"));
    }

    // Include a keyframe to the map if necessary.
    if (needNewKeyFrame(c_frame, c_robot_pose))
    {
        // Updates global variables.
        kf_counter_++;
        init_graph_ = true;
        l_kf_robot_pose_ = c_robot_pose;
        
        // Get frame visual information.
        ros::Time stamp = c_frame.getTimestamp();
        cv::Mat l_descs = c_frame.getLeftDescs();
        cv::Mat r_descs = c_frame.getRightDescs();
        std::vector<cv::Point2f> l_kpts = c_frame.getLeftKpts();
        std::vector<cv::Point2f> r_kpts = c_frame.getRightKpts();
        std::vector<cv::Point3f> obj_pts = c_frame.getObjectPoints();
        std::vector<cv::Point3f> cluster_centroids = c_frame.getClustersCentroids();
        std::vector<std::vector<int>> clusters = c_frame.getClusters();

        // Debug keyframe.
        if (enable_debugging_)
        {
            std::string c_keyframe_name = convertTo5Digits(kf_counter_);
            cv::imwrite(kf_path_ + c_keyframe_name + "_left.png", c_frame.getLeftImg());
            cv::imwrite(kf_path_ + c_keyframe_name + "_right.png", c_frame.getRightImg());
            if (clustering_img_pub_.getNumSubscribers() > 0)
                publishKeypointsClustering(c_frame.getLeftImg(), stamp, clustering_img_pub_, l_kpts, clusters);
            writeKeypointsClustering(c_frame.getLeftImg(), clusters_path_ + c_keyframe_name, l_kpts, clusters);
            ROS_INFO_STREAM("[GRAPH->Map includer]: Adding keyframe: " << kf_counter_);
        }

        // Save the frame timestamp
        frame_stamps_.push_back(stamp.toSec());
        
        // Loop of frame clusters.
        for (uint i = 0; i < clusters.size(); i++)
        {
            
        }
        
    }

    if (enable_debugging_)
    {
        std::filesystem::remove(f_path_ + c_frame_name + "_left.png");
        std::filesystem::remove(f_path_ + c_frame_name + "_right.png");
    }
}


bool Graph::needNewKeyFrame(Frame& c_frame, const tf::Transform& c_robot_pose)
{
    if (!init_graph_)
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
    else if (poseDiff2D(l_kf_robot_pose_, c_robot_pose) > min_dist_kf_)
    {
        return addFrameToMap(c_frame);
    }
    else
    {
        return false;
    }
}


bool Graph::addFrameToMap(Frame& c_frame)
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


void Graph::getRobot2Camera()
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