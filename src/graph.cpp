#include "graph.hpp"


Graph::Graph(int min_lc_inliers,
             double min_dist_kf,
             bool enable_debugging,
             const std::string& w_path)
{
    w_path_ = w_path;
    kf_counter_ = 0;
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
    Frame c_frame;
    tf::Transform c_robot_pose;
    fromFrameMsgToFrameObject(frame_msg, c_frame);
    tf::poseMsgToTF(odom_msg->pose.pose, c_robot_pose);

    if (enable_debugging_)
    {
        std::string c_frame_name = convertTo5Digits(frame_msg->header.seq);
        c_frame.setLeftImg(cv::imread(f_path_ + c_frame_name + "_left.png"));
        c_frame.setRightImg(cv::imread(f_path_ + c_frame_name + "_right.png"));
    }

    if (needNewKeyFrame(c_frame, c_robot_pose))
    {
        // Updates global variables.
        init_graph_ = true;
        l_kf_robot_pose_ = c_robot_pose;

        // Converting frame to keyframe
        c_frame.setId(kf_counter_);

        if (enable_debugging_)
        {
            std::string c_keyframe_name = convertTo5Digits(c_frame.getId());
            cv::imwrite(kf_path_ + c_keyframe_name + "_left.png", c_frame.getLeftImg());
            cv::imwrite(kf_path_ + c_keyframe_name + "_right.png", c_frame.getRightImg());
            if (clustering_img_pub_.getNumSubscribers() > 0)
                publishKeypointsClustering(c_frame.getLeftImg(), c_frame.getTimestamp(), clustering_img_pub_,
                                           c_frame.getLeftKpts(), c_frame.getClusters());
            writeKeypointsClustering(c_frame.getLeftImg(), clusters_path_ + c_keyframe_name,
                                     c_frame.getLeftKpts(), c_frame.getClusters());
            ROS_INFO_STREAM("[GRAPH->addFrameToMap]: Adding keyframe: " << kf_counter_);
        }
        kf_counter_++;
    }
}


bool Graph::needNewKeyFrame(Frame& c_frame, const tf::Transform& c_robot_pose)
{
    if (!init_graph_)
        return addFrameToMap(c_frame);
    else if (poseDiff2D(l_kf_robot_pose_, c_robot_pose) > min_dist_kf_)
        return addFrameToMap(c_frame);
    else 
        return false;
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
            ROS_WARN_STREAM("[GRAPH->addFrameToMap]: Image produces 0 clusters.");
            return false;
        }
    }
    else
    {
        ROS_WARN_STREAM("[GRAPH->addFrameToMap]: Not enough keypoints in this frame (" << c_frame.getLeftKpts().size() << ")");
        return false;
    }
}