#include "graph.hpp"


Graph::Graph(bool enable_debugging,
             const std::string& w_path)
{
    w_path_ = w_path;
    init_graph_ = false;
    enable_debugging_ = enable_debugging;
}


void Graph::run()
{
    // Create debugging directories.
    if (enable_debugging_)
    {
        f_path_ = w_path_ + "frames/";
        kf_path_ = w_path_ + "keyframes/";
        if (std::filesystem::exists(kf_path_))
            std::filesystem::remove_all(kf_path_);
        std::filesystem::create_directory(kf_path_);
    }

    // Create ROS node.
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

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
    
    // if (!init_graph_)
    // {

    // } 
    // else if (needNewKeyFrame())
    // {

    // }
    // else
    // {

    // }
}

