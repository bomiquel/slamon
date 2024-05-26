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

    ROS_INFO_STREAM("--------------------------------------------------------------");
    ROS_INFO_STREAM("[frameCallback]:" << std::endl <<
                    "Left keypoints: " << c_frame.getLeftKpts().size() << std::endl <<
                    "Right keypoints: " << c_frame.getRightKpts().size() << std::endl <<
                    "Left descriptors: " << c_frame.getLeftDescs().rows << std::endl <<
                    "Right descriptors: " << c_frame.getRightDescs().rows << std::endl <<
                    "3D points: " << c_frame.getObjectPoints().size());
     ROS_INFO_STREAM("--------------------------------------------------------------");
    
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

