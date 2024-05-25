#include "graph.hpp"


namespace slam
{
    Graph::Graph(const bool enable_debugging) 
    {
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
        frame_sub.subscribe(nh, "frame", 1);
        odom_sub.subscribe(nh, "odom", 1);
        sync.reset(new Sync_(SyncPolicy_(10), frame_sub, odom_sub));
        sync->registerCallback(boost::bind(&Graph::frameCallback, this, _1, _2));

        //
        ros::spin();
    }


    void Graph::frameCallback(const slamon::FrameConstPtr& frame_msg,
                              const nav_msgs::OdometryConstPtr& odom_msg)
    {
        ROS_INFO_STREAM("I Will Do What I Must");
    }
}

