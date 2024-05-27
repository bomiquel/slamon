#include "graph.hpp"


Graph::Graph(double ratio,
             int min_inliers,
             bool enable_debugging,
             const std::string& w_path,
             std::string& matching_type,
             std::string& motion_estimation_type):
             StereoModules(ratio, min_inliers, matching_type, motion_estimation_type)
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

    ROS_INFO_STREAM("--------------------------------------------------------------");
    ROS_INFO_STREAM("[frameCallback]:" << std::endl <<
                    "Left keypoints: " << c_frame.getLeftKpts().size() << std::endl <<
                    "Right keypoints: " << c_frame.getRightKpts().size() << std::endl <<
                    "Left descriptors: " << c_frame.getLeftDescs().rows << std::endl <<
                    "Right descriptors: " << c_frame.getRightDescs().rows << std::endl <<
                    "3D points: " << c_frame.getObjectPoints().size());
     ROS_INFO_STREAM("--------------------------------------------------------------");

    if (enable_debugging_)
    {
        std::string c_frame_name = convertTo5Digits(frame_msg->header.seq);
        std::string c_f_path = f_path_ + c_frame_name + "/";
        c_frame.setLeftImg(cv::imread(c_f_path + c_frame_name + "_left.png"));
        c_frame.setRightImg(cv::imread(c_f_path + c_frame_name + "_right.png"));
        std::string c_kf_path = kf_path_ + c_frame_name + "/";
        std::filesystem::create_directory(c_kf_path);
        cv::imwrite(c_kf_path + c_frame_name + "_left.png", c_frame.getLeftImg());
        cv::imwrite(c_kf_path + c_frame_name + "_right.png", c_frame.getRightImg());
        writeStereoKeypoints(c_frame.getLeftImg(), c_frame.getRightImg(), c_kf_path + "keypoints",
                             c_frame.getLeftKpts(), c_frame.getRightKpts());
        if (frame_msg->header.seq > 0)
        {
            std::vector<cv::Point3f> f_c_obj_pts, f_p_obj_pts;
            std::vector<cv::Point2f> f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts;
            StereoResult result = matchNonStereoKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), 
                                                          p_frame_.getLeftDescs(), p_frame_.getLeftDescs(),
                                                          c_frame.getLeftKpts(), c_frame.getRightKpts(), 
                                                          p_frame_.getLeftKpts(), p_frame_.getRightKpts(),
                                                          c_frame.getObjectPoints(), p_frame_.getObjectPoints(),
                                                          f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, 
                                                          f_c_obj_pts, f_p_obj_pts);
            writeMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_kf_path + "successive_matches",
                         f_c_l_kpts, f_p_l_kpts);
        }
    }

    p_frame_ = c_frame;

    
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

