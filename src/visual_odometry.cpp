#include "visual_odometry.hpp"


void VisualOdometry::run()
{
    // Create node
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    image_transport::ImageTransport it(nh);

    // Subscribers
    image_transport::SubscriberFilter l_img_sub, r_img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;

    // Publishers
    stereo_kps_pub_ = it.advertise("stereo_keypoints", 1);
    stereo_matches_pub_ = it.advertise("stereo_matches", 1);

    // Message sync
    boost::shared_ptr<Sync_> sync;
    l_img_sub.subscribe(it, "left_image_rect_color", 5);
    r_img_sub.subscribe(it, "right_image_rect_color", 5);
    l_info_sub.subscribe(nh, "left_camera_info", 5);
    r_info_sub.subscribe(nh, "right_camera_info", 5);
    sync.reset(new Sync_(SyncPolicy_(10), l_img_sub, r_img_sub, l_info_sub, r_info_sub));
    sync->registerCallback(boost::bind(&VisualOdometry::imageCallback, this, _1, _2, _3, _4));
    
    ros::spin();
}


void VisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                                   const sensor_msgs::ImageConstPtr& r_img_msg,
                                   const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                   const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    // From ROS image message to cv::Mat
    cv::Mat l_img, r_img;
    if(!imgMsgToMat(l_img_msg, r_img_msg, l_img, r_img))// TODO: paralellization?
        return; 

    // Detecting keypoints
    cv::Mat l_descs, r_descs;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    detectStereoKeypoints(l_img, r_img, l_descs, r_descs, l_kps, r_kps);

    // Keypoint detection debugging
    if (stereo_kps_pub_.getNumSubscribers() > 0)
        publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, l_kps, r_kps, stereo_kps_pub_);

    // Matching keypoints
    double ratio = 0.8;
    cv::Mat f_l_descs, f_r_descs;
    std::vector<cv::DMatch> f_matches;
    std::vector<cv::KeyPoint> f_l_kps, f_r_kps;
    if(!matchStereoKeypoints(ratio, l_descs, r_descs, l_kps, r_kps, f_l_descs, f_r_descs, f_matches, f_l_kps, f_r_kps))
        return;

    // Matching keypoints debugging
    if (stereo_matches_pub_.getNumSubscribers() > 0)
        publishStereoMatches(l_img, r_img, l_img_msg->header.stamp, f_matches, l_kps, r_kps, stereo_matches_pub_);
}

