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
    pub_stereo_keypoints_ = it.advertise("stereo_keypoints", 1);

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
    // From image ROS message to cv::Mat
    cv::Mat l_img, r_img;
    if(!imgMsgToMat(l_img_msg, r_img_msg, l_img, r_img))// TODO: paralellization?
        return; 

    // Keypoint detection
    cv::Mat l_descs, r_descs;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    detectStereoKeypoints(l_img, r_img, l_descs, r_descs, l_kps, r_kps);
    // Keypoint detection debugging
    cv::Mat stereo;
    cv::Scalar color(0.0, 0.0, 255);
    std::vector<cv::Point2f> c_l_kps, c_r_kps;
    cv::KeyPoint::convert(l_kps, c_l_kps, std::vector<int>());
    cv::KeyPoint::convert(r_kps, c_r_kps, std::vector<int>());
    cv::Mat l_img_drawn = drawPoints(3, l_img, color, c_l_kps);
    cv::Mat r_img_drawn = drawPoints(3, r_img, color, c_r_kps);
    l_img_drawn.copyTo(stereo);
    cv::hconcat(stereo, r_img_drawn, stereo);
    publishImage(pub_stereo_keypoints_, stereo, l_img_msg->header.stamp);
}

