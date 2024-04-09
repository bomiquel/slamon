#include "visual_odometry.hpp"


void VisualOdometry::run()
{
    // Assign "uninitialized" status to visual odometer
    status_ = UNINITIALIZED;

    // Create ROS node
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Subscribers
    image_transport::SubscriberFilter l_img_sub, r_img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;

    // Publishers
    stereo_kps_pub_ = it.advertise("stereo_keypoints", 1);
    stereo_matches_pub_ = it.advertise("stereo_matches", 1);
    successive_matches_pub_ = it.advertise("successive_matches", 1);

    // Message sync
    boost::shared_ptr<Sync_> sync;
    l_img_sub.subscribe(it, "left_image_rect_color", 5);
    r_img_sub.subscribe(it, "right_image_rect_color", 5);
    l_info_sub.subscribe(nh, "left_camera_info", 5);
    r_info_sub.subscribe(nh, "right_camera_info", 5);
    sync.reset(new Sync_(SyncPolicy_(10), l_img_sub, r_img_sub, l_info_sub, r_info_sub));
    sync->registerCallback(boost::bind(&VisualOdometry::imageCallback, this, _1, _2, _3, _4));
    
    // 
    ros::spin();
}


void VisualOdometry::imageCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                                   const sensor_msgs::ImageConstPtr& r_img_msg,
                                   const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                   const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    start_time_ = ros::Time::now();

    // Create current frame object.
    Frame c_frame; 

    // Different actions depending on the state
    switch (status_)
    {
        case UNINITIALIZED:
            if(enable_debugging_)
                ROS_INFO_STREAM("UNINITIALIZED");
            initialization(l_img_msg, r_img_msg, l_info_msg, r_info_msg, c_frame);
            break;

        case INITIALIZING:
            if(enable_debugging_)
                ROS_INFO_STREAM("INITIALIZING");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = WORKING;
            break;

        case WORKING:
            if(enable_debugging_)
                ROS_INFO_STREAM("WORKING");
            if (!processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = LOST;
            computeMotionSuccessiveFrames(c_frame, p_frame_);
            break;

        case LOST:
            if(enable_debugging_)
                ROS_INFO_STREAM("LOST");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = WORKING;
            else
                status_ = LOST;
            break;
    }

    // Store current frame as previous.
    p_frame_ = c_frame;
    
    if (enable_debugging_)
    {
        ros::Duration delta_t = ros::Time::now() - start_time_;
        double delta_t_sec = delta_t.toSec();
        std::cout << "[SLAMON:] Delta t: " << delta_t_sec << std::endl;
    }
}


void VisualOdometry::initialization(const sensor_msgs::ImageConstPtr& l_img_msg,
                                    const sensor_msgs::ImageConstPtr& r_img_msg,
                                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                    const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                    Frame& c_frame)
{
    // Get the static transformation between robot frame (base link) and camera frame
    robot2camera_.setIdentity();
    try
    {
        tf_listener_.lookupTransform("/robot_0/base_link", l_img_msg->header.frame_id, ros::Time(0), robot2camera_);
    }
    catch (const tf::TransformException& e)
    {
        ROS_WARN("%s", e.what());
        return;
    }

    // Get the stereo camera model
    stereo_camera_model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Get the projection/camera matrix
    const cv::Mat P(3, 4, CV_64FC1, const_cast<double*>(l_info_msg->P.data()));
    camera_matrix_ = P.colRange(cv::Range(0, 3)).clone();

    // Are the images scaled?
    int binning_x = l_info_msg->binning_x;
    int binning_y = l_info_msg->binning_y;
    if (binning_x > 1 || binning_y > 1)
    {
      camera_matrix_.at<double>(0,0) = camera_matrix_.at<double>(0,0) / binning_x;
      camera_matrix_.at<double>(0,2) = camera_matrix_.at<double>(0,2) / binning_x;
      camera_matrix_.at<double>(0,3) = camera_matrix_.at<double>(0,3) / binning_x;
      camera_matrix_.at<double>(1,1) = camera_matrix_.at<double>(1,1) / binning_y;
      camera_matrix_.at<double>(1,2) = camera_matrix_.at<double>(1,2) / binning_y;
      camera_matrix_.at<double>(1,3) = camera_matrix_.at<double>(1,3) / binning_y;
    }

    // Process current frame
    if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
        status_ = WORKING;
    else
        status_ = INITIALIZING;
}


bool VisualOdometry::processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                         const sensor_msgs::ImageConstPtr& r_img_msg,
                                         Frame& c_frame)
{
    // From ROS image message to cv::Mat
    cv::Mat l_img, r_img;
    if(!imgMsgToMat(l_img_msg, r_img_msg, l_img, r_img)) // TODO: paralellization?
        return false; 

    // Detecting keypoints
    cv::Mat l_descs, r_descs;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    detectStereoKeypoints(l_img, r_img, l_descs, r_descs, l_kps, r_kps);

    // Keypoint detection debugging
    if (stereo_kps_pub_.getNumSubscribers() > 0)
        publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, l_kps, r_kps, stereo_kps_pub_);

    // Matching keypoints
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::KeyPoint> h_f_l_kps, h_f_r_kps;
    if(!matchStereoKeypoints(l_descs, r_descs, l_kps, r_kps, h_f_l_descs, h_f_r_descs, h_f_l_kps, h_f_r_kps))
        return false; 

    // Compute 3D keypoints. Far points are filtered out because they do not provide accurate scale
    // and translation information (Paz et al. 2008 and Mur-Artal et al. 2017). A point is considered "far" 
    // when its z is more than 40 times the stereo baseline.
    double disparity;
    cv::Point3d camera_point;
    cv::Point2d l_point, r_point;
    cv::Mat f_l_descs, f_r_descs;
    std::vector<cv::Point3f> camera_points;
    std::vector<cv::KeyPoint> f_l_kps, f_r_kps;
    double z_limit = stereo_camera_model_.baseline() * 40.0;
    for (uint i = 0; i < h_f_l_kps.size(); i++)
    {
        l_point = h_f_l_kps[i].pt;
        r_point = h_f_r_kps[i].pt;
        disparity = l_point.x - r_point.y;
        stereo_camera_model_.projectDisparityTo3d(l_point, disparity, camera_point);
        if (isfinite(camera_point.x) && 
            isfinite(camera_point.y) && 
            isfinite(camera_point.z) && 
            (camera_point.z > 0) && 
            (camera_point.z < z_limit)) // Rejects "far" points.
        {
            f_l_kps.push_back(h_f_l_kps[i]);
            f_r_kps.push_back(h_f_r_kps[i]);
            f_l_descs.push_back(h_f_l_descs.row(i));
            f_r_descs.push_back(h_f_r_descs.row(i));
            camera_points.push_back(camera_point);
        }
    }

    if (camera_points.size() < (uint)min_num_kps_)
    {
        ROS_WARN_STREAM("[3D keypoint computer:] There are not enough close 3D keypoints");
        return false;
    }

    // Stereo matching keypoints debugging
    if (stereo_matches_pub_.getNumSubscribers() > 0)
        publishMatches(false, l_img, r_img, l_img_msg->header.stamp, f_l_kps, f_r_kps, stereo_matches_pub_);

    // Fill Frame object
    c_frame = Frame(l_img, r_img, f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kps, f_r_kps, camera_points);

    return true;
}


bool VisualOdometry::computeMotionSuccessiveFrames(const Frame& p_frame,
                                                   const Frame& c_frame)
{
    // Search for matches of keypoints between successive frames or pairs of frames.
    std::vector<cv::Point3f> f_c_camera_points, f_p_camera_points;
    std::vector<cv::KeyPoint> f_c_l_kps, f_c_r_kps, f_p_l_kps, f_p_r_kps;
    if (!matchNonStereoKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), p_frame.getLeftDescs(), p_frame.getLeftDescs(),
                                 c_frame.getLeftKps(), c_frame.getRightKps(), p_frame.getLeftKps(), p_frame.getRightKps(),
                                 c_frame.getCameraPoints(), p_frame.getCameraPoints(), f_c_l_kps, f_c_r_kps,
                                 f_p_l_kps, f_p_r_kps, f_c_camera_points, f_p_camera_points))
        return false; 

    // Non stereo matching keypoints debugging
    if (successive_matches_pub_.getNumSubscribers() > 0)
    {
        if (matching_type_ == RATIO_MATCHING || matching_type_ == ESSENTIAL_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
            publishMatches(true, c_frame.getLeftImg(), p_frame.getLeftImg(), c_frame.getTimestamp(),
                        f_c_l_kps, f_p_l_kps, successive_matches_pub_);
        else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
            publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame.getLeftImg(), p_frame.getRightImg(),
                                   c_frame.getTimestamp(), f_c_l_kps, f_c_r_kps, f_p_l_kps, f_p_r_kps, successive_matches_pub_);
        else
            ROS_ERROR_STREAM("[Match debugger:] Unforeseen type of matching!");
    }

    return true;
}


bool VisualOdometry::imgMsgToMat(const sensor_msgs::ImageConstPtr& l_img_msg,
                                 const sensor_msgs::ImageConstPtr& r_img_msg,
                                 cv::Mat& l_img, 
                                 cv::Mat& r_img)
{
    // Convert message to cv::Mat
    cv_bridge::CvImagePtr l_img_ptr, r_img_ptr;
    try
    {
        // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. 
        // If SLAM uses only this cv_bridge::toCvCopy, we can copy
        // this function into the code and the warning will no longer appear.
        l_img_ptr = cv_bridge::toCvCopy(l_img_msg, sensor_msgs::image_encodings::BGR8);
        r_img_ptr = cv_bridge::toCvCopy(r_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("[SLAMON:] cv_bridge exception: %s", e.what());
        return false;
    }
    
    // Set the images
    l_img = l_img_ptr->image;
    r_img = r_img_ptr->image;
    return true;
}