#include "visual_odometry.hpp"


void VisualOdometry::run()
{
    // Assign "uninitialized" status to visual odometer
    status_ = UNINITIALIZED;

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
    Frame c_frame; 
    start_time_ = ros::Time::now();

    switch (status_)
    {
        case UNINITIALIZED:
            ROS_INFO_STREAM("UNINITIALIZED");
            initialization(l_img_msg, r_img_msg, l_info_msg, r_info_msg, c_frame);
            std::cout << "Frame l_kps: " << c_frame.getLeftKp().size() << std::endl;
            std::cout << "Frame r_descs: " << c_frame.getRightDesc().rows << std::endl;
            std::cout << "Frame matches: " << c_frame.getMatches().size() << std::endl;
            std::cout << "Frame 3D points: " << c_frame.getCameraPoints().size() << std::endl;
            break;

        case INITIALIZING:
            ROS_INFO_STREAM("INITIALIZING");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = WORKING;
            std::cout << "Frame l_kps: " << c_frame.getLeftKp().size() << std::endl;
            std::cout << "Frame r_descs: " << c_frame.getRightDesc().rows << std::endl;
            std::cout << "Frame matches: " << c_frame.getMatches().size() << std::endl;
            std::cout << "Frame 3D points: " << c_frame.getCameraPoints().size() << std::endl;
            break;

        case WORKING:
            ROS_INFO_STREAM("WORKING");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = WORKING;
            else
                status_ = LOST;
            std::cout << "Frame l_kps: " << c_frame.getLeftKp().size() << std::endl;
            std::cout << "Frame r_descs: " << c_frame.getRightDesc().rows << std::endl;
            std::cout << "Frame matches: " << c_frame.getMatches().size() << std::endl;
            std::cout << "Frame 3D points: " << c_frame.getCameraPoints().size() << std::endl;
            break;

        case LOST:
            ROS_INFO_STREAM("LOST");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                status_ = WORKING;
            else
                status_ = LOST;
            std::cout << "Frame l_kps: " << c_frame.getLeftKp().size() << std::endl;
            std::cout << "Frame r_descs: " << c_frame.getRightDesc().rows << std::endl;
            std::cout << "Frame matches: " << c_frame.getMatches().size() << std::endl;
            std::cout << "Frame 3D points: " << c_frame.getCameraPoints().size() << std::endl;
            break;
    }
    
    ros::Duration delta_t = ros::Time::now() - start_time_;
    double delta_t_sec = delta_t.toSec();
    std::cout << "[SLAMON:] Delta t: " << delta_t_sec << std::endl;
}


void VisualOdometry::initialization(const sensor_msgs::ImageConstPtr& l_img_msg,
                                    const sensor_msgs::ImageConstPtr& r_img_msg,
                                    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                    const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                    Frame& c_frame)
{
    // Get the static transformation between robot frame (base link) and camera frame
    if (!getRobot2CameraTf("/robot_0/base_link", l_img_msg->header.frame_id))
    {
        ROS_WARN("[SLAMON:] Impossible to obtain transform between robot frame (base link) and camera frame. Waiting.");
        return;
    }

    // Get camera model and matrix
    getCameraModelMatrix(l_info_msg, r_info_msg);

    // Process current frame
    if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
        status_ = WORKING;
    else
        status_ = INITIALIZING;
}


bool VisualOdometry::getRobot2CameraTf(std::string robot_frame_id,
                                       std::string sensor_frame_id)
{
    // Init the transform
    robot2camera_.setIdentity();

    // Try to extract the transform
    try
    {
        tf_listener_.lookupTransform(robot_frame_id, sensor_frame_id, ros::Time(0), robot2camera_);
    }
    catch(const tf::TransformException& e)
    {
        ROS_WARN("%s", e.what());
        return false;
    }
    return true;    
}


void VisualOdometry::getCameraModelMatrix(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                          const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    // Get the binning factors
    int binning_x = l_info_msg->binning_x;
    int binning_y = l_info_msg->binning_y;

    // Get the stereo camera model
    camera_model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Get the projection/camera matrix
    const cv::Mat P(3, 4, CV_64FC1, const_cast<double*>(l_info_msg->P.data()));
    camera_matrix_ = P.colRange(cv::Range(0, 3)).clone();

    // Are the images scaled?
    if (binning_x > 1 || binning_y > 1)
    {
      camera_matrix_.at<double>(0,0) = camera_matrix_.at<double>(0,0) / binning_x;
      camera_matrix_.at<double>(0,2) = camera_matrix_.at<double>(0,2) / binning_x;
      camera_matrix_.at<double>(0,3) = camera_matrix_.at<double>(0,3) / binning_x;
      camera_matrix_.at<double>(1,1) = camera_matrix_.at<double>(1,1) / binning_y;
      camera_matrix_.at<double>(1,2) = camera_matrix_.at<double>(1,2) / binning_y;
      camera_matrix_.at<double>(1,3) = camera_matrix_.at<double>(1,3) / binning_y;
    }
}


bool VisualOdometry::processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                         const sensor_msgs::ImageConstPtr& r_img_msg,
                                         Frame& c_frame)
{
    // From ROS image message to cv::Mat
    cv::Mat l_img, r_img;
    if(!imgMsgToMat(l_img_msg, r_img_msg, l_img, r_img))// TODO: paralellization?
    {
        ros::Duration delta_t = ros::Time::now() - start_time_;
        double delta_t_sec = delta_t.toSec();
        std::cout << "[SLAMON:] Delta t: " << delta_t_sec << std::endl;
        return false; 
    }

    // Detecting keypoints
    cv::Mat l_descs, r_descs;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    detectStereoKeypoints(l_img, r_img, l_descs, r_descs, l_kps, r_kps);

    // Keypoint detection debugging
    if (stereo_kps_pub_.getNumSubscribers() > 0)
        publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, l_kps, r_kps, stereo_kps_pub_);

    // Matching keypoints
    double ratio = 0.8;
    cv::Mat half_f_l_descs, half_f_r_descs;
    std::vector<cv::DMatch> half_f_matches;
    std::vector<cv::KeyPoint> half_f_l_kps, half_f_r_kps;
    if(!matchStereoKeypoints(ratio, l_descs, r_descs, l_kps, r_kps, half_f_l_descs, half_f_r_descs, half_f_matches, half_f_l_kps, half_f_r_kps))
    {
        ros::Duration delta_t = ros::Time::now() - start_time_;
        double delta_t_sec = delta_t.toSec();
        std::cout << "[SLAMON:] Delta t: " << delta_t_sec << std::endl;
        return false; 
    }

    // Compute 3D keypoints
    cv::Mat f_l_descs, f_r_descs;
    std::vector<cv::DMatch> f_matches;
    std::vector<cv::KeyPoint> f_l_kps, f_r_kps;
    std::vector<cv::Point3f> camera_points = compute3DPoints(half_f_l_descs, half_f_r_descs, half_f_matches, half_f_l_kps, half_f_r_kps, 
                                                             f_l_descs, f_r_descs, f_matches, f_l_kps, f_r_kps);

    // Matching keypoints debugging
    if (stereo_matches_pub_.getNumSubscribers() > 0)
        publishStereoMatches(l_img, r_img, l_img_msg->header.stamp, f_matches, l_kps, r_kps, stereo_matches_pub_);

    std::cout << "l_kps: " << f_l_kps.size() << std::endl;
    std::cout << "r_descs: " << f_r_descs.rows << std::endl;
    std::cout << "matches: " << f_matches.size() << std::endl;
    std::cout << "3D points: " << camera_points.size() << std::endl;

    // Fill Frame object
    c_frame = Frame(f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kps, f_r_kps, f_matches, camera_points);

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
        // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. If SLAM uses only this cv_bridge::toCvCopy, we can copy this function into the code and the warning will no longer appear.
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


std::vector<cv::Point3f> VisualOdometry::compute3DPoints(const cv::Mat& l_descs,
                                                         const cv::Mat& r_descs,
                                                         const std::vector<cv::DMatch>& matches,
                                                         const std::vector<cv::KeyPoint>& l_kps,
                                                         const std::vector<cv::KeyPoint>& r_kps,
                                                         cv::Mat& f_l_descs,
                                                         cv::Mat& f_r_descs,
                                                         std::vector<cv::DMatch>& f_matches,
                                                         std::vector<cv::KeyPoint>& f_l_kps,
                                                         std::vector<cv::KeyPoint>& f_r_kps)
{
    double disparity;
    cv::Point3d camera_point;
    cv::Point2d l_point, r_point;
    std::vector<cv::Point3f> camera_points;
    for (uint i = 0; i < l_kps.size(); i++)
    {
        l_point = l_kps[i].pt;
        r_point = r_kps[i].pt;
        disparity = l_point.x - r_point.y;
        camera_model_.projectDisparityTo3d(l_point, disparity, camera_point);
        if (isfinite(camera_point.x) && isfinite(camera_point.y) && isfinite(camera_point.z) && (camera_point.z) > 0)
        {
            f_l_kps.push_back(l_kps[i]);
            f_r_kps.push_back(r_kps[i]);
            f_matches.push_back(matches[i]);
            f_l_descs.push_back(l_descs.row(i));
            f_r_descs.push_back(r_descs.row(i));
            camera_points.push_back(camera_point);
        }
    }
    return camera_points;
}