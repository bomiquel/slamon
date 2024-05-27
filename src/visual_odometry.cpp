#include "visual_odometry.hpp"


VisualOdometry::VisualOdometry(double ratio,
                               int min_inliers,
                               bool enable_debugging,
                               const std::string& w_path,
                               std::string& matching_type,
                               std::string& motion_estimation_type,
                               const std::string& robot_frame_id, 
                               const std::string& origin_frame_id):
                               StereoModules(ratio, min_inliers, matching_type, motion_estimation_type)
{
    w_path_ = w_path;
    robot_frame_id_ = robot_frame_id;
    origin_frame_id_ = origin_frame_id;
    enable_debugging_ = enable_debugging;
}


void VisualOdometry::run()
{
    // Set "uninitialised" status to visual odometer.
    status_ = UNINITIALISED;

    // Set frame id counter.
    c_frame_id_ = 0;

    // Set false to initialised pose flag.
    initialised_pose_ = false;

    // Create debugging directories.
    if (enable_debugging_)
    {
        f_path_ = w_path_ + "frames/";
        if (std::filesystem::exists(f_path_))
            std::filesystem::remove_all(f_path_);
        std::filesystem::create_directory(f_path_);
    }

    // Create ROS node.
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    image_transport::ImageTransport it(nh);

    // Client services.
    initial_pose_client_ = nh.serviceClient<slamon::GetPose>("get_first_pose");

    // Publishers.
    info_pub_ = nhp.advertise<slamon::VOInfo>("vo_info", 1);
    odom_pub_ = nhp.advertise<nav_msgs::Odometry>("odom", 1);
    frame_pub_ = nhp.advertise<slamon::Frame>("frame", 1);
    inliers_img_pub_ = nhp.advertise<sensor_msgs::Image>("inliers_img", 1);
    stereo_kps_img_pub_ = nhp.advertise<sensor_msgs::Image>("stereo_keypoints_img", 1);
    stereo_matches_img_pub_ = nhp.advertise<sensor_msgs::Image>("stereo_matches_img", 1);
    successive_matches_img_pub_ = nhp.advertise<sensor_msgs::Image>("successive_matches_img", 1);

    // Subscribers.
    image_transport::SubscriberFilter l_img_sub, r_img_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;

    // Message sync.
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
    // Time at which the program enters into the callback.
    double start_time = ros::Time::now().toSec();

    // Create current frame object.
    Frame c_frame; 

    // Valid motion flag.
    bool valid_motion = false;

    // Current camera motion.
    tf::Transform c_camera_motion;

    // Different actions depending on the state.
    switch (status_)
    {
        case UNINITIALISED:
        {
            ROS_INFO_STREAM("UNINITIALIZED");
            if (!initialisation(l_info_msg, r_info_msg))
                break;
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
            {
                status_ = RUNNING;
            }
            else
            {
                status_ = SEMI_INITIALISED;
                run_times_.successive_matching = 0.0;
                run_times_.motion_estimation = 0.0;
            }
            break;
        }
        case SEMI_INITIALISED:
        {
            ROS_INFO_STREAM("SEMI-INITIALISED");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
            {
                status_ = RUNNING;
            }
            else
            {
                run_times_.successive_matching = 0.0;
                run_times_.motion_estimation = 0.0;
            }
            break;
        }
        case RUNNING:
        {
            ROS_INFO_STREAM("RUNNING");
            if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
            {
                valid_motion = computeMotionSuccessiveFrames(c_frame, c_camera_motion);
                if (valid_motion)
                {
                    std::string c_frame_name;
                    if (!initialised_pose_)
                    {
                        initialised_pose_ = getInitialPose();

                        if (enable_debugging_)
                        {
                            c_frame_name = convertTo5Digits(c_frame_id_);
                            cv::imwrite(f_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                            cv::imwrite(f_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                        }
                        publishFrame(p_frame_.getTimestamp(), p_frame_.getLeftDescs(), p_frame_.getRightDescs(),
                                     p_frame_.getLeftKpts(), p_frame_.getRightKpts(), p_frame_.getObjectPoints(),
                                     frame_pub_);
                        publishPoseAndVelocity(p_frame_.getTimestamp(), p_frame_.getTimestamp(), c_robot_pose_,
                                               tf::Transform(), odom_pub_, "map", robot_frame_id_);
                    }
                    c_frame_id_++;

                    // From {c2}^T_{c1} to {c1}^T_{c2}.
                    c_camera_motion = c_camera_motion.inverse();

                    // Changes the coordinate axis of the movement. From camera to robot. 
                    // {r1}^T_{r2} = {r1}^T_{c1} * {c1}^T_{c2} * {c2}^T_{r2}
                    tf::Transform c_robot_motion = robot2camera_ * c_camera_motion * robot2camera_.inverse();

                    // Calculates the robot's current pose by accumulating its movement.
                    // {o}^T_{r2} = {0}^T_{r1} * {r1}^T_{r2}
                    c_robot_pose_ *= c_robot_motion;

                    if (enable_debugging_)
                    {
                        c_frame_name = convertTo5Digits(c_frame_id_);
                        cv::imwrite(f_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                        cv::imwrite(f_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                    }
                    publishFrame(c_frame.getTimestamp(), c_frame.getLeftDescs(), c_frame.getRightDescs(),
                                 c_frame.getLeftKpts(), c_frame.getRightKpts(), c_frame.getObjectPoints(),
                                 frame_pub_);
                    publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), c_robot_pose_,
                                           c_robot_motion, odom_pub_, "map", robot_frame_id_);
                }
            }
            else
            {
                run_times_.successive_matching = 0.0;
                run_times_.motion_estimation = 0.0;
            }
            break;
        }
    }

    // Store current frame as previous.
    p_frame_ = c_frame;
    
    // Publishes last iteration performance
    run_times_.total = ros::Time::now().toSec() - start_time;
    publishInfo(l_img_msg->header.stamp, valid_motion, run_times_, info_pub_);
}


bool VisualOdometry::initialisation(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                    const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    // Gets the static transformation between robot frame (base link) and camera frame.
    robot2camera_.setIdentity();
    try
    {
        tf_listener_.lookupTransform(robot_frame_id_, l_info_msg->header.frame_id, ros::Time(0), robot2camera_);
    }
    catch (const tf::TransformException& e)
    {
        ROS_WARN_STREAM("[Initialiser:] TF listener exception: " << e.what());
        return false;
    }

    // Gets projection matrix
    l_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(l_info_msg->P.data())).clone();
    r_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(r_info_msg->P.data())).clone();

    // Checks if the projection matrices are filled with 0s
    double counter = 0.0;
    for (int i = 0; i < l_proj_mat_.rows; i++)
    {
        for (int j = 0; j < l_proj_mat_.cols; j++) 
            counter += l_proj_mat_.at<double>(i,j);
    }
    if (counter == 0.0)
    {
        ROS_WARN_STREAM("[Initialiser:] Left projection matrix is full of 0s");
        return false;
    }
    counter = 0.0; 
    for (int i = 0; i < r_proj_mat_.rows; i++)
    {
        for (int j = 0; j < r_proj_mat_.cols; j++)
            counter += r_proj_mat_.at<double>(i, j);
    }
    if (counter == 0.0)
    {
        ROS_WARN_STREAM("[Initialiser:] Right projection matrix is full of 0s");
        return false;
    }

    // Are the images scaled?
    int binning_x = l_info_msg->binning_x;
    int binning_y = l_info_msg->binning_y;
    if (binning_x > 1 || binning_y > 1)
    {
    l_proj_mat_.at<double>(0,0) = l_proj_mat_.at<double>(0,0) / binning_x;
    l_proj_mat_.at<double>(0,2) = l_proj_mat_.at<double>(0,2) / binning_x;
    l_proj_mat_.at<double>(0,3) = l_proj_mat_.at<double>(0,3) / binning_x;
    l_proj_mat_.at<double>(1,1) = l_proj_mat_.at<double>(1,1) / binning_y;
    l_proj_mat_.at<double>(1,2) = l_proj_mat_.at<double>(1,2) / binning_y;
    l_proj_mat_.at<double>(1,3) = l_proj_mat_.at<double>(1,3) / binning_y;

    r_proj_mat_.at<double>(0,0) = r_proj_mat_.at<double>(0,0) / binning_x;
    r_proj_mat_.at<double>(0,2) = r_proj_mat_.at<double>(0,2) / binning_x;
    r_proj_mat_.at<double>(0,3) = r_proj_mat_.at<double>(0,3) / binning_x;
    r_proj_mat_.at<double>(1,1) = r_proj_mat_.at<double>(1,1) / binning_y;
    r_proj_mat_.at<double>(1,2) = r_proj_mat_.at<double>(1,2) / binning_y;
    r_proj_mat_.at<double>(1,3) = r_proj_mat_.at<double>(1,3) / binning_y;
    }

    return true;
}


bool VisualOdometry::processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                         const sensor_msgs::ImageConstPtr& r_img_msg,
                                         Frame& c_frame)
{
    // Auxilary variable to set the start time of a process.
    double start_time;

    // From ROS image message to cv::Mat.
    cv::Mat l_img, r_img;
    if (!fromImgMsgToMat(l_img_msg, l_img)) // TODO: paralellization?
        return false; 
    if (!fromImgMsgToMat(r_img_msg, r_img)) // TODO: paralellization?
        return false; 

    // Detecting keypoints.
    start_time = ros::Time::now().toSec();
    cv::Mat l_descs, r_descs;
    std::vector<cv::Point2f> l_kpts, r_kpts;
    std::string feature_detector_type = "sift";
    detectStereoKeypoints(l_img, r_img, feature_detector_type, l_descs, r_descs, l_kpts, r_kpts);
    run_times_.feature_extraction = ros::Time::now().toSec() - start_time;

    // Keypoint detection debugging.
    if (stereo_kps_img_pub_.getNumSubscribers() > 0)
        publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, l_kpts, r_kpts, stereo_kps_img_pub_);

    // Matching keypoints.
    start_time = ros::Time::now().toSec();
    cv::Mat h_f_l_descs, h_f_r_descs;
    std::vector<cv::Point2f> h_f_l_kpts, h_f_r_kpts;
    StereoResult result = matchStereoKeypoints(l_descs, r_descs, l_kpts, r_kpts, h_f_l_descs, h_f_r_descs, h_f_l_kpts, h_f_r_kpts);
    run_times_.stereo_matching = ros::Time::now().toSec() - start_time;
    if(!decodeStereoResult("[Stereo matcher:] ", result))
    {
        run_times_.triangulation = 0;
        return false; 
    }
        
    // Compute 3D keypoints. 
    start_time = ros::Time::now().toSec();
    cv::Mat f_l_descs, f_r_descs;
    std::vector<cv::Point3f> f_obj_pts;
    std::vector<cv::Point2f> f_l_kpts, f_r_kpts;
    result = compute3DPoints(h_f_l_descs, h_f_r_descs, h_f_l_kpts, h_f_r_kpts, f_l_descs, f_r_descs, f_l_kpts, f_r_kpts, f_obj_pts);
    run_times_.stereo_matching = ros::Time::now().toSec() - start_time;
    if(!decodeStereoResult("[Triangulator:] ", result))
        return false; 

    // Stereo matching keypoints debugging.
    if (stereo_matches_img_pub_.getNumSubscribers() > 0)
        publishMatches(false, l_img, r_img, l_img_msg->header.stamp, f_l_kpts, f_r_kpts, stereo_matches_img_pub_);

    // Fill Frame object
    c_frame = Frame(l_img, r_img, f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kpts, f_r_kpts, f_obj_pts);

    return true;
}


bool VisualOdometry::computeMotionSuccessiveFrames(const Frame& c_frame,
                                                   tf::Transform& c_camera_motion)
{
    // Auxilary variable to set the start time of a process
    double start_time;

    // Search for matches of keypoints between successive frames or pairs of frames.
    start_time = ros::Time::now().toSec();
    std::vector<cv::Point3f> f_c_obj_pts, f_p_obj_pts;
    std::vector<cv::Point2f> f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts;
    StereoResult result = matchNonStereoKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), 
                                                  p_frame_.getLeftDescs(), p_frame_.getLeftDescs(),
                                                  c_frame.getLeftKpts(), c_frame.getRightKpts(), 
                                                  p_frame_.getLeftKpts(), p_frame_.getRightKpts(),
                                                  c_frame.getObjectPoints(), p_frame_.getObjectPoints(),
                                                  f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, 
                                                  f_c_obj_pts, f_p_obj_pts);
    run_times_.successive_matching = ros::Time::now().toSec() - start_time;
    if(!decodeStereoResult("[Non-stereo matcher:] ", result))
    {
        run_times_.motion_estimation = 0.0;
        return false;
    }

    // Non stereo matching keypoints debugging.
    if (successive_matches_img_pub_.getNumSubscribers() > 0)
    {
        if (matching_type_ == RATIO_MATCHING || matching_type_ == ESSENTIAL_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
            publishMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                           f_c_l_kpts, f_p_l_kpts, successive_matches_img_pub_);

        else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
            publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                   c_frame.getTimestamp(), f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, successive_matches_img_pub_);

        else
            ROS_ERROR_STREAM("[Non-stereo matcher debugger:] Unforeseen type of matching!");
    }

    // Estimate the motion.
    start_time = ros::Time::now().toSec();
    cv::Mat r_camera_motion, t_camera_motion;
    std::vector<cv::Point3f> i_c_obj_pts, i_p_obj_pts;
    std::vector<cv::Point2f> i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts; 
    result = estimateMotion(f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_obj_pts, f_p_obj_pts,
                            r_camera_motion, t_camera_motion,
                            i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts, i_c_obj_pts, i_p_obj_pts);
    run_times_.motion_estimation = ros::Time::now().toSec() - start_time;
    if(!decodeStereoResult("[Motion estimator:] ", result))
        return false; 

    // Inliers debugging.
    if (inliers_img_pub_.getNumSubscribers() > 0)
    {
        if (matching_type_ == RATIO_MATCHING || matching_type_ == ESSENTIAL_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
            publishMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                           i_c_l_kpts, i_p_l_kpts, inliers_img_pub_);

        else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
            publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                   c_frame.getTimestamp(), i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts, inliers_img_pub_);

        else
            ROS_ERROR_STREAM("[Motion estimator debugger:] Unforeseen type of matching!");
    }

    // Build a TF
    if(!fromMatToTf(r_camera_motion, t_camera_motion, c_camera_motion))
        return false;

    return true;
}


bool VisualOdometry::getInitialPose()
{
    slamon::GetPose srv;
    if (initial_pose_client_.call(srv))
    {
        double tx = srv.response.pose.pose.pose.position.x;
        double ty = srv.response.pose.pose.pose.position.y;
        double tz = srv.response.pose.pose.pose.position.z;
        double qx = srv.response.pose.pose.pose.orientation.x;
        double qy = srv.response.pose.pose.pose.orientation.y;
        double qz = srv.response.pose.pose.pose.orientation.z;
        double qw = srv.response.pose.pose.pose.orientation.w;

        ROS_WARN_STREAM("[Initial pose:] GetPose service call successful. The initial position is as follows:" << std::endl <<
                        "tx: " << tx << std::endl << 
                        "ty: " << ty << std::endl << 
                        "tz: " << tz << std::endl << 
                        "qx: " << qx << std::endl << 
                        "qy: " << qy << std::endl << 
                        "qz: " << qz << std::endl << 
                        "qw: " << qw);

        tf::Vector3 tf_t(tx, ty, tz);
        tf::Quaternion tf_q(qx, qy, qz, qw);
        c_robot_pose_.setOrigin(tf_t);
        c_robot_pose_.setRotation(tf_q);
    }
    else
    {
        ROS_WARN_STREAM("[Initial pose:] Failed to call GetPose service. The initial pose will be assumed as the identity.");
        c_robot_pose_.setIdentity();
    }

    return true;
}


bool VisualOdometry::decodeStereoResult(const std::string& title,
                                        const StereoResult& result)
{
    bool successful; 
    switch (result)
    {
        case SUCCESSFUL:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "Successful matching!");
            successful = true;
            break;
        }
        case NOT_ENOUGH_KPTS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough keypoints to match and filter!");
            successful = false;
            break;
        }
        case NOT_ENOUGH_F_KPTS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough keypoints after outlier filtering!");
            successful = false;
            break;
        }
        case NOT_ENOUGH_E_KPTS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough keypoints after epipolar filtering!");
            successful = false;
            break;
        }
        case NOT_ENOUGH_3D_PTS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough 3D points after triangulation!");
            successful = false;
            break;
        }
        case GEOMETRIC_PROBLEM:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "Problems in applying the selected geometric constraint. Probably not enough keypoints!");
            successful = false;
            break;
        }
        case DIFFERENT_PROBLEMS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "In the parallel matches there have been different problems.");
            successful = false;
            break;
        }
        case NOT_ENOUGH_CIRCLES:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough matching circles!");
            successful = false;
            break;
        }
        case NOT_ENOUGH_INLIERS:
        {
            if (enable_debugging_)
                ROS_INFO_STREAM(title << "There are not enough inliers after motion estimation!");
            successful = false;
            break;
        }
        default:
        {
            ROS_ERROR_STREAM(title << "Unforeseen result of matching!");
            successful = false;
            break;
        }
    }
    return successful;
}   