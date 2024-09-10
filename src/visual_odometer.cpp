#include "visual_odometer.hpp"


namespace slamon
{


    VisualOdometer::VisualOdometer(double ratio,
                                   int min_inliers,
                                   int min_lc_inliers,
                                   int max_kpts_bucket,
                                   bool enable_debugging,
                                   bool enable_bucketing,
                                   bool enable_enhacement,
                                   const std::string& w_path,
                                   std::string& matching_type,
                                   std::string& robot_frame_id,
                                   std::string& motion_estimation_type):
                                   VisionMotionModules(ratio, matching_type, motion_estimation_type)
    {
        // Inilitialises parameters.
        w_path_ = w_path;
        min_inliers_ = min_inliers;
        min_lc_inliers_ = min_lc_inliers;
        robot_frame_id_ = robot_frame_id;
        max_kpts_bucket_ = max_kpts_bucket;
        enable_debugging_ = enable_debugging;
        enable_bucketing_ = enable_bucketing;
        enable_enhacement_ = enable_enhacement;

        // Set "uninitialised" status to visual odometer.
        status_ = UNINITIALISED;

        // Set frame id counter.
        c_frame_id_ = 0;

        // Set false to initialised odometry flag.
        initialised_odom_ = false;

        // Create debugging directories.
        if (enable_debugging_)
        {
            frame_path_ = w_path_ + "frames/";
            if (std::filesystem::exists(frame_path_))
                std::filesystem::remove_all(frame_path_);
            std::filesystem::create_directory(frame_path_);
        }
    }


    void VisualOdometer::run()
    {
        // Create ROS node.
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        image_transport::ImageTransport it(nh);

        // Client services.
        initial_odom_client_ = nhp.serviceClient<slamon::GetTransform>("get_first_odom");

        // Server servives.
        robot2camera_server_ = nhp.advertiseService("send_robot_to_camera_tf", &VisualOdometer::sendRobot2Camera, this);
        proj_mat_server_ = nhp.advertiseService("send_projection_matrices", &VisualOdometer::sendProjectionMatrices, this);

        // Publishers.
        stereo_data_pub_ = nhp.advertise<slamon::StereoData>("stereo_data", 1);
        run_times_pub_ = nhp.advertise<slamon::Performance>("vo_performance", 1);
        robot_odom_pub_ = nhp.advertise<nav_msgs::Odometry>("robot_odom", 1);
        camera_odom_pub_ = nhp.advertise<nav_msgs::Odometry>("camera_odom", 1);
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
        sync->registerCallback(boost::bind(&VisualOdometer::callback, this, _1, _2, _3, _4));
        
        // Calls callback when synchronised messages arrives.
        ros::spin();
    }


    void VisualOdometer::callback(const sensor_msgs::ImageConstPtr& l_img_msg,
                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg)
    {
        // Time at which the program enters into the callback.
        double start_callback_time = ros::Time::now().toSec();

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
                if (enable_debugging_)
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
                }
                run_times_.run_time_3 = 0.0; // Succesive matching run time
                run_times_.run_time_4 = 0.0; // Bucketing run time
                run_times_.run_time_5 = 0.0; // Motion estimation run time
                break;
            }
            case SEMI_INITIALISED:
            {
                if (enable_debugging_)
                    ROS_INFO_STREAM("SEMI-INITIALISED");
                if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                {
                    status_ = RUNNING;
                }
                run_times_.run_time_3 = 0.0; // Succesive matching run time
                run_times_.run_time_4 = 0.0; // Bucketing run time
                run_times_.run_time_5 = 0.0; // Motion estimation run time
                break;
            }
            case RUNNING:
            {
                if (enable_debugging_)
                    ROS_INFO_STREAM("RUNNING");
                if (processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                {
                    valid_motion = computeMotionSuccessiveFrames(c_frame, c_camera_motion);
                    if (valid_motion)
                    {
                        std::string c_frame_name;
                        if (!initialised_odom_)
                        {
                            getInitialOdom();
                            initialised_odom_ = true;

                            if (enable_debugging_)
                            {
                                c_frame_name = convertTo5Digits(c_frame_id_);
                                cv::imwrite(frame_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                                cv::imwrite(frame_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                            }
                            publishStereoData(p_frame_.getLeftDescs(), p_frame_.getRightDescs(), p_frame_.getTimestamp(), 
                                              stereo_data_pub_, p_frame_.getLeftKpts(), p_frame_.getRightKpts(), p_frame_.getCameraPoints());
                            publishPoseAndVelocity(p_frame_.getTimestamp(), p_frame_.getTimestamp(), c_camera_odom_, tf::Transform(),
                                                   camera_odom_pub_, camera_frame_id_);
                            publishPoseAndVelocity(p_frame_.getTimestamp(), p_frame_.getTimestamp(), c_robot_odom_, tf::Transform(),
                                                   robot_odom_pub_, robot_frame_id_);                            
                        }
                        c_frame_id_++;

                        // From {c_cam}^T_{p_cam} to {p_cam}^T_{c_cam}.
                        c_camera_motion = c_camera_motion.inverse();

                        // Calculates the camera's current odometry by accumulating its movement.
                        // {o}^T_{c_cam} = {o}^T_{p_cam} * {p_cam}^T_{c_cam}
                        c_camera_odom_ *= c_camera_motion;

                        // Changes the coordinate axis of the movement. From camera to robot. 
                        // {p_robot}^T_{c_robot} = {robot}^T_{cam} * {p_cam}^T_{c_cam} * {cam}^T_{robot}
                        tf::Transform c_robot_motion = robot2camera_ * c_camera_motion * robot2camera_.inverse();

                        // Calculates the robot's current odometry by accumulating its movement.
                        // {o}^T_{c_robot} = {o}^T_{p_robot} * {p_robot}^T_{c_robot}
                        c_robot_odom_ *= c_robot_motion;

                        if (enable_debugging_)
                        {
                            c_frame_name = convertTo5Digits(c_frame_id_);
                            cv::imwrite(frame_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                            cv::imwrite(frame_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                        }
                        publishStereoData(c_frame.getLeftDescs(), c_frame.getRightDescs(), c_frame.getTimestamp(),
                                          stereo_data_pub_, c_frame.getLeftKpts(), c_frame.getRightKpts(), c_frame.getCameraPoints());
                        publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), c_camera_odom_, c_camera_motion,
                                               camera_odom_pub_, camera_frame_id_);
                        publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), c_robot_odom_, c_robot_motion,
                                               robot_odom_pub_, robot_frame_id_);
                    }
                }
                else
                {
                    run_times_.run_time_3 = 0.0; // Succesive matching run time
                    run_times_.run_time_4 = 0.0; // Bucketing run time
                    run_times_.run_time_5 = 0.0; // Motion estimation run time
                }
                break;
            }
        }

        // Store current frame as previous.
        p_frame_ = c_frame;
        
        // Publishes last iteration performance.
        run_times_.run_time_5 = 0.0; // Nothing.
        run_times_.total_run_time = ros::Time::now().toSec() - start_callback_time;
        if (run_times_pub_.getNumSubscribers() > 0)
            publishPerformance(l_img_msg->header.stamp, (int)valid_motion, run_times_, run_times_pub_);
    }


    bool VisualOdometer::sendRobot2Camera(slamon::GetTransform::Request& req,
                                          slamon::GetTransform::Response& res)
    {
        res.tf.header.frame_id = camera_frame_id_;
        res.tf.pose.position.x = robot2camera_.getOrigin().getX();
        res.tf.pose.position.y = robot2camera_.getOrigin().getY();
        res.tf.pose.position.z = robot2camera_.getOrigin().getZ();
        res.tf.pose.orientation.x = robot2camera_.getRotation().getX();
        res.tf.pose.orientation.y = robot2camera_.getRotation().getY();
        res.tf.pose.orientation.z = robot2camera_.getRotation().getZ();
        res.tf.pose.orientation.w = robot2camera_.getRotation().getW();
        return true;
    }


    bool VisualOdometer::sendProjectionMatrices(slamon::GetProjectionMatrices::Request& req,
                                                slamon::GetProjectionMatrices::Response& res)
    {
        for (int row = 0; row < l_proj_mat_.rows; row++)
        {
            for (int col = 0; col < l_proj_mat_.cols; col++)
            {
                res.l_proj_mat[l_proj_mat_.cols * row + col] = l_proj_mat_.at<double>(row, col);
                res.r_proj_mat[r_proj_mat_.cols * row + col] = r_proj_mat_.at<double>(row, col);
            }
        }
        return true;
    }


    bool VisualOdometer::initialisation(const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                        const sensor_msgs::CameraInfoConstPtr& r_info_msg)
    {
        // Gets the static transformation between robot frame (base link) and camera frame.
        robot2camera_.setIdentity();
        try
        {
            camera_frame_id_ = l_info_msg->header.frame_id;
            tf_listener_.lookupTransform(robot_frame_id_, camera_frame_id_, ros::Time(0), robot2camera_);
        }
        catch (const tf::TransformException& e)
        {
            ROS_WARN_STREAM("[VO->Initialiser:] TF listener exception: " << e.what());
            return false;
        }

        // Gets projection matrix.
        l_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(l_info_msg->P.data())).clone();
        r_proj_mat_ = cv::Mat(3, 4, CV_64FC1, const_cast<double*>(r_info_msg->P.data())).clone();

        // Checks if the projection matrices are filled with 0s.
        double counter = 0.0;
        for (int i = 0; i < l_proj_mat_.rows; i++)
        {
            for (int j = 0; j < l_proj_mat_.cols; j++) 
                counter += l_proj_mat_.at<double>(i,j);
        }
        if (counter == 0.0)
        {
            ROS_WARN_STREAM("[VO->Initialiser:] Left projection matrix is full of 0s");
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
            ROS_WARN_STREAM("[VO->Initialiser:] Right projection matrix is full of 0s");
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

            n_img_cols_ = l_info_msg->roi.width / binning_x;
            n_img_rows_ = l_info_msg->roi.height / binning_y;
        }
        else
        {
            n_img_cols_ = l_info_msg->roi.width;
            n_img_rows_ = l_info_msg->roi.height;
        }

        return true;
    }


    bool VisualOdometer::processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                             const sensor_msgs::ImageConstPtr& r_img_msg,
                                             Frame& c_frame)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // From ROS image message to cv::Mat and detects stereo keypoints.
        start_module_time = ros::Time::now().toSec();
        bool l_result, r_result;
        cv::Mat l_img, r_img, l_descs, r_descs;
        std::vector<cv::KeyPoint> l_kpts, r_kpts;
        std::string feature_detector_type = "sift";
        std::thread l_detector([&] {l_result = detectROSKeypoints(enable_enhacement_, feature_detector_type, l_img_msg, l_img, l_descs, l_kpts);});
        std::thread r_detector([&] {r_result = detectROSKeypoints(enable_enhacement_, feature_detector_type, r_img_msg, r_img, r_descs, r_kpts);});
        l_detector.join();
        r_detector.join();
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Feature extraction run time.

        if (!(l_result && r_result))
            return false;

        // Keypoint detection debugging.
        if (stereo_kps_img_pub_.getNumSubscribers() > 0)
            publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, stereo_kps_img_pub_, l_kpts, r_kpts);

        // Sanity check.
        if (l_kpts.size() < (uint)(min_lc_inliers_/2))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo keypoints detector:] There are not enough keypoints to match and filter!");
            run_times_.run_time_1 = 0.0; // Stereo matching run time.
            run_times_.run_time_2 = 0.0; // Triangulation run time.
            return false;
        }

        // Matching keypoints.
        start_module_time = ros::Time::now().toSec();
        cv::Mat m_l_descs, m_r_descs;
        std::vector<cv::KeyPoint> m_l_kpts, m_r_kpts;
        matchStereoKeypoints(l_descs, r_descs, l_kpts, r_kpts, m_l_descs, m_r_descs, m_l_kpts, m_r_kpts);
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Stereo matching run time.

        // Sanity check.
        if (m_l_kpts.size() < (uint)(min_lc_inliers_/2))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo matcher:] There are not enough keypoints after epipolar filtering!");
            run_times_.run_time_2 = 0.0; // Triangulation run time.
            return false;
        }
            
        // Compute 3D keypoints. 
        start_module_time = ros::Time::now().toSec();
        cv::Mat f_l_descs, f_r_descs;
        std::vector<cv::Point3f> f_cam_pts;
        std::vector<cv::KeyPoint> f_l_kpts, f_r_kpts;
        compute3DPoints(m_l_descs, m_r_descs, m_l_kpts, m_r_kpts, f_l_descs, f_r_descs, f_l_kpts, f_r_kpts, f_cam_pts);
        run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Triangulation run time.

        // Sanity check.
        if (f_l_kpts.size() < (uint)(min_lc_inliers_/2))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Triangulator:] There are not enough 3D points after triangulation!");
            return false;
        }

        // Stereo matching keypoints debugging.
        if (stereo_matches_img_pub_.getNumSubscribers() > 0)
            publishMatches(false, l_img, r_img, l_img_msg->header.stamp, stereo_matches_img_pub_, f_l_kpts, f_r_kpts);

        // Fill Frame object
        c_frame = Frame(l_img, r_img, f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kpts, f_r_kpts, f_cam_pts);

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Stereo frame processor:] Stereoscopic image processed successfully!");

        return true;
    }


    bool VisualOdometer::computeMotionSuccessiveFrames(const Frame& c_frame,
                                                       tf::Transform& c_camera_motion)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Search for matches of keypoints between successive frames or pairs of frames.
        start_module_time = ros::Time::now().toSec();
        std::vector<cv::Point3f> m_c_cam_pts, m_p_cam_pts;
        std::vector<cv::KeyPoint> m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts;
        matchNonStereoKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), p_frame_.getLeftDescs(), p_frame_.getLeftDescs(), 
                                c_frame.getLeftKpts(), c_frame.getRightKpts(), p_frame_.getLeftKpts(), p_frame_.getRightKpts(), 
                                c_frame.getCameraPoints(), p_frame_.getCameraPoints(),
                                m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts, m_c_cam_pts, m_p_cam_pts);
        run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Successive matching run time.

        // Sanity check.
        if (m_c_l_kpts.size() < (uint)(1.5 * min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Non-stereo matcher:] There are not enough keypoints after outlier filtering!");
            run_times_.run_time_4 = 0.0; // Bucketing run time.
            run_times_.run_time_5 = 0.0; // Motion estimation run time.
            return false;
        }
        else
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Non-stereo matcher:] Succesful matching!");
        }

        // Non stereo matching keypoints debugging.
        if (successive_matches_img_pub_.getNumSubscribers() > 0)
        {
            if (matching_type_ == RATIO_MATCHING || matching_type_ == ESSENTIAL_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
                publishMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                               successive_matches_img_pub_, m_c_l_kpts, m_p_l_kpts);

            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                       c_frame.getTimestamp(), successive_matches_img_pub_, m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts);

            else
                ROS_ERROR_STREAM("[VO->Non-stereo matcher debugger:] Unforeseen type of matching!");
        }

        // Buckets keypoints.
        start_module_time = ros::Time::now().toSec();
        std::vector<cv::Point3f> f_c_cam_pts, f_p_cam_pts;
        std::vector<cv::KeyPoint> f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts;
        if (enable_bucketing_)
        {
            bucketKeypoints(n_img_rows_, n_img_cols_, max_kpts_bucket_, m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts, m_c_cam_pts, m_p_cam_pts,
                            f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_cam_pts, f_p_cam_pts);
        }
        else
        {
            f_c_l_kpts = m_c_l_kpts;
            f_c_r_kpts = m_c_r_kpts;
            f_p_l_kpts = m_p_l_kpts;
            f_p_r_kpts = m_p_r_kpts;
            f_c_cam_pts = m_c_cam_pts;
            f_p_cam_pts = m_p_cam_pts;
        }
        run_times_.run_time_4 = ros::Time::now().toSec() - start_module_time; // Bucketing run time.

        // Estimate the motion.
        start_module_time = ros::Time::now().toSec();
        cv::Mat camera_rotation, camera_translation;
        std::vector<cv::Point3f> i_c_cam_pts, i_p_cam_pts;
        std::vector<cv::KeyPoint> i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts; 
        estimateMotion(f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_cam_pts, f_p_cam_pts,
                       camera_rotation, camera_translation, 
                       i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts, i_c_cam_pts, i_p_cam_pts);
        run_times_.run_time_5 = ros::Time::now().toSec() - start_module_time; // Motion estimation run time.

        if (i_c_l_kpts.size() < (uint)(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Motion estimator:] There are not enough inliers after motion estimation!");
            return false;
        }
        else
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Motion estimator:] Succesful motion estimation!");
        }

        // Inliers debugging.
        if (inliers_img_pub_.getNumSubscribers() > 0)
        {
            if (matching_type_ == RATIO_MATCHING || matching_type_ == ESSENTIAL_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
                publishMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                               inliers_img_pub_, i_c_l_kpts, i_p_l_kpts);

            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                       c_frame.getTimestamp(), inliers_img_pub_, i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts);

            else
                ROS_ERROR_STREAM("[VO->Motion estimator debugger:] Unforeseen type of matching!");
        }

        // Build a TF
        if(!fromMatToTf(camera_rotation, camera_translation, c_camera_motion))
            return false;

        return true;
    }


    void VisualOdometer::getInitialOdom()
    {
        slamon::GetTransform srv;
        if (initial_odom_client_.call(srv))
        {
            double tx = srv.response.tf.pose.position.x;
            double ty = srv.response.tf.pose.position.y;
            double tz = srv.response.tf.pose.position.z;
            double qx = srv.response.tf.pose.orientation.x;
            double qy = srv.response.tf.pose.orientation.y;
            double qz = srv.response.tf.pose.orientation.z;
            double qw = srv.response.tf.pose.orientation.w;

            ROS_WARN_STREAM("[VO->Initial odom:] GetTransform service call successful. The initial odometry is as follows:" << std::endl <<
                            "tx: " << tx << std::endl << 
                            "ty: " << ty << std::endl << 
                            "tz: " << tz << std::endl << 
                            "qx: " << qx << std::endl << 
                            "qy: " << qy << std::endl << 
                            "qz: " << qz << std::endl << 
                            "qw: " << qw);

            tf::Vector3 translation(tx, ty, tz);
            tf::Quaternion rotation(qx, qy, qz, qw);
            c_robot_odom_.setOrigin(translation);
            c_robot_odom_.setRotation(rotation);
        }
        else
        {
            ROS_WARN_STREAM("[VO->Initial odom:] Failed to call GetTransform service. The initial odometry will be assumed as the identity.");
            c_robot_odom_.setIdentity();
        }
        c_camera_odom_ = c_robot_odom_ * robot2camera_;
    }


    bool VisualOdometer::detectROSKeypoints(const bool enable_enhacement,
                                            const std::string& feature_detector_type,
                                            const sensor_msgs::ImageConstPtr& img_msg,
                                            cv::Mat& img,
                                            cv::Mat& descs,
                                            std::vector<cv::KeyPoint>& kpts)
    {
        
        if (!fromImgMsgToMat(img_msg, img))
            return false;

        detectKeypoints(img, enable_enhacement, feature_detector_type, descs, kpts);
        return true;
    }


    bool VisualOdometer::fromImgMsgToMat(const sensor_msgs::ImageConstPtr& msg,
                                         cv::Mat& img)
    {
        // Convert message to cv::Mat
        cv_bridge::CvImagePtr img_ptr;
        try
        {
            // TODO: cv_bridge gives problems with OpenCV-4.2.0-contrib. 
            img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("[VO->Caster:] cv_bridge exception: " << e.what());
            return false;
        }
        
        // Set the images
        img = img_ptr->image;
        
        return true;
    }

    
}