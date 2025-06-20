#include "visual_odometer.hpp"


namespace slamon
{


    VisualOdometer::VisualOdometer(const int min_inliers,
                                   const int min_lc_inliers,
                                   const int max_kpts_bucket,
                                   const int max_kpts_retained,
                                   const bool enable_debugging,
                                   const bool enable_bucketing,
                                   const bool enable_enhacement,
                                   const std::string& matching_type,
                                   const std::string& robot_frame_id,
                                   const std::string& feature_detector_name,
                                   const std::string& motion_estimation_type):
                                   KeypointMatcher(matching_type), 
                                   MotionEstimator(motion_estimation_type),
                                   c_keyframe_id_{0}, c_cluster_id_{0}, min_inliers_{min_inliers}, min_lc_inliers_{min_lc_inliers}, 
                                   max_kpts_bucket_{max_kpts_bucket}, max_kpts_retained_{max_kpts_retained}, status_{UNINITIALISED}, 
                                   enable_debugging_{enable_debugging}, enable_bucketing_{enable_bucketing}, enable_enhacement_{enable_enhacement},
                                   initialised_odom_{false}, robot_frame_id_{robot_frame_id}, feature_detector_name_{feature_detector_name} {}


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
        keyframe_pub_ = nhp.advertise<slamon::Keyframe>("keyframe", 1);
        run_times_pub_ = nhp.advertise<slamon::Performance>("vo_performance", 1);
        robot_odom_pub_ = nhp.advertise<nav_msgs::Odometry>("robot_odom", 1);
        camera_odom_pub_ = nhp.advertise<nav_msgs::Odometry>("camera_odom", 1);
        inliers_num_pub_ = nhp.advertise<std_msgs::Int32>("inliers_num", 1);
        inliers_img_pub_ = nhp.advertise<sensor_msgs::Image>("inliers_img", 1);
        stereo_kpts_img_pub_ = nhp.advertise<sensor_msgs::Image>("stereo_keypoints_img", 1);
        stereo_matches_num_pub_ = nhp.advertise<std_msgs::Int32>("stereo_matches_num", 1);
        stereo_matches_img_pub_ = nhp.advertise<sensor_msgs::Image>("stereo_matches_img", 1);
        temporal_matches_num_pub_ = nhp.advertise<std_msgs::Int32>("temporal_matches_num", 1);
        temporal_matches_img_pub_ = nhp.advertise<sensor_msgs::Image>("temporal_matches_img", 1);
        keyframe_inliers_img_pub_ = nhp.advertise<sensor_msgs::Image>("keyframe_insertion_decision_img", 1);

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


    bool VisualOdometer::sendRobot2Camera(slamon::GetTransform::Request& req,
                                          slamon::GetTransform::Response& res)
    {
        res.tf.header.frame_id = camera_frame_id_;
        tf::poseTFToMsg(robot2camera_, res.tf.pose);
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

        // Reset run time structure.
        run_times_.reset();

        // Different actions depending on the status.
        switch (status_)
        {
            case UNINITIALISED:
            {
                if (enable_debugging_)
                    ROS_WARN_STREAM("[VO->Status:] UNINITIALISED");

                // Transition.
                if (!initialisation(l_info_msg, r_info_msg))
                    break;
                if (!processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                {
                    status_ = BOOTSTRAPPING;
                    break;
                }
                status_ = LOST;
                break;
            }
            case BOOTSTRAPPING:
            {
                if (enable_debugging_)
                    ROS_WARN_STREAM("[VO->Status:] BOOTSTRAPPING");

                // Transition.
                if (!processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                    break;
                status_ = LOST;

                break;
            }
            case LOST:
            {
                if (enable_debugging_)
                    ROS_WARN_STREAM("[VO->Status:] LOST");
                
                // Transition.
                if (!processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                {
                    status_ = BOOTSTRAPPING;
                    break;
                }
                tf::Transform camera_delta_odom;
                valid_motion = computeOdometry(c_frame, camera_delta_odom);
                if (!valid_motion)
                    break;
                status_ = RUNNING;

                // Sends keyframe.
                keyframe_ = p_frame_;
                bool lost = true;
                if (!initialised_odom_)
                {
                    initialised_odom_ = true;
                    getInitialOdom();
                    lost = false;
                }
                sendKeyframe(std::vector<int>(), std::vector<std::pair<int, int>>());
                publishPose(keyframe_.getTimestamp(), c_robot_odom_, lost, robot_odom_pub_, robot_frame_id_);
                publishPose(keyframe_.getTimestamp(), c_camera_odom_, lost, camera_odom_pub_, camera_frame_id_);
                c_cluster_id_ += keyframe_.getClustersNumber();
                c_keyframe_id_++;
                
                // Computes and publishes odometry.
                integrateAndPublish(c_frame, camera_delta_odom);

                // Updates current frame clusters ids.
                c_frame.increaseClustersIds(keyframe_.getClustersNumber());

                // Sends a keyframe to the map if necessary.
                sendKeyframeIfNecessary(c_frame);

                break;
            }
            case RUNNING:
            {
                if (enable_debugging_)
                    ROS_WARN_STREAM("[VO->Status:] RUNNING");

                // Transition.
                if (!processCurrentFrame(l_img_msg, r_img_msg, c_frame))
                {
                    status_ = BOOTSTRAPPING;
                    break;
                }
                tf::Transform camera_delta_odom;
                valid_motion = computeOdometry(c_frame, camera_delta_odom);
                if (!valid_motion)
                {
                    status_ = LOST;
                    break;
                }
                                
                // Computes and publishes odometry.
                integrateAndPublish(c_frame, camera_delta_odom);

                // Sends a keyframe to the map if necessary.
                sendKeyframeIfNecessary(c_frame);

                break;
            }
            default:
            {
                ROS_ERROR_STREAM("[VO->Status:] Unknown status encountered.");
                break;
            }
        }

        // Store current frame as previous.
        p_frame_ = c_frame;

        // Publishes last iteration performance.
        run_times_.total_run_time = ros::Time::now().toSec() - start_callback_time;
        if (run_times_pub_.getNumSubscribers() > 0)
            publishPerformance(l_img_msg->header.stamp, static_cast<int>(valid_motion), run_times_, run_times_pub_);
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
        catch (const tf::LookupException& e)
        {
            ROS_WARN_STREAM("[VO->Initialiser:] TF listener exception: " << e.what());
            ROS_WARN_STREAM("[VO->Initialiser:] The transform between the robot and the camera has not been found, and therefore robot2camera_ is equal to the identity.");
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
        for (int i = 0; i < l_proj_mat_.rows; ++i)
        {
            for (int j = 0; j < l_proj_mat_.cols; ++j)
                counter += l_proj_mat_.at<double>(i, j);
        }
        if (counter == 0.0)
        {
            ROS_WARN_STREAM("[VO->Initialiser:] Left projection matrix is full of 0s");
            return false;
        }
        counter = 0.0; 
        for (int i = 0; i < r_proj_mat_.rows; ++i)
        {
            for (int j = 0; j < r_proj_mat_.cols; ++j)
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

            img_width_ = l_info_msg->roi.width / binning_x;
            img_height_ = l_info_msg->roi.height / binning_y;
        }
        else
        {
            img_width_ = l_info_msg->roi.width;
            img_height_ = l_info_msg->roi.height;
        }

        return true;
    }


    bool VisualOdometer::processCurrentFrame(const sensor_msgs::ImageConstPtr& l_img_msg,
                                             const sensor_msgs::ImageConstPtr& r_img_msg,
                                             Frame& c_frame)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Detects features in a ROS image.
        start_module_time = ros::Time::now().toSec();
        bool l_result, r_result;
        cv::Mat l_img, r_img, l_descs, r_descs;
        std::vector<cv::KeyPoint> l_kpts, r_kpts;
        std::thread l_detector([&] {l_result = detectROSKeypoints(max_kpts_retained_, enable_enhacement_, feature_detector_name_, 
                                                                  std::ref(l_img_msg), std::ref(l_img), std::ref(l_descs), std::ref(l_kpts));});
        std::thread r_detector([&] {r_result = detectROSKeypoints(max_kpts_retained_, enable_enhacement_, feature_detector_name_, 
                                                                  std::ref(r_img_msg), std::ref(r_img), std::ref(r_descs), std::ref(r_kpts));});
        l_detector.join();
        r_detector.join();
        run_times_.run_time_0 = ros::Time::now().toSec() - start_module_time; // Feature extraction run time.

        if (!(l_result && r_result))
            return false;

        // Keypoint detection debugging.
        if (stereo_kpts_img_pub_.getNumSubscribers() > 0)
            publishImgStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, stereo_kpts_img_pub_, l_kpts, r_kpts);

        // Sanity check.
        if (l_kpts.size() < static_cast<size_t>(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo keypoints detector:] There are not enough keypoints to match and filter!");
            return false;
        }

        // Matching keypoints.
        start_module_time = ros::Time::now().toSec();
        cv::Mat m_l_descs, m_r_descs;
        std::vector<cv::KeyPoint> m_l_kpts, m_r_kpts;
        matchStereoKeypoints(l_descs, r_descs, l_kpts, r_kpts, m_l_descs, m_r_descs, m_l_kpts, m_r_kpts);
        run_times_.run_time_1 = ros::Time::now().toSec() - start_module_time; // Stereo matching run time.

        // Sanity check.
        if (m_l_kpts.size() < static_cast<size_t>(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo matcher:] There are not enough keypoints after epipolar filtering!");
            return false;
        }
            
        // Compute 3D keypoints. 
        start_module_time = ros::Time::now().toSec();
        cv::Mat f_l_descs, f_r_descs;
        std::vector<cv::Point3f> f_l_cam_pts;
        std::vector<cv::KeyPoint> f_l_kpts, f_r_kpts;
        compute3DPoints(l_proj_mat_, r_proj_mat_, 
                        m_l_descs, m_r_descs, m_l_kpts, m_r_kpts, 
                        f_l_descs, f_r_descs, f_l_kpts, f_r_kpts, f_l_cam_pts);
        run_times_.run_time_2 = ros::Time::now().toSec() - start_module_time; // Triangulation run time.

        // Sanity check.
        if (f_l_kpts.size() < static_cast<size_t>(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Triangulator:] There are not enough 3D points after triangulation!");
            return false;
        }

        // Stereo matching keypoints debugging.
        if (stereo_matches_num_pub_.getNumSubscribers() > 0)
            publishCounter(f_l_kpts.size(), stereo_matches_num_pub_);
        if (stereo_matches_img_pub_.getNumSubscribers() > 0)
            publishImgMatches(false, l_img, r_img, l_img_msg->header.stamp, stereo_matches_img_pub_, f_l_kpts, f_r_kpts);

        // Fill Frame object
        c_frame = Frame(l_img, r_img, f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kpts, f_r_kpts, f_l_cam_pts);

        // Keypoint clustering.
        start_module_time = ros::Time::now().toSec();
        c_frame.regionClustering(c_cluster_id_);
        run_times_.run_time_3 = ros::Time::now().toSec() - start_module_time; // Clustering run time.

        // Sanity check.
        if (!c_frame.hasClusters())
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Clusterator:] Image produces 0 clusters, cannot be sent to mapmaker!");
            return false;
        }

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Stereo frame processor:] Stereoscopic image processed successfully!");

        return true;
    }


    bool VisualOdometer::computeOdometry(const Frame& c_frame,
                                         tf::Transform& camera_delta_odom)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time;

        // Search for keypoint matches between successive frames or pairs of frames.
        start_module_time = ros::Time::now().toSec();
        std::vector<cv::Point3f> m_c_l_cam_pts, m_p_l_cam_pts;
        std::vector<cv::KeyPoint> m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts;
        matchTemporalKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), p_frame_.getLeftDescs(), p_frame_.getLeftDescs(), 
                               c_frame.getLeftKpts(), c_frame.getRightKpts(), p_frame_.getLeftKpts(), p_frame_.getRightKpts(), 
                               c_frame.getLeftCameraPoints(), p_frame_.getLeftCameraPoints(),
                               m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts, m_c_l_cam_pts, m_p_l_cam_pts);
        run_times_.run_time_4 = ros::Time::now().toSec() - start_module_time; // Temporal matching run time.

        // Non stereo matching keypoints debugging.
        if (temporal_matches_num_pub_.getNumSubscribers() > 0)
            publishCounter(m_c_l_kpts.size(), temporal_matches_num_pub_);
        if (temporal_matches_img_pub_.getNumSubscribers() > 0)
        {
            if (matching_type_ == RATIO_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
                publishImgMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                                  temporal_matches_img_pub_, m_c_l_kpts, m_p_l_kpts);
            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishImgCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                          c_frame.getTimestamp(), temporal_matches_img_pub_, m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts);
            else
                ROS_ERROR_STREAM("[VO->Temporal matcher debugger:] Unforeseen type of matching!");
        }

        // Sanity check.
        if (m_c_l_kpts.size() < static_cast<size_t>(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Temporal matcher:] There are not enough keypoints after outlier filtering!");
            return false;
        }

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Temporal matcher:] Succesful matching!");

        // Buckets keypoints.
        start_module_time = ros::Time::now().toSec();
        std::vector<cv::Point3f> f_c_l_cam_pts, f_p_l_cam_pts;
        std::vector<cv::KeyPoint> f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts;
        if (enable_bucketing_)
        {
            bucketKeypoints(static_cast<size_t>(max_kpts_bucket_), img_width_, img_height_,
                            m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts, m_c_l_cam_pts, m_p_l_cam_pts,
                            f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_l_cam_pts, f_p_l_cam_pts);
        }
        else
        {
            f_c_l_kpts = m_c_l_kpts;
            f_c_r_kpts = m_c_r_kpts;
            f_p_l_kpts = m_p_l_kpts;
            f_p_r_kpts = m_p_r_kpts;
            f_c_l_cam_pts = m_c_l_cam_pts;
            f_p_l_cam_pts = m_p_l_cam_pts;
        }
        run_times_.run_time_5 = ros::Time::now().toSec() - start_module_time; // Bucketing run time.

        // Estimate the motion.
        start_module_time = ros::Time::now().toSec();
        cv::Mat camera_rotation, camera_translation;
        std::vector<cv::Point3f> i_c_l_cam_pts, i_p_l_cam_pts;
        std::vector<cv::KeyPoint> i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts; 
        estimateRefinedMotion(l_proj_mat_, r_proj_mat_,
                              f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_l_cam_pts, f_p_l_cam_pts,
                              camera_rotation, camera_translation, 
                              i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts, i_c_l_cam_pts, i_p_l_cam_pts);
        run_times_.run_time_6 = ros::Time::now().toSec() - start_module_time; // Motion estimation run time.

        // Inliers debugging.
        if (inliers_num_pub_.getNumSubscribers() > 0)
            publishCounter(i_c_l_kpts.size(), inliers_num_pub_);
        if (inliers_img_pub_.getNumSubscribers() > 0)
        {
            if (matching_type_ == RATIO_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
                publishImgMatches(true, c_frame.getLeftImg(), p_frame_.getLeftImg(), c_frame.getTimestamp(),
                                  inliers_img_pub_, i_c_l_kpts, i_p_l_kpts);
            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishImgCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                          c_frame.getTimestamp(), inliers_img_pub_, i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts);
            else
                ROS_ERROR_STREAM("[VO->Motion estimator debugger:] Unforeseen type of matching!");
        }

        // Sanity check.
        if (i_c_l_kpts.size() < static_cast<size_t>(min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Motion estimator:] There are not enough inliers after motion estimation!");
            return false;
        }

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Motion estimator:] Succesful motion estimation!");

        // Build a TF
        if(!fromMatToTf(camera_rotation, camera_translation, camera_delta_odom))
            return false;

        return true;
    }


    void VisualOdometer::getInitialOdom()
    {
        slamon::GetTransform srv;
        if (initial_odom_client_.call(srv))
        {
            tf::poseMsgToTF(srv.response.tf.pose, c_robot_odom_);
            ROS_WARN_STREAM("[VO->Initial odom:] GetTransform service call successful. The initial odometry is as follows:" << std::endl <<
                            "tx: " << c_robot_odom_.getOrigin().getX() << std::endl << 
                            "ty: " << c_robot_odom_.getOrigin().getY() << std::endl << 
                            "tz: " << c_robot_odom_.getOrigin().getZ() << std::endl << 
                            "qx: " << c_robot_odom_.getRotation().getX() << std::endl << 
                            "qy: " << c_robot_odom_.getRotation().getY() << std::endl << 
                            "qz: " << c_robot_odom_.getRotation().getZ() << std::endl << 
                            "qw: " << c_robot_odom_.getRotation().getW());
        }
        else
        {
            ROS_WARN_STREAM("[VO->Initial odom:] Failed to call GetTransform service. The initial odometry will be assumed as the identity.");
            c_robot_odom_.setIdentity();
        }
        c_camera_odom_ = c_robot_odom_ * robot2camera_;
    }


    void VisualOdometer::sendKeyframe(const std::vector<int>& inliers_per_pair,
                                      const std::vector<std::pair<int, int>>& cluster_pairs)
    {
        if (enable_debugging_)
        {
            std::string text = "[VO->Keyframe sender:] Sending keyframe: " + std::to_string(c_keyframe_id_) + " with clusters: ";
            for (int i = 0; i < keyframe_.getClustersNumber(); ++i)
                text += std::to_string(c_cluster_id_ + i) + " ";
            ROS_INFO_STREAM(text);
        }

        if (enable_debugging_)
            publishKeyframe(c_keyframe_id_, keyframe_.getLeftImg(), keyframe_.getRightImg(),
                            keyframe_.getLeftDescs(), keyframe_.getRightDescs(), keyframe_.getTimestamp(),
                            keyframe_pub_, inliers_per_pair, keyframe_.getClusterIds(), 
                            keyframe_.getLeftKpts(), keyframe_.getRightKpts(), keyframe_.getLeftCameraPoints(),
                            keyframe_.getClustersCentroids(), cluster_pairs);
        else
            publishKeyframe(c_keyframe_id_, keyframe_.getLeftDescs(), keyframe_.getRightDescs(), keyframe_.getTimestamp(),
                            keyframe_pub_, inliers_per_pair, keyframe_.getClusterIds(), 
                            keyframe_.getLeftKpts(), keyframe_.getRightKpts(), keyframe_.getLeftCameraPoints(),
                            keyframe_.getClustersCentroids(), cluster_pairs);
    }


    void VisualOdometer::integrateAndPublish(const Frame& c_frame,
                                             tf::Transform& camera_delta_odom)
    {
        // From {c_cam}^T_{p_cam} to {p_cam}^T_{c_cam}.
        camera_delta_odom = camera_delta_odom.inverse();

        // Calculates the camera's current odometry by accumulating its movement.
        // {wo}^T_{c_cam} = {wo}^T_{p_cam} * {p_cam}^T_{c_cam}.
        c_camera_odom_ *= camera_delta_odom;

        // Changes the coordinate axis of the movement. From camera to robot. 
        // {p_robot}^T_{c_robot} = {robot}^T_{cam} * {p_cam}^T_{c_cam} * {cam}^T_{robot}.
        tf::Transform robot_delta_odom = robot2camera_ * camera_delta_odom * robot2camera_.inverse();

        // Calculates the robot's current odometry by accumulating its movement.
        // {wo}^T_{c_robot} = {wo}^T_{p_robot} * {p_robot}^T_{c_robot}.
        c_robot_odom_ *= robot_delta_odom;

        publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), c_camera_odom_, camera_delta_odom,
                               false, false, camera_odom_pub_, camera_frame_id_);
        publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), c_robot_odom_, robot_delta_odom,
                               false, false, robot_odom_pub_, robot_frame_id_);
    }


    void VisualOdometer::sendKeyframeIfNecessary(const Frame& c_frame)
    {
        // Auxilary variable to set the start time of a module.
        double start_module_time = ros::Time::now().toSec();

        // Searches for keypoint matches between current frame and keyframe.
        std::vector<int> m_c_cluster_ids, m_k_cluster_ids;
        std::vector<cv::Point3f> m_c_l_cam_pts, m_k_l_cam_pts;
        std::vector<cv::KeyPoint> m_c_l_kpts, m_c_r_kpts, m_k_l_kpts, m_k_r_kpts;
        matchTemporalClusters(c_frame.getLeftDescs(), c_frame.getRightDescs(), keyframe_.getLeftDescs(), keyframe_.getRightDescs(),
                              c_frame.getClusterIds(), keyframe_.getClusterIds(), c_frame.getLeftKpts(), c_frame.getRightKpts(),
                              keyframe_.getLeftKpts(), keyframe_.getRightKpts(), c_frame.getLeftCameraPoints(), keyframe_.getLeftCameraPoints(),
                              m_c_cluster_ids, m_k_cluster_ids, m_c_l_kpts, m_c_r_kpts, m_k_l_kpts, m_k_r_kpts, m_c_l_cam_pts, m_k_l_cam_pts);

        // Estimates the motion.
        cv::Mat camera_rotation, camera_translation;
        std::vector<int> i_c_cluster_ids, i_k_cluster_ids;
        std::vector<cv::Point3f> i_c_l_cam_pts, i_k_l_cam_pts;
        std::vector<cv::KeyPoint> i_c_l_kpts, i_c_r_kpts, i_k_l_kpts, i_k_r_kpts; 
        estimateMotion(l_proj_mat_, r_proj_mat_, 
                       m_c_cluster_ids, m_k_cluster_ids, m_c_l_kpts, m_c_r_kpts, m_k_l_kpts, m_k_r_kpts, m_c_l_cam_pts, m_k_l_cam_pts,
                       camera_rotation, camera_translation,
                       i_c_cluster_ids, i_k_cluster_ids, i_c_l_kpts, i_c_r_kpts, i_k_l_kpts, i_k_r_kpts, i_c_l_cam_pts, i_k_l_cam_pts);

        // Inliers debugging.
        if (keyframe_inliers_img_pub_.getNumSubscribers() > 0)
        {
            if (matching_type_ == RATIO_MATCHING || matching_type_ == HOMOGRAPHY_MATCHING)
                publishImgMatches(true, c_frame.getLeftImg(), keyframe_.getLeftImg(), c_frame.getTimestamp(),
                                  keyframe_inliers_img_pub_, i_c_l_kpts, i_k_l_kpts);
            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishImgCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), keyframe_.getLeftImg(), keyframe_.getRightImg(),
                                          c_frame.getTimestamp(), keyframe_inliers_img_pub_, i_c_l_kpts, i_c_r_kpts, i_k_l_kpts, i_k_r_kpts);
            else
                ROS_ERROR_STREAM("[VO->Keyframe inserter debugger:] Unforeseen type of matching!");
        }

        // It does not insert keyframe.
        if (static_cast<size_t>(1.10 * min_lc_inliers_) < i_c_l_kpts.size())
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Keyframe inserter:] No keyframe is inserted because there is still too much overlap.");
            run_times_.run_time_7 = ros::Time::now().toSec() - start_module_time; // Keyframe insertion decision run time.
            return;
        }

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Keyframe inserter:] Sending keyframe to the map!");

        // Gets inlier per cluster pair.
        bool cluster_pair_found;
        int c_cluster_id, k_cluster_id;
        std::vector<int> inliers_per_pair;
        std::vector<std::pair<int, int>> cluster_pairs; // c_cluster_id - k_cluster_id
        for (size_t i = 0; i < i_c_cluster_ids.size(); ++i)
        {
            c_cluster_id = i_c_cluster_ids[i];
            k_cluster_id = i_k_cluster_ids[i];

            // Searches if this pair already exists.
            cluster_pair_found = false;
            for (size_t j = 0; j < cluster_pairs.size(); ++j)
            {
                if ((cluster_pairs[j].first == c_cluster_id) && (cluster_pairs[j].second == k_cluster_id))
                {
                    inliers_per_pair[j]++;
                    cluster_pair_found = true;
                    break;
                }
            }
            if (!cluster_pair_found)
            {
                cluster_pairs.emplace_back(c_cluster_id, k_cluster_id);
                inliers_per_pair.push_back(1);
            }
        }

        // Sends the new keyframe.
        keyframe_ = c_frame;
        sendKeyframe(inliers_per_pair, cluster_pairs);

        // Updates attributes.
        c_keyframe_id_++;
        c_cluster_id_ += keyframe_.getClustersNumber();

        run_times_.run_time_7 = ros::Time::now().toSec() - start_module_time; // Keyframe insertion decision run time.
        return;
    }


    bool VisualOdometer::detectROSKeypoints(const int max_kpts,
                                            const bool enable_enhacement,
                                            const std::string feature_detector_type,
                                            const sensor_msgs::ImageConstPtr& img_msg,
                                            cv::Mat& img,
                                            cv::Mat& descs,
                                            std::vector<cv::KeyPoint>& kpts)
    {
        
        if (!fromImgMsgToMat(img_msg, img))
            return false;

        detectKeypoints(img, max_kpts, enable_enhacement, feature_detector_type, descs, kpts);
        return true;
    }


}