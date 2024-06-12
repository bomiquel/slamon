#include "visual_odometry.hpp"


namespace slam
{


    VisualOdometry::VisualOdometry(double ratio,
                                   int min_inliers,
                                   bool enable_debugging,
                                   const std::string& w_path,
                                   std::string& matching_type,
                                   std::string& motion_estimation_type,
                                   const std::string& robot_frame_id, 
                                   const std::string& origin_frame_id):
                                   VisionMotionModules(ratio, matching_type, motion_estimation_type)
    {
        // Inilitialises parameters.
        w_path_ = w_path;
        min_inliers_ = min_inliers;
        robot_frame_id_ = robot_frame_id;
        origin_frame_id_ = origin_frame_id;
        enable_debugging_ = enable_debugging;

        // Set "uninitialised" status to visual odometer.
        status_ = UNINITIALISED;

        // Set frame id counter.
        c_frame_id_ = 0;

        // Set false to initialised pose flag.
        initialised_pose_ = false;

        // Create debugging directories.
        if (enable_debugging_)
        {
            frame_path_ = w_path_ + "frames/";
            if (std::filesystem::exists(frame_path_))
                std::filesystem::remove_all(frame_path_);
            std::filesystem::create_directory(frame_path_);
        }
    }


    void VisualOdometry::run()
    {
        // Create ROS node.
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        image_transport::ImageTransport it(nh);

        // Client services.
        initial_pose_client_ = nhp.serviceClient<slamon::GetTransform>("get_first_pose");

        // Server servives.
        robot2camera_server_ = nhp.advertiseService("send_robot_to_camera_tf", &VisualOdometry::sendRobot2Camera, this);
        proj_mat_server_ = nhp.advertiseService("send_projection_matrices", &VisualOdometry::sendProjectionMatrices, this);

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
        
        // Calls callback when synchronised messages arrives.
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
                            getInitialPose();
                            initialised_pose_ = true;

                            if (enable_debugging_)
                            {
                                c_frame_name = convertTo5Digits(c_frame_id_);
                                cv::imwrite(frame_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                                cv::imwrite(frame_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                            }
                            publishFrame(p_frame_.getLeftDescs(), p_frame_.getRightDescs(), p_frame_.getTimestamp(), 
                                         frame_pub_, p_frame_.getLeftKpts(), p_frame_.getRightKpts(), p_frame_.getObjectPoints());
                            publishPoseAndVelocity(p_frame_.getTimestamp(), p_frame_.getTimestamp(), odom_pub_, origin_frame_id_, 
                                                   robot_frame_id_, c_robot_pose_, tf::Transform());
                        }
                        c_frame_id_++;

                        // From {c2}^T_{c1} to {c1}^T_{c2}.
                        c_camera_motion = c_camera_motion.inverse();

                        // Changes the coordinate axis of the movement. From camera to robot. 
                        // {r1}^T_{r2} = {r1}^T_{c1} * {c1}^T_{c2} * {c2}^T_{r2}
                        tf::Transform c_robot_motion = robot2camera_ * c_camera_motion * robot2camera_.inverse();

                        // Calculates the robot's current pose by accumulating its movement.
                        // {o}^T_{r2} = {o}^T_{r1} * {r1}^T_{r2}
                        c_robot_pose_ *= c_robot_motion;

                        if (enable_debugging_)
                        {
                            c_frame_name = convertTo5Digits(c_frame_id_);
                            cv::imwrite(frame_path_ + c_frame_name + "_left.png", p_frame_.getLeftImg());
                            cv::imwrite(frame_path_ + c_frame_name + "_right.png", p_frame_.getRightImg());
                        }
                        publishFrame(c_frame.getLeftDescs(), c_frame.getRightDescs(), c_frame.getTimestamp(),
                                     frame_pub_, c_frame.getLeftKpts(), c_frame.getRightKpts(), c_frame.getObjectPoints());
                        publishPoseAndVelocity(c_frame.getTimestamp(), p_frame_.getTimestamp(), odom_pub_, origin_frame_id_, 
                                               robot_frame_id_, c_robot_pose_, c_robot_motion);
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


    bool VisualOdometry::sendRobot2Camera(slamon::GetTransform::Request& req,
                                          slamon::GetTransform::Response& res)
    {
        res.tf.position.x = robot2camera_.getOrigin().getX();
        res.tf.position.y = robot2camera_.getOrigin().getY();
        res.tf.position.z = robot2camera_.getOrigin().getZ();
        res.tf.orientation.x = robot2camera_.getRotation().getX();
        res.tf.orientation.y = robot2camera_.getRotation().getY();
        res.tf.orientation.z = robot2camera_.getRotation().getZ();
        res.tf.orientation.w = robot2camera_.getRotation().getW();
        return true;
    }


    bool VisualOdometry::sendProjectionMatrices(slamon::GetProjectionMatrices::Request& req,
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
            ROS_WARN_STREAM("[VO->Initialiser:] TF listener exception: " << e.what());
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
            publishStereoKeypoints(l_img, r_img, l_img_msg->header.stamp, stereo_kps_img_pub_, l_kpts, r_kpts);

        // Sanity check.
        if (l_kpts.size() < (uint)(1.5 * min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo keypoints detector:] There are not enough keypoints to match and filter!");
            run_times_.stereo_matching = 0.0;
            run_times_.triangulation = 0.0;
            return false;
        }

        // Matching keypoints.
        start_time = ros::Time::now().toSec();
        cv::Mat h_f_l_descs, h_f_r_descs;
        std::vector<cv::Point2f> h_f_l_kpts, h_f_r_kpts;
        matchStereoKeypoints(l_descs, r_descs, l_kpts, r_kpts, h_f_l_descs, h_f_r_descs, h_f_l_kpts, h_f_r_kpts);
        run_times_.stereo_matching = ros::Time::now().toSec() - start_time;

        // Sanity check.
        if (h_f_l_kpts.size() < (uint)(1.5 * min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Stereo matcher:] There are not enough keypoints after epipolar filtering!");
            run_times_.triangulation = 0;
            return false;
        }
            
        // Compute 3D keypoints. 
        start_time = ros::Time::now().toSec();
        cv::Mat f_l_descs, f_r_descs;
        std::vector<cv::Point3f> f_obj_pts;
        std::vector<cv::Point2f> f_l_kpts, f_r_kpts;
        compute3DPoints(h_f_l_descs, h_f_r_descs, h_f_l_kpts, h_f_r_kpts, f_l_descs, f_r_descs, f_l_kpts, f_r_kpts, f_obj_pts);
        run_times_.triangulation = ros::Time::now().toSec() - start_time;

        // Sanity check.
        if (f_l_kpts.size() < (uint)(1.5 * min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Triangulator:] There are not enough 3D points after triangulation!");
            return false;
        }

        // Stereo matching keypoints debugging.
        if (stereo_matches_img_pub_.getNumSubscribers() > 0)
            publishMatches(false, l_img, r_img, l_img_msg->header.stamp, stereo_matches_img_pub_, f_l_kpts, f_r_kpts);

        // Fill Frame object
        c_frame = Frame(l_img, r_img, f_l_descs, f_r_descs, l_img_msg->header.stamp, f_l_kpts, f_r_kpts, f_obj_pts);

        if (enable_debugging_)
            ROS_INFO_STREAM("[VO->Stereo frame processor:] Stereoscopic image processed successfully!");

        return true;
    }


    bool VisualOdometry::computeMotionSuccessiveFrames(const Frame& c_frame,
                                                       tf::Transform& c_camera_motion)
    {
        // Auxilary variable to set the start time of a process
        double start_time;

        // Creates dummy variables.
        std::vector<int> c_cluster_ids, p_cluster_ids, f_c_cluster_ids, f_p_cluster_ids; 

        // Search for matches of keypoints between successive frames or pairs of frames.
        start_time = ros::Time::now().toSec();
        std::vector<cv::Point3f> f_c_obj_pts, f_p_obj_pts;
        std::vector<cv::Point2f> f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts;
        matchNonStereoKeypoints(c_frame.getLeftDescs(), c_frame.getRightDescs(), p_frame_.getLeftDescs(), p_frame_.getLeftDescs(), c_cluster_ids, p_cluster_ids, 
                                c_frame.getLeftKpts(), c_frame.getRightKpts(), p_frame_.getLeftKpts(), p_frame_.getRightKpts(), c_frame.getObjectPoints(), p_frame_.getObjectPoints(),
                                f_c_cluster_ids, f_p_cluster_ids, f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_obj_pts, f_p_obj_pts);
        run_times_.successive_matching = ros::Time::now().toSec() - start_time;

        // Sanity check.
        if (f_c_l_kpts.size() < (uint)(1.5 * min_inliers_))
        {
            if (enable_debugging_)
                ROS_INFO_STREAM("[VO->Non-stereo matcher:] There are not enough keypoints after outlier filtering!");
            run_times_.motion_estimation = 0.0;
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
                               successive_matches_img_pub_, f_c_l_kpts, f_p_l_kpts);

            else if (matching_type_ == CIRCULAR_RATIO_MATCHING || matching_type_ == CIRCULAR_ESSENTIAL_MATCHING || matching_type_ == CIRCULAR_HOMOGRAPHY_MATCHING)
                publishCircularMatches(c_frame.getLeftImg(), c_frame.getRightImg(), p_frame_.getLeftImg(), p_frame_.getRightImg(),
                                       c_frame.getTimestamp(), successive_matches_img_pub_, f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts);

            else
                ROS_ERROR_STREAM("[VO->Non-stereo matcher debugger:] Unforeseen type of matching!");
        }

        // Estimate the motion.
        start_time = ros::Time::now().toSec();
        cv::Mat camera_rotation, camera_translation;
        std::vector<cv::Point3f> i_c_obj_pts, i_p_obj_pts;
        std::vector<cv::Point2f> i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts; 
        estimateMotion(c_cluster_ids, p_cluster_ids, f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts, f_c_obj_pts, f_p_obj_pts,
                       camera_rotation, camera_translation, 
                       f_c_cluster_ids, f_p_cluster_ids, i_c_l_kpts, i_c_r_kpts, i_p_l_kpts, i_p_r_kpts, i_c_obj_pts, i_p_obj_pts);
        run_times_.motion_estimation = ros::Time::now().toSec() - start_time;
        if (i_c_l_kpts.size() < (uint)(1.5 * min_inliers_))
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


    void VisualOdometry::getInitialPose()
    {
        slamon::GetTransform srv;
        if (initial_pose_client_.call(srv))
        {
            double tx = srv.response.tf.position.x;
            double ty = srv.response.tf.position.y;
            double tz = srv.response.tf.position.z;
            double qx = srv.response.tf.orientation.x;
            double qy = srv.response.tf.orientation.y;
            double qz = srv.response.tf.orientation.z;
            double qw = srv.response.tf.orientation.w;

            ROS_WARN_STREAM("[VO->Initial pose:] GetTransform service call successful. The initial position is as follows:" << std::endl <<
                            "tx: " << tx << std::endl << 
                            "ty: " << ty << std::endl << 
                            "tz: " << tz << std::endl << 
                            "qx: " << qx << std::endl << 
                            "qy: " << qy << std::endl << 
                            "qz: " << qz << std::endl << 
                            "qw: " << qw);

            tf::Vector3 translation(tx, ty, tz);
            tf::Quaternion rotation(qx, qy, qz, qw);
            c_robot_pose_.setOrigin(translation);
            c_robot_pose_.setRotation(rotation);
        }
        else
        {
            ROS_WARN_STREAM("[VO->Initial pose:] Failed to call GetTransform service. The initial pose will be assumed as the identity.");
            c_robot_pose_.setIdentity();
        }
    }


}