#include "loop_closing.hpp"


namespace slam
{


    LoopClosing::LoopClosing(int min_lc_inliers,
                             bool enable_debugging,
                             const std::string& w_path)
    {
        // Initialises parameters.
        w_path_ = w_path;
        min_lc_inliers_ = min_lc_inliers;
        enable_debugging_ = enable_debugging;

        cluster_counter_ = 0;
        
        p_frame_id_ = 0;

        // Create debugging directories.
        if (enable_debugging_)
        {
            keyframe_path_ = w_path_ + "keyframes/";
            clusters_path_ = w_path_ + "lc_clusters/";
            loop_closures_path_ = w_path_ + "loop_closures/";
            if (std::filesystem::exists(clusters_path_))
                std::filesystem::remove_all(clusters_path_);
            if (std::filesystem::exists(loop_closures_path_))
                std::filesystem::remove_all(loop_closures_path_);
            std::filesystem::create_directory(clusters_path_);
            std::filesystem::create_directory(loop_closures_path_);
        }
    }


    void LoopClosing::run()
    {
        // Loop
        ros::Rate r(2);
        while (ros::ok())
        {
            ROS_INFO_STREAM("[LOOP_CLOSING->run]: HELLO THERE");
            // Process new cluster
            if (checkNewClusterInQueue())
            {
                processNewCluster();
            }
            
            r.sleep();
        }
    }


    void LoopClosing::addClusterToQueue(Cluster cluster)
    {
        mutex_cluster_queue_.lock();
        cluster_queue_.push_back(cluster);
        mutex_cluster_queue_.unlock();
    }


    bool LoopClosing::checkNewClusterInQueue()
    {
        mutex_cluster_queue_.lock();
        bool full = !cluster_queue_.empty();
        mutex_cluster_queue_.unlock();
        ROS_INFO_STREAM("[LOOP_CLOSING->checkNewClusterInQueue]: Full: " << full);
        return full;   
    }


    void LoopClosing::processNewCluster()
    {
        mutex_cluster_queue_.lock();
        c_cluster_ = cluster_queue_.front();
        cluster_queue_.pop_front();
        mutex_cluster_queue_.unlock();

        ROS_INFO_STREAM("[LOOP_CLOSING->processNewCluster]: General Kenobi");

        if (enable_debugging_)
        {
            cv::Mat l_img;
            std::string c_keyframe_name = convertTo5Digits(c_cluster_.getFrameId());
            if (std::filesystem::exists(clusters_path_ + c_keyframe_name + ".png"))
            {
                ROS_INFO_STREAM(clusters_path_ + c_keyframe_name + ".png");
                l_img = cv::imread(clusters_path_ + c_keyframe_name + ".png");
            }
            else
            {
                ROS_INFO_STREAM(keyframe_path_ + c_keyframe_name + "_left.png");
                l_img = cv::imread(keyframe_path_ + c_keyframe_name + "_left.png");
            }

            if (p_frame_id_ == c_cluster_.getFrameId())
                cluster_counter_++;
            else
                cluster_counter_ = 1;

            cv::Scalar color;
            cv::RNG rng(12345);
            for (int i = 0; i < cluster_counter_; i++)
            {
                color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
                writeKeypoints(l_img, color, clusters_path_ + c_keyframe_name, c_cluster_.getLeftKpts());
            }
            p_frame_id_ = c_cluster_.getFrameId();
        }
    }


}