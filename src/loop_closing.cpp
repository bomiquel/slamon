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

        // Creates execution directory
        execution_path_ = w_path_ + "haloc/";
        if (std::filesystem::exists(execution_path_))
            std::filesystem::remove_all(execution_path_);
        std::filesystem::create_directory(execution_path_);

        // Creates debugging directories.
        if (enable_debugging_)
        {
            keyframe_path_ = w_path_ + "keyframes/";
            lc_path_ = w_path_ + "loop_closures/";
            if (std::filesystem::exists(lc_path_))
                std::filesystem::remove_all(lc_path_);
            std::filesystem::create_directory(lc_path_);
        }
    }


    void LoopClosing::run()
    {
        // Loop
        ros::Rate r(2);
        while (ros::ok())
        {
            // Search loop closing in the new cluster.
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
        return full;   
    }


    void LoopClosing::processNewCluster()
    {
        mutex_cluster_queue_.lock();
        c_cluster_ = cluster_queue_.front();
        cluster_queue_.pop_front();
        mutex_cluster_queue_.unlock();

        // Initialise hash
        if (!hash_.isInitialized())
            hash_.init(c_cluster_.getLeftDescs());

        // Save hash to table
        hash_table_.push_back(std::make_pair(c_cluster_.getId(), hash_.getHash(c_cluster_.getLeftDescs())));

        // Store
        cv::FileStorage fs(execution_path_ + convertTo5Digits(c_cluster_.getId()) + ".yml", cv::FileStorage::WRITE);
        cv::write(fs, "frame_id", c_cluster_.getFrameId());
        cv::write(fs, "l_kpts", c_cluster_.getLeftKpts());
        cv::write(fs, "r_kpts", c_cluster_.getRightKpts());
        cv::write(fs, "l_descs", c_cluster_.getLeftDescs());
        cv::write(fs, "r_descs", c_cluster_.getRightDescs());
        cv::write(fs, "obj_pts", c_cluster_.getObjectPoints());
        fs.release();

        cluster_id_ = c_cluster_.getId();
    }


}