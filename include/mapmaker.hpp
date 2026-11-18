/**
 *  \file mapmaker.hpp
 *  \brief The Mapmaker class constructs and updates the topological pose-graph map from incoming keyframes and odometry.
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  \author Original: Pep Lluís Negre-Carrasco (2013)
 *  \author Modified: Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon). Both the original 
 *  work and the modifications are released under the BSD 4-Clause License. 
 *  See the top-level LICENSE file for details.
 */

 
#ifndef MAPMAKER_HPP
#define MAPMAKER_HPP


#include <string>
#include <vector>
#include <limits>
#include <thread>
#include <filesystem>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "map.hpp"
#include "queue.hpp"
#include "tools.hpp"
#include "frame.hpp"
#include "cluster.hpp"
#include "publisher.hpp"

#include "slamon/GetTransform.h"


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{


    /** \class Mapmaker
     *  \brief Incrementally builds the pose-graph map by synchronizing incoming keyframes and odometry.
     *  
     *  This class subscribes to synchronized robot odometry and keyframe messages, unpacks the 
     *  spatial keypoint clusters, inserts them as vertices and odometry/intra-keyframe edges 
     *  into the global `Map`, and pushes new clusters into the thread-safe queue for loop closure detection.
     */
    class Mapmaker: protected Publisher
    {
        public:


            /** \brief Parameterized class constructor.
             *  \param[in] map Pointer to the global SLAM map object.
             *  \param[in] min_lc_inliers Minimum number of inliers required for loop closure.
             *  \param[in] w_path Working directory path for storing outputs and debug data.
             *  \param[in] enable_debugging If `true`, enables debugging outputs and image saving.
             *  \param[in] cluster_queue Pointer to the thread-safe queue of clusters for the loop closer.
             *  \param[in] robot_frame_id Name of the robot base coordinate frame (e.g., `"base_link"`).
             */
            Mapmaker(Map* map,
                     const int min_lc_inliers,
                     const std::string& w_path,
                     const bool enable_debugging,
                     Queue<Cluster>* cluster_queue, 
                     const std::string& robot_frame_id);


            /** \brief Starts the mapmaker node execution loop.
             */
            void run();


        private:


            /** \brief Synchronized message callback triggered when odometry and keyframe messages arrive.
             *  \param[in] odom_msg Incoming ROS message containing the current robot odometry.
             *  \param[in] keyframe_msg Incoming custom ROS message containing a processed keyframe and its clusters.
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
                          const slamon::KeyframeConstPtr& keyframe_msg);


            /** \brief Periodic wall-timer callback triggered to save the latest map results to disk.
             *  \param[in] event The wall-clock timer event structure (`ros::WallTimerEvent`).
             */
            void timerCallback(const ros::WallTimerEvent& event);


            /** \brief Requests the static transformation between the robot base frame (`base_link`) 
             *         and the camera sensor frame from the visual odometry node via a ROS service.
             */
            void getRobot2Camera();


            /** \brief Converts a custom `slamon::Batch` ROS message into a `Cluster` object.
             *  \param[in] keyframe_id ID of the keyframe to which the cluster belongs.
             *  \param[in] cluster_local_id Local ID of the cluster within the keyframe.
             *  \param[in] batch_msg Custom `slamon::Batch` ROS message containing serialized cluster data.
             *  \return The constructed `Cluster` object.
             */
            Cluster fromBatchToCluster(const int keyframe_id,
                                       const int cluster_local_id,
                                       const slamon::Batch& batch_msg);


            /** \brief Generates all possible combinations of `r` elements from a set of `n` elements.
             *  \param[in] n Total number of elements in the set.
             *  \param[in] r Number of elements in each combination.
             *  \return Vector containing all generated index combinations.
             */
            std::vector<std::vector<int>> createCombinations(const size_t n,
                                                             const size_t r);


            /** \brief Writes the keyframe poses and loop closure edges to disk.
             */
            void writeMapResults();

            
            /** \brief Flag indicating map status. */
            bool initialised_map_;

            /** \brief Flag to enable or disable debugging outputs and image saving. */
            bool enable_debugging_;

            /** \brief Maximum number of inliers for a graph edge. */
            int max_edge_inliers_;

            /** \brief File path where the keyframe poses are saved. */
            std::string poses_file_;

            /** \brief File path where the loop closure edges are saved. */
            std::string edges_file_;

            /** \brief Directory path to store keyframe debug data. */
            std::string keyframe_path_;

            /** \brief Directory path to store keypoint clustering debug data. */
            std::string clusters_path_;

            /** \brief Robot base coordinate frame ID (e.g., `"base_link"`). */
            std::string robot_frame_id_;

            /** \brief Camera sensor coordinate frame ID. */
            std::string camera_frame_id_;

            /** \brief Pointer to the global SLAM map object. */
            Map* map_;

            /** \brief Pointer to the thread-safe queue of clusters shared with the loop closer. */
            Queue<Cluster>* cluster_queue_;

            /** \brief Execution time profiler for internal mapmaking stages. */
            RunTimes run_times_;

            /** \brief Public ROS node handle. */
            ros::NodeHandle nh_;
            
            /** \brief Private ROS node handle (`"~"`). */
            ros::NodeHandle nhp_;

            /** \brief Wall-time timer used to periodically write the latest map update to disk. */
            ros::WallTimer timer_;

            /** \brief Static transformation between the robot base frame (`base_link`) and the camera frame. */
            tf2::Transform robot2camera_;

            /** \brief Robot odometry pose recorded when the previous keyframe was inserted. */
            tf2::Transform p_robot_odom_;

            /** \brief ROS publisher for the map-corrected robot pose. */
            ros::Publisher robot_map_pub_;
            
            /** \brief ROS publisher for the map-corrected camera pose. */
            ros::Publisher camera_map_pub_;

            /** \brief ROS publisher for per-iteration mapmaker execution times. */
            ros::Publisher run_times_pub_;

            /** \brief ROS publisher for the total number of inserted keyframes. */
            ros::Publisher keyframes_num_pub_;

            /** \brief ROS publisher for keyframe spatial clustering visualization images. */
            ros::Publisher clustering_img_pub_;

            /** \brief ROS service client for requesting the static robot-to-camera transformation. */
            ros::ServiceClient robot2camera_client_;

            /** \brief Message filter subscriber for incoming robot odometry messages. */
            message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

            /** \brief Message filter subscriber for incoming keyframe messages. */
            message_filters::Subscriber<slamon::Keyframe> keyframe_sub_;

            /** \brief Approximate-time synchronization policy for odometry and keyframe messages. */
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, slamon::Keyframe> SyncPolicy_;

            /** \brief Message filter synchronizer type using `SyncPolicy_`. */
            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

            /** \brief Shared pointer to the active message filter synchronizer instance. */
            std::shared_ptr<Sync_> sync_;

    };

    
} // namespace slamon


#endif // MAPMAKER_HPP