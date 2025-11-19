/**
 *  @file mapmaker.hpp
 *  @brief The Mapmaker constructs a topological map where nodes are clusters of keypoints 
 *         and edges are the relative movement between them and the number of keypoint matchings.
 *  @package slamon
 *  @project SLAMON
 *
 *  This file contains modified portions of code derived from the original
 *  stereo_slam project developed by the Systems, Robotics and Vision Group
 *  (srv.uib.es), University of the Balearic Islands.
 *
 *  @author
 *    Original:  Pep Lluís Negre-Carrasco (2013)
 *    Modified:  Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    Both the original work and the modifications are released under the
 *    BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#ifndef MAPMAKER_HPP
#define MAPMAKER_HPP


#include <string>
#include <vector>
#include <limits>
#include <filesystem>

#include <ros/ros.h>
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


namespace slamon
{


    class Mapmaker: protected Publisher
    {
        public:


            /** \brief Class constructor
             *  \param[in] map Map object pointer.
             *  \param[in] min_lc_inliers Minimum number of inliers for loop closure.
             *  \param[in] w_path Working path for debugging purpose.
             *  \param[in] enable_debugging Enabling or disabling debugging.
             *  \param[in] cluster_queue Cluster queue pointer.
             *  \param[in] robot_frame_id Robot coordinate axis identficator (base link).
             */
            Mapmaker(Map* map,
                     const int min_lc_inliers,
                     const std::string& w_path,
                     const bool enable_debugging,
                     Queue<Cluster>* cluster_queue, 
                     const std::string& robot_frame_id);


            /** \brief Starts mapmaker.
             */
            void run();


        private:


            /** \brief Messages callback. This function is called when synchronized stereo
             *  data and odom messages are received.
             *  \param[in] odom_msg ROS message containing current robot odometry.
             *  \param[in] keyframe_msg Custom ROS message containing a keyframe.
             *  \param[in] keycloud_msg ROS message containing the pointcloud of the scene.
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
                          const slamon::KeyframeConstPtr& keyframe_msg,
                          const sensor_msgs::PointCloud2ConstPtr& keycloud_msg);


            /** \brief Callback that is called when the timer is triggered.
             *  \param[in] event ros::WallTimerEvent.
             */
            void timerCallback(const ros::WallTimerEvent& event);


            /** \brief Obtains, from the VO and through a service, the transformation
             *  between the centre of the robot (base_link) and the sensor (camera).
             */
            void getRobot2Camera();


            /** \brief Cast a batch custom ROS message to a cluster object.
             *  \param[in] keyframe_id Identifier of the keyframe to which the cluster belongs.
             *  \param[in] cluster_local_id Cluster identifier within the keyframe.
             *  \param[in] batch_msg Batch custom ROS message.
             *  @return A cluster object.
             */
            Cluster fromBatchToCluster(const int keyframe_id,
                                       const int cluster_local_id,
                                       const slamon::Batch& batch_msg);


            /** \brief Given a set of n elements, returns a vector with all possible
             *  combinations of r elements.
             *  \param[in] n Total number of elements.
             *  \param[in] r Number of elements in each combination.
             *  @return the list of combinations.
             */
            std::vector<std::vector<int>> createCombinations(const size_t n,
                                                             const size_t r);


            /** \brief Writes the keyframe poses and the loop closure edges to disk.
             */
            void writeMapResults();


            // Attributes:

            bool initialised_map_; //!> Map status.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            int max_edge_inliers_; //!> Maximum number of inliers for an edge.

            std::string poses_file_; //!> Path to the file where the keyframes positions are stored.

            std::string edges_file_; //!> Path to the file where the loop closure edges are stored.

            std::string keyframe_path_; //!> Path to store keyframes with debugging purpose.

            std::string clusters_path_; //!> Path to store keypoints clustering with debugging purpose.

            tf::Transform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            std::string robot_frame_id_; //!> Robot coordinate axis identficator (base link).

            std::string camera_frame_id_; //!> Camera coordinate axis identficator.

            ros::WallTimer timer_; //!> Timer to write the last map update to disk.

            tf::Transform p_robot_odom_; //!> The robot's odometry the last time a keyframe was inserted.

            ros::Publisher keycloud_pub_; //!> Pointcloud publisher.

            ros::Publisher robot_map_pub_; //!> Updated robot pose publisher.
            
            ros::Publisher camera_map_pub_; //!> Updated camera pose publisher.

            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher keyframes_num_pub_; // Publisher for the number of keyframes.

            ros::Publisher clustering_img_pub_; //!> Publisher for the frame clustering.

            ros::ServiceClient robot2camera_client_; //!> Service for requesting robot2camera static transform.

            Map* map_; //!> Map pointer.

            Queue<Cluster>* cluster_queue_; //!> Cluster queue pointer.

            RunTimes run_times_; //!> Internal map execution times.

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, slamon::Keyframe, sensor_msgs::PointCloud2> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };

    
} // End namespace


#endif // MAPMAKER_HPP