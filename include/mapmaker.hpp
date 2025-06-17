/**
 *  @file
 *  @brief Mapmaker class.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2024-2025
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
             *  \param[in] enable_debugging Enabling or disabling debugging.
             *  \param[in] w_path Working path for debugging purpose.
             *  \param[in] cluster_queue Cluster queue pointer.
             *  \param[in] robot_frame_id Robot coordinate axis identficator (base link).
             */
            Mapmaker(Map* map,
                     int min_lc_inliers,
                     bool enable_debugging,
                     const std::string& w_path,
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
             *  \param[in] cloud_msg ROS message containing the pointcloud of the scene.
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
                          const slamon::KeyframeConstPtr& keyframe_msg,
                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


            /** \brief Obtains, from the VO and through a service, the transformation
             *  between the centre of the robot (base_link) and the sensor (camera).
             */
            void getRobot2Camera();


            /** \brief Cast a batch custom ROS message to a cluster object.
             *  \param[in] keyframe_id Identifier of the keyframe to which the cluster belongs.
             *  \param[in] cluster_local_id Cluster identifier within the keyframe.
             *  \param[in] cluster_global_id Cluster identifier within the map.
             *  \param[in] batch_msg Batch custom ROS message.
             *  @return A cluster object.
             */
            Cluster fromBatchToCluster(const int keyframe_id,
                                       const int cluster_local_id,
                                       const int cluster_global_id,
                                       const slamon::Batch& batch_msg);


            /** \brief Given a set of n elements, returns a vector with all possible
             *  combinations of r elements.
             *  \param[in] n Total number of elements.
             *  \param[in] r Number of elements in each combination.
             *  @return the list of combinations.
             */
            std::vector<std::vector<int>> createCombinations(const size_t n,
                                                             const size_t r);


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

            tf::Transform p_robot_odom_; //!> The robot's odometry the last time a keyframe was inserted.

            ros::Publisher cloud_pub_; //!> Pointcloud publisher.

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