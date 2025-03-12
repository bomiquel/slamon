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
             *  \param[in] min_dist_keyframe Minimum distance between keyframes.
             *  \param[in] w_path Working path for debugging purpose.
             *  \param[in] cluster_queue Cluster queue pointer.
             *  \param[in] robot_frame_id Robot coordinate axis identficator (base link).
             */
            Mapmaker(Map* map,
                     int min_lc_inliers,
                     bool enable_debugging,
                     double min_dist_keyframe,
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
             *  \param[in] cloud_msg ROS message containing the pointcloud of the scene.
             *  \param[in] stereo_attributes_msg Custom ROS message containing current 
             *  stereo pair image relevant information.
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          const slamon::StereoImageAttributesConstPtr& stereo_attributes_msg);


            /** \brief Decides if new keyframe is needed.
             *  \param[in] c_odom_robot_pose Current world-odom robot pose.
             *  \param[in, out] c_frame Current frame.
             *  @return True if new keyframe will be inserted into the graph.
             */
            bool needNewKeyFrame(const tf::Transform& c_odom_robot_pose,
                                 Frame& c_frame);


            /** \brief Adds a frame to the graph if enough inliers
             *  \param[in] c_frame Current frame.
             *  @return True if new keyframe will be inserted into the map
             */
            bool addFrameToMap(Frame& c_frame);


            /** \brief Obtains, from the VO and through a service, the transformation
             *  between the centre of the robot (base_link) and the sensor (camera).
             */
            void getRobot2Camera();


            /** \brief Given a set of n elements, returns a vector with all possible
             *  combinations of r elements.
             *  \param[in] n Total number of elements.
             *  \param[in] r Number of elements in each combination.
             *  @return the list of combinations.
             */
            std::vector<std::vector<int>> createCombinations(const uint n,
                                                             const uint r);
            

            /** \brief Convert custom ROS stereo image attributes msg to frame 
             *  object (slam::Frame)
             *  \param[in] msg ROS message with relevant visual information.
             *  \param[out] frame Will contain the relevant visual information.
             */
            void fromStereoAttributesToFrame(const slamon::StereoImageAttributesConstPtr& msg,
                                             Frame& frame);


            /** \brief Convert cv::Point3f to tf::Transform.
             *  \param[in] pt 3D point.
             *  @return 3D point converted into a transform.
             */
            tf::Transform fromPoint3DToTf(const cv::Point3f& pt);


            // Attributes:

            bool init_map_; //!> Map state.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.
            
            int c_keyframe_id_; //!> Keyframe counter. 

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            double min_dist_keyframe_; //!> Minimum distance between keyframes.

            std::string poses_file_; //!> Path to the file where the keyframes positions are stored.

            std::string edges_file_; //!> Path to the file where the loop closure edges are stored.

            std::string frame_path_; //!> Path to store frames with debugging purpose.

            std::string keyframe_path_; //!> Path to store keyframes with debugging purpose.

            std::string clusters_path_; //!> Path to store keypoints clustering with debugging purpose.

            tf::Transform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            std::string robot_frame_id_; //!> Robot coordinate axis identficator (base link).

            std::string camera_frame_id_; //!> Camera coordinate axis identficator.

            tf::Transform p_odom_robot_pose_; //!> The robot's odometry the last time a keyframe was inserted.

            ros::Publisher cloud_pub_; //!> Pointcloud publisher.

            ros::Publisher robot_map_pub_; //!> Updated robot pose publisher.
            
            ros::Publisher camera_map_pub_; //!> Updated camera pose publisher.

            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher keyframes_num_pub_; // Publisher for the number of keyframes.

            ros::Publisher clustering_img_pub_; //!> Publisher for the frame clustering.

            ros::ServiceClient robot2camera_client_; //!> Service for requesting robot2camera static transform.

            std::vector<tf::Transform> initial_cluster_pose_history_; //!> Stores the initial cluster poses, before graph update.

            Map* map_; // Map pointer.

            Queue<Cluster>* cluster_queue_; // Cluster queue pointer.

            RunTimes run_times_; // !> Internal map execution times.


            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                                    sensor_msgs::PointCloud2,
                                                                    slamon::StereoImageAttributes> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };

    
} // End namespace


#endif // MAPMAKER_HPP