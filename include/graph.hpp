/**
 *  @file
 *  @brief Graph class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "frame.hpp"
#include "caster.hpp"
#include "cluster.hpp"
#include "publisher.hpp"

#include "slamon/GetTransform.h"


class Graph: protected Caster, protected Publisher
{   
    public:

        /** \brief Class constructor
         *  \param min_lc_inliers Minimum number of inliers for loop closing.
         *  \param min_dist_kf Minimum distance between keyframes.
         *  \param enable_debugging Enabling or disabling debugging.
         *  \param w_path Working path for debugging purpose.
         */
        Graph(int min_lc_inliers,
              double min_dist_kf,
              bool enable_debugging,
              const std::string& w_path);


        /** \brief Starts graph
         */
        void run();


    protected:


        /** \brief Messages callback. This function is called when synchronized frame
         *  and odom messages are received.
         *  \param frame_msg Custom ROS message containing current stereo pair image
         *  relevant information (input).
         *  \param odom_msg ROS message containing current robot odometry (input).
         */  
        void frameCallback(const slamon::FrameConstPtr& frame_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg);


        /** \brief Decides if new keyframe is needed.
         *  \param c_frame Current frame (input).
         *  \param c_robot_pose Current robot pose (input).
         *  @return True if new keyframe will be inserted into the graph.
         */
        bool needNewKeyFrame(Frame& c_frame, const tf::Transform& c_robot_pose);


        /** \brief Adds a frame to the graph if enough inliers
         *  \param c_frame Current frame (input).
         *  @return True if new keyframe will be inserted into the map
         */
        bool addFrameToMap(Frame& c_frame);


        /** \brief Computes the absolute diference between 2 poses (omitting z).
         *  \param pose_0 transformation matrix of pose 0
         *  \param pose_1 transformation matrix of pose 1
         *  @return the norm between two poses
         */
        static inline double poseDiff2D(tf::Transform pose_0, tf::Transform pose_1)
        {
            tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
            return sqrt(d.x()*d.x() + d.y()*d.y());
        }


        /** \brief Computes the absolute diference between 2 poses.
         *  \param pose_0 transformation matrix of pose 0
         *  \param pose_1 transformation matrix of pose 1
         *  @return the norm between two poses
         */
        static inline double poseDiff3D(tf::Transform pose_0, tf::Transform pose_1)
        {
            tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
            return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
        }

        /** \brief Obtains, from the VO and through a service, the transformation
         *  between the centre of the robot (base_link) and the sensor (camera).
         */
        void getRobot2Camera();


        /** \brief Adds a vertex to the graph
         *  \param pose Vertex pose
         *  @return the vertex id
         */
        int addVertex(tf::Transform pose);


    private:

        bool init_graph_; //!> Graph state.

        bool enable_debugging_; //!> Enabling or disabling debugging messages.
        
        int kf_counter_; //!> Keyframe counter. 

        int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

        double min_dist_kf_; //!> Minimum distance between keyframes.

        std::string w_path_; //!> Path to working directory.

        std::string f_path_; //!> Path to store frames with debugging purpose.

        std::string kf_path_; //!> Path to store keyframes with debugging purpose.

        std::string clusters_path_; //!> Path to store keypoints clustering with debugging purpose.

        tf::Transform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

        tf::Transform l_kf_robot_pose_; //!> The robot's pose the last time a keyframe was inserted.

        std::vector<double> frame_stamps_; //> Stores the frame timestamps

        ros::Publisher clustering_img_pub_; //!> Publisher for the frame clustering

        ros::ServiceClient robot2camera_client_; //!> Service client to assign the robot2camera static transform.

        std::vector<tf::Transform> initial_cluster_pose_history_; //!> Stores the initial cluster poses, before graph update.

        std::mutex mutex_graph_; //!> Mutex for the graph manipulation.

        g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer.

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<slamon::Frame,
                                                                nav_msgs::Odometry> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

};


#endif // GRAPH_HPP