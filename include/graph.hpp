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
#include <limits>
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
#include "loop_closing.hpp"

#include "slamon/GetTransform.h"


namespace slam
{


    class LoopClosing;


    class Graph: protected Caster, protected Publisher
    {   
        public:


            /** \brief Class constructor
             *  \param min_lc_inliers Minimum number of inliers for loop closing.
             *  \param enable_debugging Enabling or disabling debugging.
             *  \param min_dist_keyframe Minimum distance between keyframes.
             *  \param w_path Working path for debugging purpose.
             *  \param loop_closing Loop closing object pointer.
             *  \param robot_frame_id Robot coordinate axis identficator (base link).
             */
            Graph(int min_lc_inliers,
                  bool enable_debugging,
                  double min_dist_keyframe,
                  const std::string& w_path,
                  LoopClosing* loop_closing,
                  const std::string& robot_frame_id);


            /** \brief Starts graph
             */
            void run();


            /** \brief Get the frame id of some specific vertex.
             *  \param vertex_id The vertex id (input).
             *  @return The frame id.
             */
            int getVertexKeyframeId(int vertex_id);


            /** \brief Get the graph vertex pose.
             *  \param vertex_id The id of the vertex for which the pose is desired (input).
             *  \param lock Set to true if graph locking is required (input/optional).
             *  @return Tf with the graph pose.
             */
            tf::Transform getVertexPose(int vertex_id, bool lock);


            /** \brief Get the camera pose of some specific vertex.
             *  \param vertex_id The id of the vertex for which the pose is desired (input).
             *  \param lock Set to true if graph locking is required (input/optional).
             *  @return Tf with the graph vertex camera pose.
             */
            tf::Transform getVertexCameraPose(int vertex_id, bool lock);


            /** \brief Retrieve the list of the vertices/clusters of a corresponding keyframe.
             *  \param keyframe_id The corresponding keyframe id (input).
             *  \param vertices Will contain the list of vertices for this keyframe (output).
             */
            void getKeyframeVertices(int keyframe_id, 
                                     std::vector<int>& vertices);


            /** \brief Gets the poses of each keyframe in the graph.
             *  \param stamps The timestamp of each pose (output).
             *  \param keyframe_poses A list containing the poses of each keyframe (output).
             */
            void getKeyframePoses(std::vector<double>& stamps,
                                  std::vector<tf::Transform>& keyframe_poses);


            /** \brief Add an edge to the graph.
             *  \param vertex_0_id Index of graph vertex 0 (input).
             *  \param vertex_1_id Index of graph vertex 1 (input).
             *  \param transformation Transformation between graph vertices (input).
             */
            void addEdge(int vertex_0_id, 
                         int vertex_1_id, 
                         const tf::Transform& tf_edge);


            /** \brief Gets nearest neighbour vertices by distance.
             *  \param vertex_id The vertex id to retrieve its neighbors (input).
             *  \param window_center The vertex where discard window will be centered (input).
             *  \param discard_window The number of discarded vertices (input).
             *  \param num_closest_neighbors The number of neighbors to be retrieved (input).
             *  \param closest_neighbors Will contain the list of closest neighbors by distance (output).
             */
            void findClosestVertices(int vertex_id,
                                     int window_center,
                                     int discard_window,
                                     uint num_closest_neighbors,
                                     std::vector<int>& closest_neighbors);


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
            static inline double poseDiff2D(const tf::Transform& pose_0, 
                                            const tf::Transform& pose_1)
            {
                tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
                return sqrt(d.x()*d.x() + d.y()*d.y());
            }


            /** \brief Computes the absolute diference between 2 poses.
             *  \param pose_0 transformation matrix of pose 0
             *  \param pose_1 transformation matrix of pose 1
             *  @return the norm between two poses
             */
            static inline double poseDiff3D(const tf::Transform& pose_0,
                                            const tf::Transform& pose_1)
            {
                tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
                return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
            }

            /** \brief Obtains, from the VO and through a service, the transformation
             *  between the centre of the robot (base_link) and the sensor (camera).
             */
            void getRobot2Camera();


            /** \brief Adds a vertex to the graph.
             *  \param pose Vertex pose.
             *  @return the vertex id.
             */
            int addVertex(tf::Transform pose);


            /** \brief Return all possible combinations of 2 elements of the input vector.
             *  \param Vec Input vector with all values.
             *  @return the list of combinations.
             */
            std::vector<std::vector<int>> createComb(const std::vector<int>& vec);


            /** \brief Returns whether the second element of the first
             *  std::pair is less than the second element of the second std::pair
             *  \param d_0 First pair (input).
             *  \param d_1 Second pair (input).
             *  @return true if first pair is smaller than second pair.
             */
            static inline bool sortByDistance(const std::pair<int, double> d_0, const std::pair<int, double> d_1)
            {
                return (d_0.second < d_1.second);
            }


        private:

            bool init_graph_; //!> Graph state.

            bool enable_debugging_; //!> Enabling or disabling debugging messages.
            
            int c_keyframe_id_; //!> Keyframe counter. 

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            double min_dist_keyframe_; //!> Minimum distance between keyframes.

            std::string w_path_; //!> Path to working directory.

            std::string frame_path_; //!> Path to store frames with debugging purpose.

            std::string keyframe_path_; //!> Path to store keyframes with debugging purpose.

            std::string clusters_path_; //!> Path to store keypoints clustering with debugging purpose.

            tf::Transform robot2camera_; //!> Transformation between robot frame (base link) and camera frame.

            std::string robot_frame_id_; //!> Robot coordinate axis identficator (base link).

            tf::Transform p_keyframe_robot_pose_; //!> The robot's pose the last time a keyframe was inserted.

            std::vector<double> keyframe_stamps_; //> Stores the frame timestamps

            ros::Publisher map_pub_; //!> Updated robot pose publisher.

            ros::Publisher clustering_img_pub_; //!> Publisher for the frame clustering.

            ros::ServiceClient robot2camera_client_; //!> Service for requesting robot2camera static transform.

            std::vector<tf::Transform> local_cluster_poses_; //!> Stores the cluster poses relative to camera frame

            std::vector<tf::Transform> initial_cluster_pose_history_; //!> Stores the initial cluster poses, before graph update.

            std::vector<std::pair<int, int>> cluster_frame_relation_; //!> Stores the cluster/frame relation (cluster_id, frame_id)

            std::mutex mutex_graph_; //!> Mutex for the graph manipulation.

            g2o::SparseOptimizer graph_optimizer_; //!> G2O graph optimizer.

            LoopClosing* loop_closing_; // Loop closing object pointer.

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<slamon::Frame,
                                                                    nav_msgs::Odometry> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };


} // End namespace


#endif // GRAPH_HPP