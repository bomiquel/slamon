/**
 *  @file
 *  @brief Mapmaker class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024-2025
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

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "frame.hpp"
#include "cluster.hpp"
#include "publisher.hpp"
#include "loop_closer.hpp"

#include "slamon/GetTransform.h"


namespace slamon
{


    class LoopCloser;


    class Mapmaker: protected Publisher
    {
        public:


            /** \brief Class constructor
             *  \param[in] min_lc_inliers Minimum number of inliers for loop closure.
             *  \param[in] enable_debugging Enabling or disabling debugging.
             *  \param[in] min_dist_keyframe Minimum distance between keyframes.
             *  \param[in] w_path Working path for debugging purpose.
             *  \param[in] loop_closer Loop closer object pointer.
             *  \param[in] robot_frame_id Robot coordinate axis identficator (base link).
             */
            Mapmaker(int min_lc_inliers,
                     bool enable_debugging,
                     LoopCloser* loop_closer,
                     double min_dist_keyframe,
                     const std::string& w_path,
                     const std::string& robot_frame_id);


            /** \brief Starts mapmaker.
             */
            void run();


            /** \brief Get the frame id of some specific vertex.
             *  \param[in] vertex_id The vertex id.
             *  @return The frame id.
             */
            int getVertexKeyframeId(int vertex_id);


            /** \brief Get the world-map vertex pose from the graph.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  \param[in] lock Set to true if graph locking is required (optional).
             *  @return Tf with world-map vertex pose.
             */
            tf::Transform getVertexPose(int vertex_id, bool lock);


            /** \brief Get the graph vertex pose relative to camera.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  @return Tf with vertex pose relative to camera.
             */
            tf::Transform getVertexPoseRelativeToCamera(int vertex_id);


            /** \brief Get the world-map camera pose of some specific vertex.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  \param[in] lock Set to true if graph locking is required (optional).
             *  @return Tf with the world-map camera pose.
             */
            tf::Transform getVertexCameraPose(int vertex_id, bool lock);
            

            /** \brief Retrieve the list of the vertices/clusters of a corresponding keyframe.
             *  \param[in] keyframe_id The corresponding keyframe id.
             *  \param[out] vertices Will contain the list of vertices for this keyframe.
             */
            void getKeyframeVertices(int keyframe_id, 
                                     std::vector<int>& vertices);


            /** \brief Gets the absolute pose of a specific keyframe.
             *  \param[in] keyframe_id The id of the keyframe for which the pose is desired.
             *  @return Tf with the world-map camera pose.
             */
            tf::Transform getKeyframePose(int keyframe_id);


            /** \brief Gets the poses of each keyframe in the graph.
             *  \param[out] stamps The timestamp of each pose.
             *  \param[out] keyframe_ids A list containing the id of each keyframe.
             *  \param[out] keyframe_poses A list containing the poses of each keyframe.
             */
            void getKeyframePoses(std::vector<double>& stamps,
                                  std::vector<int>& keyframe_ids,
                                  std::vector<tf::Transform>& keyframe_poses);


            /** \brief Gets all loop closure edge information.
             *  \param[out] keyframe_ids A list of the identifiers of the keyframe pairs that close a loop.
             *  \param[out] keyframe_poses A list of the poses of the keyframe pairs that close a loop.
             */
            void getLoopClosureEdges(std::vector<std::pair<int, int>>& keyframe_ids,
                                     std::vector<std::pair<tf::Transform, tf::Transform>>& keyframe_poses);


            /** \brief Add an edge to the graph.
             *  \param[in] vertex_0_id Index of graph vertex 0.
             *  \param[in] vertex_1_id Index of graph vertex 1.
             *  \param[in] transformation Transformation between graph vertices.
             */
            void addEdge(int vertex_0_id, 
                         int vertex_1_id, 
                         tf::Transform tf_edge);
            

            /** \brief Gets nearest neighbour vertices by distance.
             *  \param[in] vertex_id The vertex id to retrieve its neighbors.
             *  \param[in] security_window Discarded number of vertices before the critical vertex.
             *  \param[in] critical_vertex_id Identifier of the vertex that the search cannot reach.
             *  \param[in] closest_neighbors_num The number of neighbors to be retrieved.
             *  \param[out] closest_neighbors Will contain the list of closest neighbors by distance.
             */
            void findClosestVertices(int vertex_id,
                                     int security_window,
                                     int critical_vertex_id,
                                     uint closest_neighbors_num,
                                     std::vector<int>& closest_neighbors);


            /** \brief Optimitzes the graph. 
             */
            void update();


        private:


            /** \brief Messages callback. This function is called when synchronized stereo
             *  data and odom messages are received.
             *  \param[in] odom_msg ROS message containing current robot odometry.
             *  \param[in] stereo_attributes_msg Custom ROS message containing current 
             *  stereo pair image relevant information.
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
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


            /** \brief Adds a vertex to the graph.
             *  \param[in] pose Vertex pose.
             *  @return the vertex id.
             */
            int addVertex(tf::Transform pose);


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


            /** \brief Convert a tf::Transform to Eigen::Isometry3d
             *  \param[in] tf The tf::Transform.
             *  @return The Eigen::Isometry3d matrix.
             */
            Eigen::Isometry3d fromTfToIsometry(const tf::Transform& tf);


            /** \brief Convert an Eigen::Isometry3d to tf::Transform.
             *  \param[in] iso The Eigen::Isometry3d matrix.
             *  @return The tf::Transform.
             */
            tf::Transform fromIsometryToTf(const Eigen::Isometry3d& iso);
            

            /** \brief Convert g2o::VertexSE3 to tf::Transform.
             *  \param[in] vertex The g2o::VertexSE3.
             *  @return The tf::Transform.
             */ 
            tf::Transform fromVertexToTf(const g2o::VertexSE3* vertex);


            /** \brief Computes the absolute diference between 2 poses (omitting z).
             *  \param[in] pose_0 transformation matrix of pose 0
             *  \param[in] pose_1 transformation matrix of pose 1
             *  @return the norm between two poses
             */
            static inline double poseDiff2D(const tf::Transform& pose_0, 
                                            const tf::Transform& pose_1)
            {
                tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
                return sqrt(d.x()*d.x() + d.y()*d.y());
            }


            /** \brief Computes the absolute diference between 2 poses.
             *  \param[in] pose_0 transformation matrix of pose 0
             *  \param[in] pose_1 transformation matrix of pose 1
             *  @return the norm between two poses
             */
            static inline double poseDiff3D(const tf::Transform& pose_0,
                                            const tf::Transform& pose_1)
            {
                tf::Vector3 d = pose_0.getOrigin() - pose_1.getOrigin();
                return sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
            }


            /** \brief Returns whether the second element of the first
             *  std::pair is less than the second element of the second std::pair
             *  \param[in] d_0 First pair.
             *  \param[in] d_1 Second pair.
             *  @return true if first pair is smaller than second pair.
             */
            static inline bool sortByDistance(const std::pair<int, double> d_0, const std::pair<int, double> d_1)
            {
                return (d_0.second < d_1.second);
            }


            // Atributes.
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

            std::vector<double> keyframe_stamps_; //> Stores the frame timestamps

            ros::Publisher robot_map_pub_; //!> Updated robot pose publisher.
            
            ros::Publisher camera_map_pub_; //!> Updated camera pose publisher.

            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher keyframes_num_pub_; // Publisher for the number of keyframes.

            ros::Publisher clustering_img_pub_; //!> Publisher for the frame clustering.

            ros::ServiceClient robot2camera_client_; //!> Service for requesting robot2camera static transform.

            std::vector<tf::Transform> local_cluster_poses_; //!> Stores the cluster poses relative to camera frame

            std::vector<tf::Transform> initial_cluster_pose_history_; //!> Stores the initial cluster poses, before graph update.

            std::vector<std::pair<int, int>> cluster_keyframe_relation_; //!> Stores the cluster-keyframe relation (cluster_id, frame_id).

            std::mutex mutex_graph_; //!> Mutex for the graph manipulation.

            g2o::SparseOptimizer graph_; //!> G2O graph.

            LoopCloser* loop_closer_; // Loop closer pointer.

            RunTimes run_times_; // !> Internal map execution times.


            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                                    slamon::StereoImageAttributes> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };

    
} // End namespace


#endif // MAPMAKER_HPP