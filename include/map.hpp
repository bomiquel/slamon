/**
 *  @file
 *  @brief Map class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef MAP_HPP
#define MAP_HPP


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


    class Map: protected Publisher
    {
        public:


            /** \brief Class constructor
             *  \param min_lc_inliers Minimum number of inliers for loop closure.
             *  \param enable_debugging Enabling or disabling debugging.
             *  \param min_dist_keyframe Minimum distance between keyframes.
             *  \param w_path Working path for debugging purpose.
             *  \param loop_closer Loop closer object pointer.
             *  \param robot_frame_id Robot coordinate axis identficator (base link).
             */
            Map(int min_lc_inliers,
                bool enable_debugging,
                LoopCloser* loop_closer,
                double min_dist_keyframe,
                const std::string& w_path,
                const std::string& robot_frame_id);


            /** \brief Starts map.
             */
            void run();


            /** \brief Get the frame id of some specific vertex.
             *  \param vertex_id The vertex id (input).
             *  @return The frame id.
             */
            int getVertexKeyframeId(int vertex_id);


            /** \brief Get the world vertex pose from the graph.
             *  \param vertex_id The id of the vertex for which the pose is desired (input).
             *  \param lock Set to true if graph locking is required (input/optional).
             *  @return Tf with world vertex pose.
             */
            tf::Transform getVertexPose(int vertex_id, bool lock);


            /** \brief Get the graph vertex pose relative to camera.
             *  \param vertex_id The id of the vertex for which the pose is desired (input).
             *  @return Tf with vertex pose relative to camera.
             */
            tf::Transform getVertexPoseRelativeToCamera(int vertex_id);


            /** \brief Get the world camera pose of some specific vertex.
             *  \param vertex_id The id of the vertex for which the pose is desired (input).
             *  \param lock Set to true if graph locking is required (input/optional).
             *  @return Tf with the world camera pose.
             */
            tf::Transform getVertexCameraPose(int vertex_id, bool lock);
            

            /** \brief Retrieve the list of the vertices/clusters of a corresponding keyframe.
             *  \param keyframe_id The corresponding keyframe id (input).
             *  \param vertices Will contain the list of vertices for this keyframe (output).
             */
            void getKeyframeVertices(int keyframe_id, 
                                     std::vector<int>& vertices);


            /** \brief Gets the absolute pose of a specific keyframe.
             *  \param keyframe_id The id of the keyframe for which the pose is desired (input).
             *  @return Tf with the world camera pose.
             */
            tf::Transform getKeyframePose(int keyframe_id);


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
                         tf::Transform tf_edge);
            

            /** \brief Gets nearest neighbour vertices by distance.
             *  \param vertex_id The vertex id to retrieve its neighbors (input).
             *  \param security_window Discarded number of vertices before the critical vertex (input).
             *  \param critical_vertex_id Identifier of the vertex that the search cannot reach (input).
             *  \param closest_neighbors_num The number of neighbors to be retrieved (input).
             *  \param closest_neighbors Will contain the list of closest neighbors by distance (output).
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
             *  \param odom_msg ROS message containing current robot odometry (input).
             *  \param stereo_data_msg Custom ROS message containing current stereo pair image
             *  relevant information (input).
             */  
            void callback(const nav_msgs::OdometryConstPtr& odom_msg,
                          const slamon::StereoDataConstPtr& stereo_data_msg);


            /** \brief Decides if new keyframe is needed.
             *  \param c_robot_odom Current robot odom (input).
             *  \param c_frame Current frame (input/output).
             *  @return True if new keyframe will be inserted into the graph.
             */
            bool needNewKeyFrame(const tf::Transform& c_robot_odom,
                                 Frame& c_frame);


            /** \brief Adds a frame to the graph if enough inliers
             *  \param c_frame Current frame (input).
             *  @return True if new keyframe will be inserted into the map
             */
            bool addFrameToMap(Frame& c_frame);


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
            

            /** \brief Convert custom ROS stereo data msg to frame object (slam::Frame)
             *  \param msg ROS message with relevant visual information (input).
             *  \param frame Will contain the relevant visual information (output).
             */
            void fromStereoDataToFrame(const slamon::StereoDataConstPtr& msg,
                                       Frame& frame);


            /** \brief Convert cv::Point3f to tf::Transform.
             *  \param pt 3D point (input).
             *  @return 3D point converted into a transform.
             */
            tf::Transform fromPoint3DToTf(const cv::Point3f& pt);


            /** \brief Convert a tf::Transform to Eigen::Isometry3d
             *  \param tf The tf::Transform (input).
             *  @return The Eigen::Isometry3d matrix.
             */
            Eigen::Isometry3d fromTfToIsometry(const tf::Transform& tf);


            /** \brief Convert an Eigen::Isometry3d to tf::Transform.
             *  \param iso The Eigen::Isometry3d matrix (input).
             *  @return The tf::Transform.
             */
            tf::Transform fromIsometryToTf(const Eigen::Isometry3d& iso);
            

            /** \brief Convert g2o::VertexSE3 to tf::Transform.
             *  \param vertex The g2o::VertexSE3 (input).
             *  @return The tf::Transform.
             */ 
            tf::Transform fromVertexToTf(const g2o::VertexSE3* vertex);


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


            // Atributes.
            bool init_map_; //!> Map state.

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

            std::string camera_frame_id_; //!> Camera coordinate axis identficator.

            tf::Transform p_robot_odom_; //!> The robot's odometry the last time a keyframe was inserted.

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
                                                                    slamon::StereoData> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };

    
} // End namespace


#endif // MAP_HPP