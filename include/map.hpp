/**
 *  @file map.hpp
 *  @brief Map class based on a topological map where nodes are clusters of keypoints
 *         and edges are the relative movement between them and the number of keypoint 
 *         matchings. g2o graph wrapper.
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

 
#ifndef MAP_HPP
#define MAP_HPP


#include <unordered_set>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "tools.hpp"


namespace slamon
{


    class Map
    {


        public:


            /** \brief Class constructor.
             */
            Map();


            /** \brief Class destructor.
             */
            ~Map();


            /** \brief Gets last vertex id.
             *  @return Last vertex id.
             */
            int getCurrentVertexId();


            /** \brief Get the frame id of some specific vertex.
             *  \param[in] vertex_id The vertex id.
             *  @return The frame id.
             */
            int getVertexKeyframeId(const int vertex_id);


            /** \brief Get the world-map vertex pose from the graph.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  \param[in] use_lock Set to true if map locking is required
             *  @return Tf with world-map vertex pose.
             */
            tf2::Transform getVertexPose(const int vertex_id, 
                                         const bool use_lock);


            /** \brief Gets the poses of each vertex in the graph.
             *  \param[out] ids A vertex identifier list.
             *  \param[out] stamps The timestamp of each pose.
             *  \param[out] vertex_poses A list containing the poses of each vertex.
             */
            void getVertexPoses(std::vector<int>& ids,
                                std::vector<double>& stamps,
                                std::vector<tf2::Transform>& vertex_poses);


            /** \brief Get the graph vertex pose relative to camera.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  @return Tf with vertex pose relative to camera.
             */
            tf2::Transform getVertexPoseRelativeToCamera(const int vertex_id);


            /** \brief Get the world-map camera pose of some specific vertex.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  \param[in] lock Set to true if map locking is required.
             *  @return Tf with the world-map camera pose.
             */
            tf2::Transform getVertexCameraPose(const int vertex_id, 
                                               const bool use_lock);


            /** \brief Retrieve the list of the vertices/clusters of a corresponding keyframe.
             *  \param[in] keyframe_id The corresponding keyframe id.
             *  \param[out] vertices Will contain the list of vertices for this keyframe.
             */
            void getKeyframeVertices(const int keyframe_id, 
                                     std::vector<int>& vertices);


            /** \brief Gets the absolute pose of a specific keyframe.
             *  \param[in] keyframe_id The id of the keyframe for which the pose is desired.
             *  @return Tf with the world-map camera pose.
             */
            tf2::Transform getKeyframePose(const int keyframe_id);


            /** \brief Gets the poses of each keyframe in the graph.
             *  \param[out] stamps The timestamp of each pose.
             *  \param[out] keyframe_poses A list containing the poses of each keyframe.
             */
            void getKeyframePoses(std::vector<double>& stamps,
                                  std::vector<tf2::Transform>& keyframe_poses);


            /** \brief Gets all loop closure edge information.
             *  \param[out] weights A simple scalar list reflecting the total confidence in all dimensions of each graph edge.
             *  \param[out] keyframe_ids A list of the identifiers of the keyframe pairs that close a loop.
             *  \param[out] keyframe_poses A list of the poses of the keyframe pairs that close a loop.
             */
            void getLoopClosureEdges(std::vector<double>& weights,
                                     std::vector<std::pair<int, int>>& keyframe_ids,
                                     std::vector<std::pair<tf2::Transform, tf2::Transform>>& keyframe_poses);


            /** \brief Gets nearest neighbour vertices by distance.
             *  \param[in] q_vertex_id The vertex id to retrieve its neighbors.
             *  \param[in] security_window Discarded number of vertices before the critical vertex.
             *  \param[in] critical_vertex_id Identifier of the vertex that the search cannot reach.
             *  \param[in] closest_neighbors_num The number of neighbors to be retrieved.
             *  \param[out] closest_neighbors Will contain the list of closest neighbors by distance.
             */
            void findClosestVertices(const int q_vertex_id,
                                     const int security_window,
                                     const int critical_vertex_id,
                                     const int closest_neighbors_num,
                                     std::vector<int>& closest_neighbors);


            /** \brief Adds keyframe timestamp to the vector.
             *  \param[in] stamp Keyframe timestamp.
             */
            void addTimestamp(const double stamp);


            /** \brief Adds local cluster pose to the vector.
             *  \param[in] local_cluster_pose Cluster pose relative to camera frame.
             */
            void addLocalClusterPose(const tf2::Transform& local_cluster_pose);


            /** \brief Adds cluster-keyframe relation.
             *  \param[in] relation Cluster-keyframe relation.
             */
            void addClusterKeyframeRelation(const std::pair<int, int>& relation);


            /** \brief Adds a vertex to the graph.
             *  \param[in] id Vertex identifier.
             *  \param[in] pose Vertex pose.
             */
            void addVertex(const int id,
                           const tf2::Transform& pose);


            /** \brief Adds an edge to the graph.
             *  \param[in] vertex_0_id Index of one of the two vertices of the graph joined by the edge.
             *  \param[in] vertex_1_id Index of the other vertex of the graph joined by the edge.
             *  \param[in] confidence The confidence in the edge.
             *  \param[in] relative_movement Coordinate transformation between graph vertices.
             */
            void addEdge(const int vertex_0_id, 
                         const int vertex_1_id,
                         const double confidence,
                         const tf2::Transform& relative_movement);


            /** \brief Performs a Pose-Graph Optimisation.
             */
            void optimiseGraphPoses();


        private:

                
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

            
            /** \brief Convert a tf2::Transform to Eigen::Isometry3d
             *  \param[in] tf The tf2::Transform.
             *  @return The Eigen::Isometry3d matrix.
             */
            Eigen::Isometry3d fromTfToIsometry(const tf2::Transform& tf);


            /** \brief Convert an Eigen::Isometry3d to tf2::Transform.
             *  \param[in] iso The Eigen::Isometry3d matrix.
             *  @return The tf2::Transform.
             */
            tf2::Transform fromIsometryToTf(const Eigen::Isometry3d& iso);


            /** \brief Convert g2o::VertexSE3 to tf2::Transform.
             *  \param[in] vertex The g2o::VertexSE3.
             *  @return The tf2::Transform.
             */ 
            tf2::Transform fromVertexToTf(const g2o::VertexSE3* vertex);


            /** \brief Sorts graph vertices by index in ascending order.
             *  @return Sorted graph vertices.
             */
            std::vector<std::pair<int, g2o::VertexSE3*>> sortVertices();


            // Attributes:
            
            std::mutex mutex_graph_; //!> Mutex for graph manipulation.

            std::mutex mutex_keyframe_stamps_; //!> Mutex for the timestamps container.

            std::mutex mutex_local_cluster_poses_; //!> Mutex for the cluster poses, relative to camera frame, container.

            std::mutex mutex_cluster_keyframe_relations_; //!> Mutex for the cluster-keyframe relations container.

            g2o::SparseOptimizer graph_; //!> G2O graph.

            std::vector<double> keyframe_stamps_; //> Stores the keyframe timestamps.

            std::vector<tf2::Transform> local_cluster_poses_; //!> Stores the cluster poses relative to camera frame.

            std::vector<std::pair<int, int>> cluster_keyframe_relations_; //!> Stores the cluster-keyframe relations (cluster_id, frame_id).


    };


} // End namespace


#endif // MAP_HPP