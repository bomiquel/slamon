/**
 *  \file map.hpp
 *  \brief Topological pose-graph map wrapper around g2o, where vertices represent keypoint 
 *         clusters and edges represent relative 6-DoF transformations.
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


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{

    /** \class Map
     *  \brief Manages the global 3D pose-graph map using the g2o optimization framework.
     *  
     *  In this topological-metric map, graph vertices (`g2o::VertexSE3`) correspond to spatial 
     *  clusters of keypoints, and graph edges (`g2o::EdgeSE3`) represent 6-DoF relative motion 
     *  constraints weighted by the number of matched keypoint inliers. Thread-safe access is 
     *  provided to support concurrent visual odometry and loop closure operations.
     */
    class Map
    {


        public:


            /** \brief Default class constructor. Initializes the g2o sparse optimizer and solver.
             */
            Map();


            /** \brief Class destructor.
             */
            ~Map();


            /** \brief Gets the ID of the most recently added vertex in the graph.
             *  \return The last vertex ID.
             */
            int getCurrentVertexId();


            /** \brief Gets the keyframe ID associated with a specific graph vertex.
             *  \param[in] vertex_id Target vertex (cluster) ID.
             *  \return The corresponding keyframe ID.
             */
            int getVertexKeyframeId(const int vertex_id);


            /** \brief Gets the world-map pose of a specific vertex from the graph.
             *  \param[in] vertex_id ID of the vertex whose pose is requested.
             *  \param[in] use_lock Set to `true` if mutex locking of the graph is required.
             *  \return Transform representing the world-map vertex pose.
             */
            tf2::Transform getVertexPose(const int vertex_id, 
                                         const bool use_lock);


            /** \brief Gets the world-map poses, IDs, and timestamps of all vertices in the graph.
             *  \param[out] ids Output vector of vertex IDs.
             *  \param[out] stamps Output vector containing the timestamp of each vertex pose.
             *  \param[out] vertex_poses Output vector containing the world-map pose of each vertex.
             */
            void getVertexPoses(std::vector<int>& ids,
                                std::vector<double>& stamps,
                                std::vector<tf2::Transform>& vertex_poses);


            /** \brief Gets the local pose of a graph vertex relative to its corresponding left camera frame.
             *  \param[in] vertex_id ID of the vertex whose local pose is requested.
             *  \return Transform representing the vertex pose relative to the camera frame.
             */
            tf2::Transform getVertexPoseRelativeToCamera(const int vertex_id);


            /** \brief Computes the world-map camera pose associated with a specific graph vertex.
             *  \param[in] vertex_id ID of the vertex for which the camera pose is requested.
             *  \param[in] use_lock Set to `true` if mutex locking of the graph is required.
             *  \return Transform representing the world-map camera pose.
             */
            tf2::Transform getVertexCameraPose(const int vertex_id, 
                                               const bool use_lock);


            /** \brief Retrieves the list of vertex (cluster) IDs belonging to a specific keyframe.
             *  \param[in] keyframe_id Target keyframe ID.
             *  \param[out] vertices Output vector of vertex IDs associated with the keyframe.
             */
            void getKeyframeVertices(const int keyframe_id, 
                                     std::vector<int>& vertices);


            /** \brief Gets the world-map camera pose of a specific keyframe.
             *  \param[in] keyframe_id ID of the keyframe whose pose is requested.
             *  \return Transform representing the world-map keyframe camera pose.
             */
            tf2::Transform getKeyframePose(const int keyframe_id);


            /** \brief Gets the world-map poses and timestamps of all keyframes in the graph.
             *  \param[out] stamps Output vector containing the timestamp of each keyframe.
             *  \param[out] keyframe_poses Output vector containing the world-map pose of each keyframe.
             */
            void getKeyframePoses(std::vector<double>& stamps,
                                  std::vector<tf2::Transform>& keyframe_poses);


            /** \brief Retrieves information for all loop closure edges in the pose graph.
             *  \param[out] weights Output list of scalar weights reflecting the confidence of each loop closure edge.
             *  \param[out] keyframe_ids Output list of keyframe ID pairs connected by a loop closure.
             *  \param[out] keyframe_poses Output list of world-map pose pairs for the keyframes connected by a loop closure.
             */
            void getLoopClosureEdges(std::vector<double>& weights,
                                     std::vector<std::pair<int, int>>& keyframe_ids,
                                     std::vector<std::pair<tf2::Transform, tf2::Transform>>& keyframe_poses);


            /** \brief Finds the nearest neighbor vertices to a query vertex based on Euclidean distance.
             *  \param[in] q_vertex_id Query vertex ID for which neighbors are searched.
             *  \param[in] security_window Number of vertices immediately preceding the critical vertex to discard.
             *  \param[in] critical_vertex_id Upper bound vertex ID that the neighbor search cannot exceed.
             *  \param[in] closest_neighbors_num Maximum number of nearest neighbors to retrieve.
             *  \param[out] closest_neighbors Output vector of the closest neighbor vertex IDs sorted by distance.
             */
            void findClosestVertices(const int q_vertex_id,
                                     const int security_window,
                                     const int critical_vertex_id,
                                     const int closest_neighbors_num,
                                     std::vector<int>& closest_neighbors);


            /** \brief Appends a new keyframe timestamp to the internal container.
             *  \param[in] stamp Keyframe capture timestamp.
             */
            void addTimestamp(const double stamp);


            /** \brief Appends a local cluster pose (relative to the camera frame) to the internal container.
             *  \param[in] local_cluster_pose Cluster pose relative to the left camera coordinate frame.
             */
            void addLocalClusterPose(const tf2::Transform& local_cluster_pose);


            /** \brief Appends a cluster-to-keyframe association pair to the internal container.
             *  \param[in] relation Pair mapping a global cluster ID to its corresponding keyframe ID (`cluster_id`, `keyframe_id`).
             */
            void addClusterKeyframeRelation(const std::pair<int, int>& relation);


            /** \brief Adds a new SE(3) vertex to the pose graph.
             *  \param[in] id Unique vertex (cluster) ID.
             *  \param[in] pose Initial world-map 6-DoF pose of the vertex.
             */
            void addVertex(const int id,
                           const tf2::Transform& pose);


            /** \brief Adds a new SE(3) constraint edge between two vertices in the pose graph.
             *  \param[in] vertex_0_id ID of the first vertex connected by the edge.
             *  \param[in] vertex_1_id ID of the second vertex connected by the edge.
             *  \param[in] confidence Scalar weight/confidence of the edge constraint (information matrix scale).
             *  \param[in] relative_movement Relative 6-DoF transformation between the two vertices.
             */
            void addEdge(const int vertex_0_id, 
                         const int vertex_1_id,
                         const double confidence,
                         const tf2::Transform& relative_movement);


            /** \brief Executes global Levenberg-Marquardt Pose-Graph Optimization on the g2o graph.
             */
            void optimiseGraphPoses();


        private:

                
            /** \brief Comparator function to sort pairs of (vertex ID, Euclidean distance) in ascending order of distance.
             *  \param[in] d_0 First pair (ID, distance).
             *  \param[in] d_1 Second pair (ID, distance).
             *  \return `true` if the distance of `d_0` is strictly less than that of `d_1`, `false` otherwise.
             */
            static inline bool sortByDistance(const std::pair<int, double>& d_0, const std::pair<int, double>& d_1)
            {
                return (d_0.second < d_1.second);
            }

            
            /** \brief Converts a `tf2::Transform` into an `Eigen::Isometry3d` matrix.
             *  \param[in] tf Input `tf2::Transform`.
             *  \return Equivalent `Eigen::Isometry3d` transformation matrix.
             */
            Eigen::Isometry3d fromTfToIsometry(const tf2::Transform& tf);


            /** \brief Converts an `Eigen::Isometry3d` matrix into a `tf2::Transform`.
             *  \param[in] iso Input `Eigen::Isometry3d` transformation matrix.
             *  \return Equivalent `tf2::Transform`.
             */
            tf2::Transform fromIsometryToTf(const Eigen::Isometry3d& iso);


            /** \brief Extracts the 6-DoF pose of a `g2o::VertexSE3` as a `tf2::Transform`.
             *  \param[in] vertex Pointer to the input `g2o::VertexSE3`.
             *  \return Equivalent `tf2::Transform`.
             */  
            tf2::Transform fromVertexToTf(const g2o::VertexSE3* vertex);


            /** \brief Sorts graph vertices by index in ascending order.
             *  \return Sorted graph vertices.
             */
            std::vector<std::pair<int, g2o::VertexSE3*>> sortVertices();

            
            /** \brief Mutex protecting concurrent access to the g2o pose graph. */
            std::mutex mutex_graph_;

            /** \brief Mutex protecting concurrent access to the keyframe timestamps container. */
            std::mutex mutex_keyframe_stamps_;

            /** \brief Mutex protecting concurrent access to the local cluster poses container. */
            std::mutex mutex_local_cluster_poses_;

            /** \brief Mutex protecting concurrent access to the cluster-keyframe relations container. */
            std::mutex mutex_cluster_keyframe_relations_;

            /** \brief Instance of the g2o sparse optimizer representing the pose graph. */
            g2o::SparseOptimizer graph_;

            /** \brief Container storing the capture timestamps of all keyframes. */
            std::vector<double> keyframe_stamps_;

            /** \brief Container storing the local cluster poses relative to their respective left camera frames. */
            std::vector<tf2::Transform> local_cluster_poses_;

            /** \brief Container storing the cluster-to-keyframe associations (`cluster_id`, `keyframe_id`). */
            std::vector<std::pair<int, int>> cluster_keyframe_relations_;


    };


} // namespace slamon


#endif // MAP_HPP