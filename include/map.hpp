/**
 *  @file
 *  @brief Map class. g2o graph wrapper.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2024-2025
 */

#ifndef MAP_HPP
#define MAP_HPP


#include <ros/ros.h>
#include <tf/tf.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "tools.hpp"


namespace slamon
{


    class Map
    {
        public:


            /** \brief Class constructor
             */
            Map();


            /** \brief Gets last vertex id.
             *  @return Last vertex id.
             */
            int getCurrentVertexId();


            /** \brief Get the frame id of some specific vertex.
             *  \param[in] vertex_id The vertex id.
             *  \param[in] lock Set to true if map locking is required.
             *  @return The frame id.
             */
            int getVertexKeyframeId(int vertex_id, bool lock);


            /** \brief Get the world-map vertex pose from the graph.
             *  \param[in] vertex_id The id of the vertex for which the pose is desired.
             *  \param[in] lock Set to true if map locking is required
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
             *  \param[in] lock Set to true if map locking is required.
             *  @return Tf with the world-map camera pose.
             */
            tf::Transform getVertexCameraPose(int vertex_id, bool lock);


            /** \brief Retrieve the list of the vertices/clusters of a corresponding keyframe.
             *  \param[in] keyframe_id The corresponding keyframe id.
             *  \param[in] lock Set to true if map locking is required.
             *  \param[out] vertices Will contain the list of vertices for this keyframe.
             */
            void getKeyframeVertices(int keyframe_id, 
                                     bool lock,
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


            /** \brief Adds keyframe timestamp to the vector.
             *  \param[in] stamp Keyframe timestamp.
             */
            void addTimestamp(double stamp);


            /** \brief Adds local cluster pose to the vector.
             *  \param[in] local_cluster_pose Cluster pose relative to camera frame.
             */
            void addLocalClusterPose(const tf::Transform& local_cluster_pose);


            /** \brief Adds cluster-keyframe relation.
             *  \param[in] relation Cluster-keyframe relation.
             */
            void addClusterKeyframeRelation(const std::pair<int, int>& relation);


            /** \brief Adds a vertex to the graph.
             *  \param[in] pose Vertex pose.
             *  @return the vertex id.
             */
            int addVertex(const tf::Transform& pose);


            /** \brief Add an edge to the graph.
             *  \param[in] vertex_0_id Index of graph vertex 0.
             *  \param[in] vertex_1_id Index of graph vertex 1.
             *  \param[in] transformation Transformation between graph vertices.
             */
            void addEdge(int vertex_0_id, 
                         int vertex_1_id, 
                         tf::Transform tf_edge);


            /** \brief Optimitzes the graph. 
             */
            void update();


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


            // Attributes:
            
            std::mutex mutex_map_; //!> Mutex for map manipulation.

            g2o::SparseOptimizer graph_; //!> G2O graph.

            std::vector<double> keyframe_stamps_; //> Stores the keyframe timestamps

            std::vector<tf::Transform> local_cluster_poses_; //!> Stores the cluster poses relative to camera frame

            std::vector<std::pair<int, int>> cluster_keyframe_relation_; //!> Stores the cluster-keyframe relation (cluster_id, frame_id).


    };


} // End namespace


#endif // MAP_HPP