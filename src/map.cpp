/**
 *  \file map.cpp
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

 
#include "map.hpp"


namespace slamon
{


    Map::Map()
    {
        // Inilitialises the g2o graph optimiser.
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr (new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>());
        std::unique_ptr<g2o::BlockSolverX> solver_ptr (new g2o::BlockSolverX(std::move(linear_solver_ptr)));
        g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
        graph_.setAlgorithm(solver);
    }


    Map::~Map()
    {
        graph_.clear();
        graph_.clearParameters();
        keyframe_stamps_.clear();
        local_cluster_poses_.clear();
        cluster_keyframe_relations_.clear();
    }


    int Map::getCurrentVertexId()
    {
        std::scoped_lock lock(mutex_graph_);
        return graph_.vertices().size() - 1;
    }


    int Map::getVertexKeyframeId(const int vertex_id)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relations_);

        int frame_id = -1;
        for (size_t i = 0; i < cluster_keyframe_relations_.size(); ++i)
        {
            if (cluster_keyframe_relations_[i].first == vertex_id)
            {
                frame_id = cluster_keyframe_relations_[i].second;
                break;
            }
        }
        return frame_id;
    }


    tf2::Transform Map::getVertexPose(const int vertex_id, 
                                      const bool use_lock)
    {
        // Inits variables.
        g2o::VertexSE3* vertex;
        tf2::Transform vertex_pose;
        vertex_pose.setIdentity();

        // Sanity check.
        if (vertex_id < 0)
            return vertex_pose;

        // Gets vertex.
        if (use_lock)
        {
            std::scoped_lock lock(mutex_graph_);
            vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
        }
        else
        {
            vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
        }

        // Sanity check 2. 
        if (vertex)
            vertex_pose = fromVertexToTf(vertex);
        else
            ROS_WARN_STREAM("[Map->getVertexPose:] Problems getting vertex: " << vertex_id << "! Returning identity!");

        return vertex_pose;
    }


    void Map::getVertexPoses(std::vector<int>& ids,
                             std::vector<double>& stamps,
                             std::vector<tf2::Transform>& vertex_poses)
    {
        // Sanity check
        ids.clear();
        stamps.clear();
        vertex_poses.clear();

        // Sorts graph vertices.
        std::vector<std::pair<int, g2o::VertexSE3*>> vertices = sortVertices();

        // Reserves.
        ids.reserve(vertices.size());
        stamps.reserve(vertices.size());
        vertex_poses.reserve(vertices.size());

        // Get keyframe poses
        int keyframe_id;
        for (std::vector<std::pair<int, g2o::VertexSE3*>>::iterator it = vertices.begin(); it != vertices.end(); ++it)
        {
            // The keyframe identifier is needed to obtain the timestamp.
            keyframe_id = getVertexKeyframeId(it->first);
            if (keyframe_id < 0)
                continue;

            // Save data.
            ids.emplace_back(it->first);
            {
                std::scoped_lock lock(mutex_keyframe_stamps_);
                stamps.emplace_back(keyframe_stamps_[keyframe_id]);
            }
            vertex_poses.emplace_back(fromIsometryToTf(it->second->estimate()));
        }
    }


    tf2::Transform Map::getVertexPoseRelativeToCamera(const int vertex_id)
    {
        std::scoped_lock lock(mutex_local_cluster_poses_);
        return local_cluster_poses_[vertex_id];
    }


    tf2::Transform Map::getVertexCameraPose(const int vertex_id, 
                                            const bool use_lock)
    {
        tf2::Transform vertex_pose = getVertexPose(vertex_id, use_lock);

        std::scoped_lock lock(mutex_local_cluster_poses_);
        return vertex_pose * local_cluster_poses_[vertex_id].inverse();
    }


    void Map::getKeyframeVertices(const int keyframe_id,
                                  std::vector<int>& vertices)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relations_);

        vertices.clear();
        for (size_t i = 0; i < cluster_keyframe_relations_.size(); ++i)
        {
            if (cluster_keyframe_relations_[i].second == keyframe_id)
                vertices.push_back(cluster_keyframe_relations_[i].first);
        }
    }


    tf2::Transform Map::getKeyframePose(const int keyframe_id)
    {
        std::vector<int> keyframe_vertices;
        getKeyframeVertices(keyframe_id, keyframe_vertices);
        return getVertexCameraPose(keyframe_vertices[0], true);
    }


    void Map::getKeyframePoses(std::vector<double>& stamps,
                               std::vector<tf2::Transform>& keyframe_poses)
    {
        // Sanity check
        stamps.clear();
        keyframe_poses.clear();

        // Sorts graph vertices.
        std::vector<std::pair<int, g2o::VertexSE3*>> vertices = sortVertices();

        // Get keyframe poses.
        int vertex_id, keyframe_id;
        std::unordered_set<int> processed_keyframe_ids;
        for (std::vector<std::pair<int, g2o::VertexSE3*>>::iterator it = vertices.begin(); it != vertices.end(); ++it)
        {
            vertex_id = it->first;

            // Is this keyframe processed?
            keyframe_id = getVertexKeyframeId(vertex_id);
            if (keyframe_id < 0)
                continue;
            if (processed_keyframe_ids.find(keyframe_id) != processed_keyframe_ids.end())
                continue;

            // Save data.
            {
                std::scoped_lock lock(mutex_keyframe_stamps_);
                stamps.emplace_back(keyframe_stamps_[keyframe_id]);
            }
            keyframe_poses.emplace_back(getVertexCameraPose(vertex_id, true));

            // Marks this keyframe as processed.
            processed_keyframe_ids.insert(keyframe_id);
        }
    }


    void Map::getLoopClosureEdges(std::vector<double>& weights,
                                  std::vector<std::pair<int, int>>& keyframe_ids,
                                  std::vector<std::pair<tf2::Transform, tf2::Transform>>& keyframe_poses)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Sanity check
        weights.clear();
        keyframe_ids.clear();
        keyframe_poses.clear();

        // Gets loop closure edges.
        g2o::EdgeSE3* edge;
        int keyframe_0, keyframe_1;
        tf2::Transform pose_0, pose_1;
        for (g2o::OptimizableGraph::EdgeSet::iterator it = graph_.edges().begin(); it != graph_.edges().end(); ++it)
        {
            edge = dynamic_cast<g2o::EdgeSE3*> (*it);
            if (!edge)
            {
                ROS_WARN_STREAM("[Map->getLoopClosureEdges:] Problems casting edge!");
                continue;
            }

            // Get the keyframes corresponding to this edge.
            keyframe_0 = getVertexKeyframeId(edge->vertices()[0]->id());
            keyframe_1 = getVertexKeyframeId(edge->vertices()[1]->id());

            if (std::abs(keyframe_0 - keyframe_1) <= 1)
                continue;

            // Get the keyframes camera pose.
            pose_0 = getVertexCameraPose(edge->vertices()[0]->id(), false);
            pose_1 = getVertexCameraPose(edge->vertices()[1]->id(), false);

            // Save data.
            weights.push_back(edge->information().trace()); // Get the confidence from this edge.
            keyframe_ids.emplace_back(keyframe_0, keyframe_1);
            keyframe_poses.emplace_back(pose_0, pose_1);
        }
    }


    void Map::findClosestVertices(const int q_vertex_id,
                                  const int security_window,
                                  const int critical_vertex_id,
                                  const int closest_neighbors_num,
                                  std::vector<int>& closest_neighbors)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Sanity check.
        closest_neighbors.clear();

        // Gets query vertex pose.
        tf2::Transform q_vertex_pose = getVertexPose(q_vertex_id, false);

        // Loop thought all the other nodes.
        double dist;
        int vertex_id;
        g2o::VertexSE3* vertex;
        tf2::Transform neighbor_vertex_pose;
        std::vector<std::pair<int, double>> neighbors;
        const int lower_limit = critical_vertex_id - security_window;
        const int upper_limit = critical_vertex_id + security_window;
        for (const std::pair<const int, g2o::HyperGraph::Vertex*>& it : graph_.vertices())
        {
            vertex_id = it.first;
            vertex = dynamic_cast<g2o::VertexSE3*>(it.second);
            if (!vertex)
            {
                ROS_WARN_STREAM("[Map->findClosestVertices:] Problems casting vertex: " << vertex_id << "!");
                continue;
            }

            // Avoids itself.
            if (vertex_id == q_vertex_id)
                continue;

            // Avoids if it is inside security window.
            if (vertex_id >= lower_limit && vertex_id <= upper_limit)
                continue;

            // Gets the distance between the queried vertex and the ith vertex.
            neighbor_vertex_pose = getVertexPose(vertex_id, false);
            dist = poseDiff2D(neighbor_vertex_pose, q_vertex_pose);
            neighbors.emplace_back(vertex_id, dist);
        }

        // Exits if no neighbors.
        if (neighbors.empty())
            return;

        // Sorts the neighbours from least to furthest distance to the query neighbour.
        std::sort(neighbors.begin(), neighbors.end(), sortByDistance);

        // Gets closest vertices (minnor distance)
        const int final_closest_neighbors_num = std::min(closest_neighbors_num, static_cast<int>(neighbors.size()));
        closest_neighbors.reserve(final_closest_neighbors_num);
        for (int i = 0; i < final_closest_neighbors_num; ++i)
            closest_neighbors.emplace_back(neighbors[i].first);
    }


    void Map::addTimestamp(const double stamp)
    {
        std::scoped_lock lock(mutex_keyframe_stamps_);
        keyframe_stamps_.push_back(stamp);
    }


    void Map::addLocalClusterPose(const tf2::Transform& local_cluster_pose)
    {
        std::scoped_lock lock(mutex_local_cluster_poses_);
        local_cluster_poses_.emplace_back(local_cluster_pose);
    }


    void Map::addClusterKeyframeRelation(const std::pair<int, int>& relation)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relations_);
        cluster_keyframe_relations_.emplace_back(relation);
    }


    void Map::addVertex(const int id,
                        const tf2::Transform& pose)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Convert pose for graph.
        Eigen::Isometry3d vertex_pose = fromTfToIsometry(pose);

        // Build the vertex.
        g2o::VertexSE3* c_vertex = new g2o::VertexSE3();
        c_vertex->setId(id);
        c_vertex->setEstimate(vertex_pose);
        
        // First time, no edges.
        if (id == 0)
            c_vertex->setFixed(true);

        // Add vertex.
        graph_.addVertex(c_vertex);
    }


    void Map::addEdge(const int vertex_0_id,
                      const int vertex_1_id,
                      const double confidence,
                      const tf2::Transform& relative_movement)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Gets the graph vertices.
        g2o::VertexSE3* vertex_0 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_0_id]);
        if (!vertex_0)
        {
            ROS_WARN_STREAM("[Map->addEdge:] Problems casting vertex: " << vertex_0_id << "!");
            return;
        }
        g2o::VertexSE3* vertex_1 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_1_id]);
        if (!vertex_1)
        {
            ROS_WARN_STREAM("[Map->addEdge:] Problems casting vertex: " << vertex_1_id << "!");
            return;
        }

        // Adds edge to graph.
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->setVertex(0, vertex_0);
        edge->setVertex(1, vertex_1);
        edge->setMeasurement(fromTfToIsometry(relative_movement));
        edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * confidence);

        graph_.addEdge(edge);
    }


    void Map::optimiseGraphPoses()
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Optimize!
        graph_.initializeOptimization();
        graph_.optimize(20);

        ROS_INFO_STREAM("[Map->optimiseGraphPoses:] PGO done in graph with: " << graph_.vertices().size() << " vertices.");
    }


    Eigen::Isometry3d Map::fromTfToIsometry(const tf2::Transform& tf)
    {
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        // Translation.
        const tf2::Vector3& t = tf.getOrigin();
        out.translation() << t.x(), t.y(), t.z();
        // Rotation.
        const tf2::Quaternion& q = tf.getRotation();
        Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z()); // (w, x, y, z)!!!!
        out.linear() = q_eigen.toRotationMatrix();
        return out;
    }


    tf2::Transform Map::fromIsometryToTf(const Eigen::Isometry3d& iso)
    {
        // Translation.
        const Eigen::Vector3d& t = iso.translation();
        tf2::Vector3 t_tf(t.x(), t.y(), t.z());
        // Rotation.
        Eigen::Quaterniond q(iso.rotation()); 
        tf2::Quaternion q_tf(q.x(), q.y(), q.z(), q.w()); // (x, y, z, w)!!!!
        return tf2::Transform(q_tf, t_tf);
    }


    tf2::Transform Map::fromVertexToTf(const g2o::VertexSE3* vertex)
    {
        Eigen::Isometry3d iso = vertex->estimate();
        tf2::Transform tf = fromIsometryToTf(iso);
        return tf;
    }


    std::vector<std::pair<int, g2o::VertexSE3*>> Map::sortVertices()
    {
        // Locks graph.
        std::scoped_lock lock(mutex_graph_);

        // Gets vertex id.
        int vertex_id;
        g2o::VertexSE3* vertex;
        std::vector<std::pair<int, g2o::VertexSE3*>> vertices;
        vertices.reserve(graph_.vertices().size());
        for (const std::pair<const int, g2o::HyperGraph::Vertex*>& it : graph_.vertices())
        {
            vertex_id = it.first;
            vertex = dynamic_cast<g2o::VertexSE3*>(it.second);
            if (!vertex)
            {
                ROS_WARN_STREAM("[Map->sortVertices:] Problems casting vertex: " << vertex_id << "!");
                continue;
            }
            vertices.emplace_back(vertex_id, vertex);
        }
            
        // Sorts by id.
        std::sort(vertices.begin(), vertices.end(), [](const auto& a, const auto& b) 
        {
            return a.first < b.first;
        });

        return vertices;
    }


}