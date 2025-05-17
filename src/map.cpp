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
        for (unsigned int i = 0; i < cluster_keyframe_relations_.size(); i++)
        {
            if (cluster_keyframe_relations_[i].first == vertex_id)
            {
                frame_id = cluster_keyframe_relations_[i].second;
                break;
            }
        }
        return frame_id;
    }


    tf::Transform Map::getVertexPose(const int vertex_id, 
                                     const bool use_lock)
    {
        // Inits variables.
        g2o::VertexSE3* vertex;
        tf::Transform vertex_pose;
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
            ROS_WARN_STREAM("[Map->getVertexPose:] Problems getting vertix: " << vertex_id << "! Returning identity!");

        return vertex_pose;
    }


    void Map::getVertexPoses(std::vector<int>& ids,
                             std::vector<double>& stamps,
                             std::vector<tf::Transform>& vertex_poses)
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


    tf::Transform Map::getVertexPoseRelativeToCamera(const int vertex_id)
    {
        std::scoped_lock lock(mutex_local_cluster_poses_);
        return local_cluster_poses_[vertex_id];
    }


    tf::Transform Map::getVertexCameraPose(const int vertex_id, 
                                           const bool use_lock)
    {
        tf::Transform vertex_pose = getVertexPose(vertex_id, use_lock);

        std::scoped_lock lock(mutex_local_cluster_poses_);
        return vertex_pose * local_cluster_poses_[vertex_id].inverse();
    }


    void Map::getKeyframeVertices(const int keyframe_id,
                                  std::vector<int>& vertices)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relations_);

        vertices.clear();
        for (unsigned int i = 0; i < cluster_keyframe_relations_.size(); i++)
        {
            if (cluster_keyframe_relations_[i].second == keyframe_id)
                vertices.push_back(cluster_keyframe_relations_[i].first);
        }
    }


    tf::Transform Map::getKeyframePose(const int keyframe_id)
    {
        std::vector<int> keyframe_vertices;
        getKeyframeVertices(keyframe_id, keyframe_vertices);
        return getVertexCameraPose(keyframe_vertices[0], true);
    }


    void Map::getKeyframePoses(std::vector<double>& stamps,
                               std::vector<tf::Transform>& keyframe_poses)
    {
        // Sanity check
        stamps.clear();
        keyframe_poses.clear();

        // Sorts graph vertices.
        std::vector<std::pair<int, g2o::VertexSE3*>> vertices = sortVertices();

        // Get keyframe poses.
        int vertex_id, keyframe_id;
        std::unordered_set<int> processed_keyframe_ids;
        for (std::vector<std::pair<int, g2o::VertexSE3*>>::iterator it = vertices.begin(); it != vertices.end(); it++)
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
            keyframe_poses.emplace_back(getVertexCameraPose(vertex_id, false));

            // Marks this keyframe as processed.
            processed_keyframe_ids.insert(keyframe_id);
        }
    }


    void Map::getLoopClosureEdges(std::vector<std::pair<int, int>>& keyframe_ids,
                                  std::vector<std::pair<tf::Transform, tf::Transform>>& keyframe_poses)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Sanity check
        keyframe_ids.clear();
        keyframe_poses.clear();

        // Gets loop closure edges.
        g2o::EdgeSE3* edge;
        int keyframe_0, keyframe_1;
        tf::Transform pose_0, pose_1;
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
        tf::Transform q_vertex_pose = getVertexPose(q_vertex_id, false);

        // Loop thought all the other nodes.
        double dist;
        int vertex_id;
        g2o::VertexSE3* vertex;
        tf::Transform neighbor_vertex_pose;
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
        for (int i = 0; i < final_closest_neighbors_num; i++)
            closest_neighbors.emplace_back(neighbors[i].first);
    }


    void Map::addTimestamp(const double stamp)
    {
        std::scoped_lock lock(mutex_keyframe_stamps_);
        keyframe_stamps_.push_back(stamp);
    }


    void Map::addLocalClusterPose(const tf::Transform& local_cluster_pose)
    {
        std::scoped_lock lock(mutex_local_cluster_poses_);
        local_cluster_poses_.push_back(local_cluster_pose);
    }


    void Map::addClusterKeyframeRelation(const std::pair<int, int>& relation)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relations_);
        cluster_keyframe_relations_.push_back(relation);
    }


    int Map::addVertex(const tf::Transform& pose)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Convert pose for graph.
        Eigen::Isometry3d vertex_pose = fromTfToIsometry(pose);

        // Set node id equal to graph size.
        int id = graph_.vertices().size();

        // Build the vertex.
        g2o::VertexSE3* c_vertex = new g2o::VertexSE3();
        c_vertex->setId(id);
        c_vertex->setEstimate(vertex_pose);
        
        // First time, no edges.
        if (id == 0)
            c_vertex->setFixed(true);

        // Add vertex.
        graph_.addVertex(c_vertex);

        return id;
    }


    void Map::addEdge(const int vertex_0_id,
                      const int vertex_1_id,
                      tf::Transform tf_edge)
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

        // Adds graph edge to graph.
        g2o::EdgeSE3* graph_edge = new g2o::EdgeSE3();
        Eigen::Isometry3d iso_edge = fromTfToIsometry(tf_edge);
        graph_edge->setVertex(0, vertex_0);
        graph_edge->setVertex(1, vertex_1);
        graph_edge->setMeasurement(iso_edge);

        graph_.addEdge(graph_edge);
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


    Eigen::Isometry3d Map::fromTfToIsometry(const tf::Transform& tf)
    {
        tf::Vector3 t_in = tf.getOrigin();
        tf::Quaternion q_in = tf.getRotation();
        Eigen::Vector3d t_out(t_in.getX(), t_in.getY(), t_in.getZ());
        Eigen::Quaterniond q_out;
        q_out.setIdentity();
        q_out.x() = q_in.getX();
        q_out.y() = q_in.getY();
        q_out.z() = q_in.getZ();
        q_out.w() = q_in.getW();
        Eigen::Isometry3d out = (Eigen::Isometry3d)q_out;
        out.translation() = t_out;
        return out;
    }


    tf::Transform Map::fromIsometryToTf(const Eigen::Isometry3d& iso)
    {
        Eigen::Vector3d t_in = iso.translation();
        Eigen::Quaterniond q_in = (Eigen::Quaterniond)iso.rotation();
        tf::Vector3 t_out(t_in.x(), t_in.y(), t_in.z());
        tf::Quaternion q_out(q_in.x(), q_in.y(), q_in.z(), q_in.w());
        tf::Transform out(q_out, t_out);
        return out;
    }


    tf::Transform Map::fromVertexToTf(const g2o::VertexSE3* vertex)
    {
        Eigen::Isometry3d iso = vertex->estimate();
        tf::Transform tf = fromIsometryToTf(iso);
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
                ROS_WARN_STREAM("[Map->sortVertices:] Problems casting vertex: " << vertex_id);
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