#include "map.hpp"


namespace slamon
{

    Map::Map()
    {
        // Inilitialises the g2o graph optimizer.
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr (new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>());
        std::unique_ptr<g2o::BlockSolverX> solver_ptr (new g2o::BlockSolverX(std::move(linear_solver_ptr)));
        g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
        graph_.setAlgorithm(solver);
    }


    int Map::getCurrentVertexId()
    {
        std::scoped_lock lock(mutex_graph_);
        return graph_.vertices().size() - 1;
    }


    int Map::getVertexKeyframeId(int vertex_id)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relation_);

        int frame_id = -1;
        for (uint i = 0; i < cluster_keyframe_relation_.size(); i++)
        {
            if (cluster_keyframe_relation_[i].first == vertex_id)
            {
                frame_id = cluster_keyframe_relation_[i].second;
                break;
            }
        }
        return frame_id;
    }


    tf::Transform Map::getVertexPose(int vertex_id, bool use_lock)
    {
        if (use_lock)
        {
            // Locks map.
            std::scoped_lock lock(mutex_graph_);

            tf::Transform vertex_pose;
            if (vertex_id >= 0)
            {
                g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
                vertex_pose = fromVertexToTf(vertex);
            }
            else
            {
                vertex_pose.setIdentity();
            }
            return vertex_pose;
        }
        else
        {
            tf::Transform vertex_pose;
            if (vertex_id >= 0)
            {
                g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_id]);
                vertex_pose = fromVertexToTf(vertex);
            }
            else
            {
                vertex_pose.setIdentity();
            }
            return vertex_pose;
        }
    }


    tf::Transform Map::getVertexPoseRelativeToCamera(int vertex_id)
    {
        std::scoped_lock lock(mutex_local_cluster_poses_);
        return local_cluster_poses_[vertex_id];
    }


    tf::Transform Map::getVertexCameraPose(int vertex_id, bool use_lock)
    {
        tf::Transform vertex_pose = getVertexPose(vertex_id, use_lock);

        std::scoped_lock lock(mutex_local_cluster_poses_);
        return vertex_pose * local_cluster_poses_[vertex_id].inverse();
    }


    void Map::getKeyframeVertices(int keyframe_id,
                                  std::vector<int>& vertices)
    {
        std::scoped_lock lock(mutex_cluster_keyframe_relation_);

        vertices.clear();
        for (uint i = 0; i < cluster_keyframe_relation_.size(); i++)
        {
            if (cluster_keyframe_relation_[i].second == keyframe_id)
                vertices.push_back(cluster_keyframe_relation_[i].first);
        }
    }


    tf::Transform Map::getKeyframePose(int keyframe_id)
    {
        std::vector<int> keyframe_vertices;
        getKeyframeVertices(keyframe_id, keyframe_vertices);
        return getVertexCameraPose(keyframe_vertices[0], true);
    }


    void Map::getKeyframePoses(std::vector<double>& stamps,
                               std::vector<int>& keyframe_ids,
                               std::vector<tf::Transform>& keyframe_poses)
    {
        // Locks graph
        std::scoped_lock lock(mutex_graph_);

        // Sanity check
        stamps.clear();
        keyframe_ids.clear();
        keyframe_poses.clear();

        // Get keyframe poses
        bool found;
        int keyframe_id;
        for (uint vertex_id = 0; vertex_id < graph_.vertices().size(); vertex_id++)
        {
            // Is this keyframe processed?
            found = false;
            keyframe_id = getVertexKeyframeId(vertex_id);
            // for (uint processed_keyframe_id = processed_keyframes.size() - 1; processed_keyframe_id >= 0; processed_keyframe_id--)
            for (uint processed_keyframe_id = 0; processed_keyframe_id < keyframe_ids.size(); processed_keyframe_id++)
            {
                if (keyframe_ids[processed_keyframe_id] == keyframe_id)
                {
                    found = true;
                    break;
                }
            }
            if (found)
                continue;

            // Save data.
            keyframe_ids.push_back(keyframe_id);
            {
                std::scoped_lock lock(mutex_keyframe_stamps_);
                stamps.push_back(keyframe_stamps_[keyframe_id]);
            }
            keyframe_poses.push_back(getVertexCameraPose(vertex_id, false));
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
        g2o::EdgeSE3* e;
        int keyframe_0, keyframe_1;
        tf::Transform pose_0, pose_1;
        for (g2o::OptimizableGraph::EdgeSet::iterator it = graph_.edges().begin(); it != graph_.edges().end(); it++)
        {
            e = dynamic_cast<g2o::EdgeSE3*> (*it);
            if (e)
            {
                // Get the keyframes corresponding to this edge.
                keyframe_0 = getVertexKeyframeId(e->vertices()[0]->id());
                keyframe_1 = getVertexKeyframeId(e->vertices()[1]->id());

                if (abs(keyframe_0 - keyframe_1) > 1)
                {
                    // Get the keyframes camera pose.
                    pose_0 = getVertexCameraPose(e->vertices()[0]->id(), false);
                    pose_1 = getVertexCameraPose(e->vertices()[1]->id(), false);

                    // Save data.
                    keyframe_ids.push_back(std::make_pair(keyframe_0, keyframe_1));
                    keyframe_poses.push_back(std::make_pair(pose_0, pose_1));
                }
            }

        }
    }


    void Map::findClosestVertices(int vertex_id,
                                  int security_window,
                                  int critical_vertex_id,
                                  uint closest_neighbors_num,
                                  std::vector<int>& closest_neighbors)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Init.
        closest_neighbors.clear();
        tf::Transform q_vertex_pose = getVertexPose(vertex_id, false);

        // Loop thought all the other nodes.
        double dist;
        tf::Transform neighbor_vertex_pose;
        std::vector<std::pair<int, double>> neighbors;
        int search_end = critical_vertex_id - security_window;
        for (int i = 0; i <= search_end; i++)
        {
            // Avoids itself.
            if (i == vertex_id)
                continue;

            // Gets the distance between the queried vertex and the ith vertex.
            neighbor_vertex_pose = getVertexPose(i, false);
            dist = poseDiff2D(neighbor_vertex_pose, q_vertex_pose);
            neighbors.push_back(std::make_pair(i, dist));
        }

        // Exits if no neighbors.
        if (neighbors.size() == 0)
            return;

        // Sorts the neighbours from least to furthest distance to the query neighbour.
        std::sort(neighbors.begin(), neighbors.end(), sortByDistance);

        // Gets closest vertices (minnor distance)
        if (closest_neighbors_num > neighbors.size())
            closest_neighbors_num = neighbors.size();
        for (uint i = 0; i < closest_neighbors_num; i++)
            closest_neighbors.push_back(neighbors[i].first);
    }


    void Map::addTimestamp(double stamp)
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
        std::scoped_lock lock(mutex_cluster_keyframe_relation_);
        cluster_keyframe_relation_.push_back(relation);
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


    void Map::addEdge(int vertex_0_id,
                      int vertex_1_id,
                      tf::Transform tf_edge)
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Gets the graph vertices.
        g2o::VertexSE3* vertex_0 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_0_id]);
        g2o::VertexSE3* vertex_1 = dynamic_cast<g2o::VertexSE3*>(graph_.vertices()[vertex_1_id]);

        // Adds graph edge to graph.
        g2o::EdgeSE3* graph_edge = new g2o::EdgeSE3();
        Eigen::Isometry3d iso_edge = fromTfToIsometry(tf_edge);
        graph_edge->setVertex(0, vertex_0);
        graph_edge->setVertex(1, vertex_1);
        graph_edge->setMeasurement(iso_edge);

        graph_.addEdge(graph_edge);
    }


    void Map::update()
    {
        // Locks map.
        std::scoped_lock lock(mutex_graph_);

        // Optimize!
        graph_.initializeOptimization();
        graph_.optimize(20);

        ROS_INFO_STREAM("[Map->updater:] Optimization done in graph with: " << graph_.vertices().size() << " vertices.");
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
}