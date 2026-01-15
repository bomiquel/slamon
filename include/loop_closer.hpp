/**
 *  @file loop_closer.hpp
 *  @brief The LoopCloser class detects and closes loops between keypoint 
 *         clusters from multiple keyframes. 
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

 
#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP


#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>

#include "map.hpp"
#include "hash.hpp"
#include "queue.hpp"
#include "tools.hpp"
#include "cluster.hpp"
#include "publisher.hpp"
#include "motion_estimator.hpp"
#include "keypoint_matcher.hpp"

#include "slamon/GetProjectionMatrices.h"


namespace slamon
{


    class LoopCloser: protected KeypointMatcher, protected MotionEstimator, protected Publisher
    {
        public:


            /** \brief Class constructor
             *  \param[in] map Map object pointer.
             *  \param[in] lc_neighbors Number of neighbours to recover in order to 
             *  increase the possibility of closing a loop.
             *  \param[in] lc_candidates Number of candidates to close a loop.
             *  \param[in] min_lc_inliers Minimum number of inliers for loop closure.
             *  \param[in] lc_discard_window Window size of discarded graph vertices.
             *  \param[in] enable_debugging Enabling or disabling debugging.
             *  \param[in] cluster_queue Cluster queue pointer.
             *  \param[in] w_path Working path.
             *  \param[in] matching_type Desired matching type name.
             *  \param[in] motion_estimation_type Desired motion type name.
             */
            LoopCloser(Map* map,
                       const int lc_neighbors,
                       const int lc_candidates,
                       const int min_lc_inliers,
                       const int lc_discard_window,
                       const bool enable_debugging,
                       Queue<Cluster>* cluster_queue,
                       const std::string& w_path,
                       const std::string& matching_type,
                       const std::string& motion_estimation_type);


            /** \brief Starts loop closer.
             */
            void run();


        private:


            /** \brief Obtains, from the VO through a service, 
             *  the stereo camera projection matrices. 
             *  \param[out] l_proj_mat Left projection matrix.
             *  \param[out] r_proj_mat Right projection matrix.
             */
            void getProjectionMatrices(cv::Mat& l_proj_mat,
                                       cv::Mat& r_proj_mat);


            /** \brief Processes the new cluster.
             *  \param[in] q_cluster The query cluster.
             */
            void processNewCluster(const Cluster& q_cluster);


            /** \brief Get the best candidates to close a loop by hash.
             *  \param[in] q_cluster_id The query cluster id.
             *  \param[in] cand_num The number of best candidates to be retrieved.
             *  \param[in] no_cand Clusters list, outside the discard window, that cannot be
             *  candidates for closing loop.
             *  \param[out] closest_cand The list of best candidates.
             */
            void getHashCandidates(const int q_cluster_id, 
                                   const size_t cand_num,
                                   std::vector<int>& no_cand,
                                   std::vector<int>& closest_cand);


            /** \brief Returns whether the second element of the first
             *  std::pair is less than the second element of the second std::pair.
             *  \param[in] d_0 First pair.
             *  \param[in] d_1 Second pair.
             *  @return true if first pair is smaller than second pair.
             */
            static inline bool sortByMatching(const std::pair<int, float> d_0, const std::pair<int, float> d_1)
            {
                return (d_0.second < d_1.second);
            }


            /** \brief Reads cluster data from file.
             *  \param[in] id Cluster id.
             *  @return The cluster.
             */
            Cluster readCluster(const int id);


            /** \brief Checks if the current cluster was already read from disk in
             *  the last iteration, if it was already read it retrieves it, otherwise 
             *  it reads it from disk.
             *  \param[in] search_type Type of search used (proximity or hash).
             *  \param[in] cand_cluster_ids Identifiers of the candidate clusters.
             *  \param[out] cand_clusters The candidate clusters.
             *  \param[out] neighbours_cand_clusters The neighbors of the candidate clusters.
             */
            void readCandidateClusters(const std::string& search_type,
                                       const std::vector<int>& cand_cluster_ids,
                                       std::vector<std::pair<Cluster, std::string>>& cand_clusters,
                                       std::vector<std::vector<Cluster>>& neighbours_cand_clusters);


            /** \brief Checks if the neighboring cluster was already read from disk in
             *  the last iteration, if it was already read it retrieves it, otherwise
             *  it reads it from disk.
             *  \param[in] neighbor_id Neighbor identifier.
             *  \param[in] neighbours_cand_cluster Neighbors of the candidate group.
             *  @return The candidate neighbor cluster.
             */
            Cluster readCandidateNeighborsClusters(const int neighbor_id,
                                                   std::vector<Cluster>& neighbours_cand_cluster);


            /** \brief Updates the pose of a cluster.
             *  \param[in out] cluster to be upgraded.
             */
            void updateClusterPose(Cluster& cluster);


            /** \brief It tries to detect a loop closure between the query cluster
             *  and the candidate.
             *  \param[in] q_cluster Query cluster.
             *  \param[in] cand_cluster Candidate cluster.
             *  \param[in] neighbours_cand_cluster Neighbors of the candidate cluster.
             *  \param[out] estimated_motion Little transform between query and candidate
             *  cluster, if there is loop closure.
             *  \param[out] i_q_cluster_ids Cluster ids inliers from query cluster and its 
             *  neighbors.
             *  \param[out] i_cand_cluster_ids Cluster ids inliers from candidate cluster and its
             *  neighbors.
             *  \param[out] i_q_l_kpts Left keypoints inliers from query cluster and its neighbors.
             *  \param[out] i_q_r_kpts Right keypoints inliers from query cluster and its neighbors.
             *  \param[out] i_cand_l_kpts Left keypoints inliers from candidate cluster and its neighbors.
             *  \param[out] i_cand_r_kpts Right keypoints inliers from candidate cluster and its neighbors.
             *  @return True if there is a loop closure.
             */
            bool detectLoop(Cluster& q_cluster,
                            Cluster& cand_cluster,
                            std::vector<Cluster>& neighbours_cand_cluster,
                            tf2::Transform& estimated_motion,
                            std::vector<int>& i_q_cluster_ids,
                            std::vector<int>& i_cand_cluster_ids,
                            std::vector<cv::KeyPoint>& i_q_l_kpts,
                            std::vector<cv::KeyPoint>& i_q_r_kpts,
                            std::vector<cv::KeyPoint>& i_cand_l_kpts,
                            std::vector<cv::KeyPoint>& i_cand_r_kpts);


            /** \brief Add constrains to the map (edge(s) to the graph) and
             *  performs a pose-graph optimisation.
             *  \param[in] q_keyframe_id Identifier of the keyframe to which
             *  the query cluster belongs.
             *  \param[in] cand_keyframe_id Identifier of the keyframe to which
             *  the candidate cluster belongs.
             *  \param[in] search_method Type of search used: proximity or hash.
             *  \param[in] estimated_motion Little transform between query and 
             *  candidate cluster.
             *  \param[in] i_q_cluster_ids Cluster ids inliers from query cluster
             *  and its neighbors.
             *  \param[in] i_cand_cluster_ids Cluster ids inliers from candidate 
             *  cluster and its neighbors.
             *  \param[in] i_q_l_kpts Left keypoints inliers from query cluster 
             *  and its neighbors.
             *  \param[in] i_q_r_kpts Right keypoints inliers from query cluster
             *  and its neighbors.
             *  \param[in] i_cand_l_kpts Left keypoints inliers from candidate 
             *  cluster and its neighbors.
             *  \param[in] i_cand_r_kpts Right keypoints inliers from candidate
             *  cluster and its neighbors.
             *  @return If some edge has been added.
             */
            bool loopCorrection(const int q_keyframe_id,
                                const int cand_keyframe_id,
                                const std::string& search_method,
                                const tf2::Transform& estimated_motion,
                                const std::vector<int>& i_q_cluster_ids,
                                const std::vector<int>& i_cand_cluster_ids,
                                const std::vector<cv::KeyPoint>& i_q_l_kpts,
                                const std::vector<cv::KeyPoint>& i_q_r_kpts,
                                const std::vector<cv::KeyPoint>& i_cand_l_kpts,
                                const std::vector<cv::KeyPoint>& i_cand_r_kpts);


            /** \brief Gets the inliers per cluster pair.
             *  \param[in] i_q_cluster_ids Cluster ids inliers from query cluster and its 
             *  neighbors.
             *  \param[in] i_cand_cluster_ids Cluster ids inliers from candidate cluster and its
             *  neighbors.
             *  \param[in] i_q_l_kpts Left keypoints inliers from query cluster and its neighbors.
             *  \param[out] inliers_per_pair Number of inliers per cluster pair.
             *  \param[out] cand_keyframe_ids Identifiers of keyframes containing candidate 
             *  clusters that have closed loops.
             *  \param[out] cluster_pairs Cluster pairs vector (q_cluster_id - cand_cluster_id).
             *  \param[out] cand_cluster_keyframe_relations Stores the candidate cluster-keyframe
             *  relations (cand_cluster_id, cand_keyframe_id).
             */
            void getInliersPerClusterPair(const std::vector<int>& i_q_cluster_ids,
                                          const std::vector<int>& i_cand_cluster_ids,
                                          const std::vector<cv::KeyPoint>& i_q_l_kpts,
                                          std::vector<int>& inliers_per_pair,
                                          std::vector<int>& cand_keyframe_ids,
                                          std::vector<std::pair<int, int>>& cluster_pairs,
                                          std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations);


            // Attributes:
            
            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            bool init_loop_closer_; //!> Loop closer state.

            int lc_neighbors_; //!> Number of neighbours to recover in order to increase the possibility of closing the loop.

            int lc_candidates_; //!> Number of candidates to close a loop.

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            int max_edge_inliers_; //!> Maximum number of inliers for an edge.

            int lc_discard_window_; //!> Window size of discarded graph vertices (clusters).
            
            int loop_closure_num_; //!> Stores the number of loop closures.

            cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Left projection matrix.

            cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Right projection matrix.

            std::string lc_path_; //!> Path to store loop closures images with debugging purpose.

            std::string keyframe_path_; //!> Path to keyframes directories with debugging purpose.

            std::string execution_path_; //!> Path to store cluster data with execution purpose.

            std::vector<std::pair<int, int>> cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them.

            std::vector<std::pair<int, std::vector<float>>> hash_table_; //!> Hash table: stores a hash for every cluster. This is the unique variable that grows with the robot trajectory.
            
            std::vector<Cluster> p_q_cluster_neighbours_; //!> Previous neighbours of the query cluster.

            std::vector<std::pair<Cluster, std::string>> p_cand_clusters_; //!> Previous candidate clusters.

            std::vector<std::vector<Cluster>> p_cands_cluster_neighbours_; //!> Previous neighbours of the candidates.

            Map* map_; //!> Map object pointer.

            Queue<Cluster>* cluster_queue_; // Cluster queue pointer.

            haloc::Hash haloc_; //!> HAsh-based LOop Closure object.

            RunTimes run_times_; //!> Internal loop closing execution times.

            ros::NodeHandle nh_; //!> Public ROS node handle.
            
            ros::NodeHandle nhp_; //!> Private ROS node handle.

            ros::ServiceClient proj_mat_client_; //!> Service for requesting projection matrices.
            
            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher cluster_num_pub_; //!> Cluster number publisher.

            ros::Publisher loop_closure_num_pub_; //!> Loop closing number publisher.

            ros::Publisher loop_closure_img_pub_; //!> Loop closure image publisher.

            ros::Publisher loop_closure_matches_num_pub_; //!> Multi-cluster matches number publisher.

            ros::Publisher loop_closure_inliers_num_pub_; //!> Multi-cluster inliers number publisher.

    };


} // End namespace

#endif // LOOP_CLOSER_HPP