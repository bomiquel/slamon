/**
 *  \file loop_closer.hpp
 *  \brief The LoopCloser class detects and closes loops between keypoint 
 *         clusters from multiple keyframes. 
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


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{


    /** \class LoopCloser
     *  \brief Detects and validates loop closures between spatial keypoint clusters 
     *         from different keyframes.
     *  
     *  This class retrieves loop closure candidates via spatial proximity and HALOC 
     *  hash matching, performs multi-cluster feature matching and 6-DoF motion estimation, 
     *  and adds loop closure constraints (edges) to the pose-graph map for global optimization.
     */
    class LoopCloser: protected KeypointMatcher, protected MotionEstimator, protected Publisher
    {
        public:


            /** \brief Parameterized class constructor.
             *  \param[in] map Pointer to the global SLAM map object.
             *  \param[in] lc_neighbors Number of neighboring clusters to retrieve to increase loop closure recall.
             *  \param[in] lc_candidates Number of candidate clusters to evaluate for loop closure.
             *  \param[in] min_lc_inliers Minimum number of geometrical inliers required to accept a loop closure.
             *  \param[in] lc_discard_window Window size of recent graph vertices (clusters) to ignore during candidate search.
             *  \param[in] enable_debugging If `true`, enables debugging outputs and image saving.
             *  \param[in] cluster_queue Pointer to the thread-safe queue of incoming clusters.
             *  \param[in] w_path Working directory path for storing temporary and debug data.
             *  \param[in] matching_type String identifier of the desired keypoint matching strategy.
             *  \param[in] motion_estimation_type String identifier of the desired motion estimation strategy.
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


            /** \brief Starts the main loop closer processing loop.
             */
            void run();


            /** \brief Finalizes the loop closer execution and removes temporary HALOC files from disk.
             */
            void finalise();


        private:


            /** \brief Requests the stereo camera projection matrices from the visual odometry node via a ROS service.
             *  \param[out] l_proj_mat Left camera 3x4 projection matrix.
             *  \param[out] r_proj_mat Right camera 3x4 projection matrix.
             */
            void getProjectionMatrices(cv::Mat& l_proj_mat,
                                       cv::Mat& r_proj_mat);


            /** \brief Processes a newly received cluster to update the hash table and search for loop closures.
             *  \param[in] q_cluster The incoming query cluster.
             */
            void processNewCluster(const Cluster& q_cluster);


            /** \brief Retrieves the best loop closure candidate clusters based on HALOC hash distance.
             *  \param[in] q_cluster_id Global ID of the query cluster.
             *  \param[in] cand_num Number of top candidates to retrieve.
             *  \param[in] no_cand List of cluster IDs (outside the discard window) excluded from being candidates.
             *  \param[out] closest_cand Output list of the best candidate cluster IDs.
             */
            void getHashCandidates(const int q_cluster_id, 
                                   const size_t cand_num,
                                   std::vector<int>& no_cand,
                                   std::vector<int>& closest_cand);


            /** \brief Comparator function to sort pairs of (cluster ID, hash distance) in ascending order of distance.
             *  \param[in] d_0 First pair (ID, distance).
             *  \param[in] d_1 Second pair (ID, distance).
             *  \return `true` if the distance of `d_0` is strictly less than that of `d_1`, `false` otherwise.
             */
            static inline bool sortByMatching(const std::pair<int, float>& d_0, const std::pair<int, float>& d_1)
            {
                return (d_0.second < d_1.second);
            }


            /** \brief Reads a cluster's data from disk given its global ID.
             *  \param[in] id Global cluster ID.
             *  \return The loaded `Cluster` object.
             */
            Cluster readCluster(const int id);


            /** \brief Retrieves candidate clusters and their neighbors, reusing cached data from the previous 
             *         iteration if available or reading from disk otherwise.
             *  \param[in] search_type Type of candidate search used (`"proximity"` or `"hash"`).
             *  \param[in] cand_cluster_ids Global IDs of the candidate clusters.
             *  \param[out] cand_clusters Vector of pairs containing the loaded candidate clusters and their search type.
             *  \param[out] neighbours_cand_clusters Vector containing the neighboring clusters for each candidate.
             */
            void readCandidateClusters(const std::string& search_type,
                                       const std::vector<int>& cand_cluster_ids,
                                       std::vector<std::pair<Cluster, std::string>>& cand_clusters,
                                       std::vector<std::vector<Cluster>>& neighbours_cand_clusters);


            /** \brief Retrieves a neighboring cluster of a candidate, reusing cached data from the previous 
             *         iteration if available or reading from disk otherwise.
             *  \param[in] neighbor_id Global ID of the neighboring cluster.
             *  \param[in] neighbours_cand_cluster Cached neighboring clusters from the candidate group.
             *  \return The retrieved neighbor `Cluster` object.
             */
            Cluster readCandidateNeighborsClusters(const int neighbor_id,
                                                   std::vector<Cluster>& neighbours_cand_cluster);


            /** \brief Updates the world-map camera pose of a given cluster using the latest pose-graph estimate.
             *  \param[in,out] cluster Cluster whose camera pose is to be updated.
             */
            void updateClusterPose(Cluster& cluster);


            /** \brief Attempts to detect and geometrically validate a loop closure between the query cluster 
             *         and a candidate cluster (including their respective neighbors).
             *  \param[in] q_cluster Query cluster.
             *  \param[in] cand_cluster Candidate cluster.
             *  \param[in] neighbours_cand_cluster Neighboring clusters of the candidate cluster.
             *  \param[out] estimated_motion Estimated relative 6-DoF transformation between the query and candidate clusters.
             *  \param[out] i_q_cluster_ids Inlier cluster IDs from the query cluster and its neighbors.
             *  \param[out] i_cand_cluster_ids Inlier cluster IDs from the candidate cluster and its neighbors.
             *  \param[out] i_q_l_kpts Inlier left keypoints from the query cluster and its neighbors.
             *  \param[out] i_q_r_kpts Inlier right keypoints from the query cluster and its neighbors.
             *  \param[out] i_cand_l_kpts Inlier left keypoints from the candidate cluster and its neighbors.
             *  \param[out] i_cand_r_kpts Inlier right keypoints from the candidate cluster and its neighbors.
             *  \return `true` if a valid loop closure is detected, `false` otherwise.
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


            /** \brief Adds loop closure constraints (edges) to the pose-graph map and triggers global pose-graph optimization.
             *  \param[in] q_keyframe_id ID of the keyframe to which the query cluster belongs.
             *  \param[in] cand_keyframe_id ID of the keyframe to which the candidate cluster belongs.
             *  \param[in] search_method Candidate search method used (`"proximity"` or `"hash"`).
             *  \param[in] estimated_motion Estimated relative 6-DoF transformation between the query and candidate clusters.
             *  \param[in] i_q_cluster_ids Inlier cluster IDs from the query cluster and its neighbors.
             *  \param[in] i_cand_cluster_ids Inlier cluster IDs from the candidate cluster and its neighbors.
             *  \param[in] i_q_l_kpts Inlier left keypoints from the query cluster and its neighbors.
             *  \param[in] i_q_r_kpts Inlier right keypoints from the query cluster and its neighbors.
             *  \param[in] i_cand_l_kpts Inlier left keypoints from the candidate cluster and its neighbors.
             *  \param[in] i_cand_r_kpts Inlier right keypoints from the candidate cluster and its neighbors.
             *  \return `true` if at least one loop closure edge was added to the graph, `false` otherwise.
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


            /** \brief Counts the number of inliers per individual cluster pair involved in a loop closure.
             *  \param[in] i_q_cluster_ids Inlier cluster IDs from the query cluster and its neighbors.
             *  \param[in] i_cand_cluster_ids Inlier cluster IDs from the candidate cluster and its neighbors.
             *  \param[in] i_q_l_kpts Inlier left keypoints from the query cluster and its neighbors.
             *  \param[out] inliers_per_pair Number of matched inliers for each unique cluster pair.
             *  \param[out] cand_keyframe_ids IDs of the keyframes containing candidate clusters that closed a loop.
             *  \param[out] cluster_pairs Vector of matched cluster ID pairs (`q_cluster_id`, `cand_cluster_id`).
             *  \param[out] cand_cluster_keyframe_relations Vector of pairs mapping candidate cluster IDs to their keyframe IDs (`cand_cluster_id`, `cand_keyframe_id`).
             */
            void getInliersPerClusterPair(const std::vector<int>& i_q_cluster_ids,
                                          const std::vector<int>& i_cand_cluster_ids,
                                          const std::vector<cv::KeyPoint>& i_q_l_kpts,
                                          std::vector<int>& inliers_per_pair,
                                          std::vector<int>& cand_keyframe_ids,
                                          std::vector<std::pair<int, int>>& cluster_pairs,
                                          std::vector<std::pair<int, int>>& cand_cluster_keyframe_relations);


            /** \brief Flag to enable or disable debugging outputs and image saving. */
            bool enable_debugging_;

            /** \brief Flag indicating whether the loop closer has been initialized. */
            bool init_loop_closer_;

            /** \brief Number of candidates neighboring clusters to retrieve to increase loop closure probability. */
            int lc_neighbors_;

            /** \brief Number of candidate clusters to compare with the query cluster for loop closure. */
            int lc_candidates_;

            /** \brief Minimum number of inliers required to accept a loop closure. */
            int min_lc_inliers_;

            /** \brief Maximum number of inliers recorded for a single loop closure edge. */
            int max_edge_inliers_;

            /** \brief Window size of recent graph vertices (clusters) discarded from candidate search. */
            int lc_discard_window_;
            
            /** \brief Total number of detected loop closures. */
            int loop_closure_num_;

            /** \brief Left stereo camera 3x4 projection matrix. */
            cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);

            /** \brief Right stereo camera 3x4 projection matrix. */
            cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1);

            /** \brief Directory path to store loop closure debug images. */
            std::string lc_path_;

            /** \brief Directory path to store keyframe debug data. */
            std::string keyframe_path_;

            /** \brief Directory path to store serialized cluster data during execution. */
            std::string execution_path_;

            /** \brief Pairs of cluster IDs that have already closed a loop, stored to prevent duplicate loop closures. */
            std::vector<std::pair<int, int>> cluster_lc_found_;

            /** \brief Hash table storing the global ID and HALOC hash vector for every cluster (grows linearly with trajectory length). */
            std::vector<std::pair<int, std::vector<float>>> hash_table_;
            
            /** \brief Cached neighboring clusters of the previous query cluster. */
            std::vector<Cluster> p_q_cluster_neighbours_;

            /** \brief Cached candidate clusters from the previous iteration along with their search method. */
            std::vector<std::pair<Cluster, std::string>> p_cand_clusters_;

            /** \brief Cached neighboring clusters of the previous candidate clusters. */
            std::vector<std::vector<Cluster>> p_cands_cluster_neighbours_;

            /** \brief Pointer to the global SLAM map object. */
            Map* map_;

            /** \brief Pointer to the thread-safe queue of incoming clusters. */
            Queue<Cluster>* cluster_queue_;

            /** \brief HAsh-based LOop Closure (HALOC) hash generator and matcher instance. */
            haloc::Hash haloc_;

            /** \brief Execution time profiler for internal loop closing stages. */
            RunTimes run_times_;

            /** \brief Public ROS node handle. */
            ros::NodeHandle nh_;
            
            /** \brief Private ROS node handle (`"~"`). */
            ros::NodeHandle nhp_;

            /** \brief ROS service client for requesting stereo projection matrices. */
            ros::ServiceClient proj_mat_client_;
            
            /** \brief ROS publisher for per-iteration loop closing execution times. */
            ros::Publisher run_times_pub_;

            /** \brief ROS publisher for the total number of processed clusters. */
            ros::Publisher cluster_num_pub_;

            /** \brief ROS publisher for the total number of detected loop closures. */
            ros::Publisher loop_closure_num_pub_;
            
            /** \brief ROS publisher for loop closure visualization images. */
            ros::Publisher loop_closure_img_pub_;

            /** \brief ROS publisher for the number of multi-cluster feature matches. */
            ros::Publisher loop_closure_matches_num_pub_;

            /** \brief ROS publisher for the number of multi-cluster geometrical inliers. */
            ros::Publisher loop_closure_inliers_num_pub_;

    };


} // namespace slamon

#endif // LOOP_CLOSER_HPP