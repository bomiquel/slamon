/**
 *  @file
 *  @brief Loop closer class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef LOOP_CLOSER_HPP
#define LOOP_CLOSER_HPP


#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>

#include "map2.hpp"
#include "hash.hpp"
#include "tools.hpp"
#include "cluster.hpp"
#include "publisher.hpp"
#include "vision_motion_modules.hpp"

#include "slamon/GetProjectionMatrices.h"


namespace slamon
{
    

    class Map2;


    class LoopCloser: protected Publisher, protected VisionMotionModules
    {
        public:


            /** \brief Class constructor
             *  \param ratio The ratio for Lowe's ratio test (input).
             *  \param lc_neighbors Number of neighbours to recover in order to 
             *  increase the possibility of closing a loop (input).
             *  \param lc_candidates Number of candidates to close a loop (input).
             *  \param min_lc_inliers Minimum number of inliers for loop closure (input).
             *  \param lc_discard_window Window size of discarded graph vertices (input).
             *  \param enable_debugging Enabling or disabling debugging (input).
             *  \param w_path Working path for debugging purpose (input).
             *  \param matching_type Desired matching type name (input).
             *  \param motion_estimation_type Desired motion type name (input).
             */
            LoopCloser(double ratio,
                       int lc_neighbors,
                       int lc_candidates,
                       int min_lc_inliers,
                       int lc_discard_window,
                       bool enable_debugging,
                       const std::string& w_path,
                       std::string& matching_type,
                       std::string& motion_estimation_type);


            /** \brief Set the graph object.
             *  \param map The object pointer (input).
             */
            inline void setMap(Map2 *map) {map_ = map;}


            /** \brief Starts loop closer.
             */
            void run();


            /** \brief Add a cluster to the queue of clusters.
             *  \param Cluster The cluster to be inserted (input).
             */
            void addClusterToQueue(Cluster cluster);


        private:


            /** \brief Check if there are clusters in the queue.
             *  @return True if clusters queue is not empty.
             */
            bool checkNewClusterInQueue();


            /** \brief Obtains, from the VO and through a service, 
             *  the stereo camera projection matrices. 
             */
            void getProjectionMatrices();


            /** \brief Processes the new cluster.
             *  \param c_cluster The current cluster (output).
             */
            void processNewCluster(Cluster& c_cluster);


            /** \brief Searches a loop closing between current
             *  cluster and its closest neighbors
             *  \param c_cluster The current clusters (input).
             */
            void searchByProximity(const Cluster& c_cluster);


            /** \brief Searches a loop closing between current
             *  cluster and all other clusters using the hash
             *  \param c_cluster The current clusters (input).
             */
            void searchByHash(const Cluster& c_cluster);


            /** \brief Get the best candidates to close a loop by hash.
             *  \param c_cluster_id The current cluster id (input).
             *  \param closest_candidates_num The number of best candidates to be retrieved (input).
             *  \param candidates The list of best candidates (output).
             */
            void getHashCandidates(int c_cluster_id, 
                                   uint closest_candidates_num,
                                   std::vector<int>& closest_candidates);


            /** \brief Returns whether the second element of the first
             *  std::pair is less than the second element of the second std::pair.
             *  \param d_0 First pair (input).
             *  \param d_1 Second pair (input).
             *  @return true if first pair is smaller than second pair.
             */
            static inline bool sortByMatching(const std::pair<int, float> d_0, const std::pair<int, float> d_1)
            {
                return (d_0.second < d_1.second);
            }


            /** \brief Reads cluster data from file.
             *  \param id Cluster id (input).
             *  @return The cluster.
             */
            Cluster readCluster(int id);


            /** \brief Tries to close a loop between query and candidate cluster.
             *  If a loop is closed, the graph is updated.
             *  \param q_cluster Query cluster (input).
             *  \param cand_cluster Candidate cluster (input).
             *  \param seach_method Type of search used (proximity or hash (input).
             *  @return True if loop closing.
             */
            bool closeLoopBetween(Cluster q_cluster,
                                  Cluster cand_cluster,
                                  const std::string& search_method);

            // Atributes.
            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            bool init_loop_closer_; //!> Loop closer state.

            int lc_neighbors_; //!> Number of neighbours to recover in order to increase the possibility of closing the loop.

            int lc_candidates_; //!> Number of candidates to close a loop.

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            int lc_discard_window_; //!> Window size of discarded graph vertices (clusters).
            
            int loop_closure_num_; //!> Stores the number of loop closures.

            std::list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

            std::string w_path_; //!> Path to working directory.

            std::string lc_path_; //!> Path to store loop closures images with debugging purpose.

            std::string keyframe_path_; //!> Path to keyframes directories with debugging purpose.

            std::string execution_path_; //!> Path to store cluster data with execution purpose.

            std::mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters.

            std::vector<std::pair<int, int>> cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them.

            std::vector<std::pair<int, std::vector<float>>> hash_table_; //!> Hash table: stores a hash for every cluster. This is the unique variable that grows with the robot trajectory.

            ros::ServiceClient proj_mat_client_; //!> Service for requesting projection matrices.
            
            ros::Publisher run_times_pub_; //!> Last iteration performance publisher.

            ros::Publisher cluster_num_pub_; //!> Cluster number publisher.

            ros::Publisher loop_closure_num_pub_; //!> Loop closing number publisher.

            ros::Publisher loop_closure_img_pub_; //!> Loop closure image publisher.

            Map2* map_; //!> Graph object pointer.

            haloc::Hash haloc_; //!> HAsh-based LOop Closure object.

            RunTimes run_times_; // !> Internal loop closing execution times.
    };


} // End namespace

#endif // LOOP_CLOSER_HPPPP