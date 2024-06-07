/**
 *  @file
 *  @brief Loop closing class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H


#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>

#include "hash.hpp"
#include "graph.hpp"
#include "caster.hpp"
#include "cluster.hpp"
#include "publisher.hpp"
#include "stereo_modules.hpp"


namespace slam
{
    

    class Graph;


    class LoopClosing: protected Caster, protected Publisher
    {
        public:


            /** \brief Class constructor
             *  \param lc_neighbors Number of neighbours to recover in order to 
             *  increase the possibility of closing a loop.
             *  \param min_lc_inliers Minimum number of inliers for loop closing.
             *  \param lc_discard_window Window size of discarded graph vertices.
             *  \param enable_debugging Enabling or disabling debugging.
             *  \param w_path Working path for debugging purpose.
             */
            LoopClosing(int lc_neighbors,
                        int min_lc_inliers,
                        int lc_discard_window,
                        bool enable_debugging,
                        const std::string& w_path);


            /** \brief Set the graph object
             *  \param graph The object pointer
             */
            inline void setGraph(Graph *graph) {graph_ = graph;}


            /** \brief Starts loop closing.
             */
            void run();


            /** \brief Add a cluster to the queue of clusters.
             *  \param Cluster The cluster to be inserted.
             */
            void addClusterToQueue(Cluster cluster);


        protected:


            /** \brief Check if there are clusters in the queue.
             *  @return True if clusters queue is not empty.
             */
            bool checkNewClusterInQueue();


            /** \brief Processes the new cluster.
             *  \param c_cluster The current cluster (output).
             */
            void processNewCluster(Cluster& c_cluster);


            /** \brief Searches a loop closing between current
             *  cluster and all other clusters using the hash
             *  \param c_cluster The current clusters (input).
             */
            void searchByHash(const Cluster& c_cluster);


            /** \brief Get the best candidates to close a loop by hash.
             *  \param c_cluster_id The current cluster id.
             *  \param candidates The list of best candidates.
             */
            void getHashCandidates(int c_cluster_id, 
                                   std::vector<std::pair<int, float>>& candidates);


            /** \brief Sort 2 std::pairs by size.
             *  \param d_0 First pair.
             *  \param d_1 Second pair.
             *  @return true if first pair is smaller than second pair.
             */
            static inline bool sortByDistance(const std::pair<int, double> d_0, const std::pair<int, double> d_1)
            {
                return (d_0.second < d_1.second);
            }


        private:

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            int cluster_id_; //!> Processed cluster counter.

            int lc_neighbors_; //!> Number of neighbours to recover in order to increase the possibility of closing the loop.

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            int lc_discard_window_; //!> Window size of discarded graph vertices (clusters).

            std::list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

            std::string w_path_; //!> Path to working directory.

            std::string lc_path_; //!> Path to store loop closures images with debugging purpose.

            std::string keyframe_path_; //!> Path to keyframes directories with debugging purpose.

            std::string execution_path_; //!> Path to store cluster data with execution purpose.

            std::mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters.

            Graph* graph_; //!> Graph object pointer.

            haloc::Hash haloc_; //!> HAsh-based LOop Closure object.

            std::vector<std::pair<int, int>> cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them.

            std::vector<std::pair<int, std::vector<float>>> hash_table_; //!> Hash table: stores a hash for every cluster. This is the unique variable that grows with the robot trajectory.
    };


} // End namespace

#endif // LOOP_CLOSING_HPP