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
             *  \param min_lc_inliers Minimum number of inliers for loop closing.
             *  \param enable_debugging Enabling or disabling debugging.
             *  \param w_path Working path for debugging purpose.
             */
            LoopClosing(int min_lc_inliers,
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


            /** \brief Processes the new cluster
             */
            void processNewCluster();


        private:

            int p_frame_id_; 

            int cluster_counter_; 

            bool enable_debugging_; //!> Enabling or disabling debugging messages.

            int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

            std::list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

            std::string w_path_; //!> Path to working directory.

            std::string keyframe_path_; //!> Path to keyframes directories.

            std::string clusters_path_; //!> Path to store keypoints clustering with debugging purpose.

            std::string loop_closures_path_; //!> Path to store loop closures images.

            std::mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters.

            Graph* graph_; //!> Graph object pointer.

            Cluster c_cluster_; //!> Current cluster.
    };


} // End namespace

#endif // LOOP_CLOSING_HPP