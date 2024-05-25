/**
 *  @file
 *  @brief Graph class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "frame.hpp"
#include "caster.hpp"
#include "publisher.hpp"

#include "slamon/Frame.h"
#include "slamon/Keypoint.h"


namespace slam
{
    class Graph: protected Caster, protected Publisher
    {   
        public:

            /** \brief Class constructor
             */
            Graph(const bool enable_debugging);


            /** \brief Starts graph
             */
            void run();


        protected:


            /** \brief Messages callback. This function is called when synchronized frame
             *  and odom messages are received.
             *  \param frame_msg Custom ROS message containing current stereo pair image
             *  relevant information (input).
             *  \param odom_msg ROS message containing current robot odometry (input).
             */  
            void frameCallback(const slamon::FrameConstPtr& frame_msg,
                               const nav_msgs::OdometryConstPtr& odom_msg);

        private:


            bool enable_debugging_;

            // Topic sync
            typedef message_filters::sync_policies::ApproximateTime<slamon::Frame,
                                                                    nav_msgs::Odometry> SyncPolicy_;

            typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

    };
}

#endif // GRAPH_HPP