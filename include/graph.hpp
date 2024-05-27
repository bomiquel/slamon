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
#include "stereo_modules.hpp"

#include "slamon/Frame.h"
#include "slamon/Keypoint.h"


class Graph: protected Caster, protected Publisher, protected StereoModules 
{   
    public:

        /** \brief Class constructor
         *  \param ratio The ratio for Lowe's ratio test.
         *  \param min_inliers The minimum number of keypoints to estimate the movement.
         *  \param enable_debugging Enabling or disabling debugging.
         *  \param w_path Working path for debugging purpose.
         *  \param matching_type Matching type.
         *  \param motion_estimation_type Motion estimation type.
         */
        Graph(double ratio,
              int min_inliers,
              bool enable_debugging,
              const std::string& w_path,
              std::string& matching_type,
              std::string& motion_estimation_type);


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

        bool init_graph_; //!> Graph state.

        Frame p_frame_; //!> Previous frame.

        bool enable_debugging_; //!> Enabling or disabling debugging messages.

        std::string w_path_; //!> Path to working directory.

        std::string f_path_; //!> Path to store frames with debugging purpose.

        std::string kf_path_; //!> Path to store keyframes with debugging purpose.

        // Topic sync
        typedef message_filters::sync_policies::ApproximateTime<slamon::Frame,
                                                                nav_msgs::Odometry> SyncPolicy_;

        typedef message_filters::Synchronizer<SyncPolicy_> Sync_;

};


#endif // GRAPH_HPP