/**
 *  \file initial_pose_server.hpp
 *  \brief Obtains the initial robot pose from the TF tree or ROS parameters 
 *         and provides it via a ROS service.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */


#ifndef INITIAL_POSE_SERVER_HPP
#define INITIAL_POSE_SERVER_HPP


#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "slamon/GetTransform.h"


/** \class InitialPoseServer
 *  \brief ROS service server that provides the initial 6-DoF pose of the robot.
 *  
 *  This class advertises the `"get_pose"` service. When a request is received, it first
 *  attempts to look up the transformation between the world frame and the robot frame 
 *  in the TF2 tree at the requested timestamp. If the TF lookup fails, it falls back 
 *  to returning a static pose loaded from the ROS parameter server (`.launch` or `.yaml`).
 */
class InitialPoseServer
{

    public:

        /** \brief Default class constructor. Loads ROS parameters, builds the fallback 
         *         transform, and advertises the initial pose service.
         */
        InitialPoseServer();


    protected:

        /** \brief Service callback that retrieves the initial robot pose.
         *  \param[in] req Service request containing the target query timestamp.
         *  \param[out] res Service response populated with the resulting stamped transform.
         *  \return `true` once the response transform has been populated (via TF2 or parameter fallback).
         */
        bool getInitialPose(slamon::GetTransform::Request& req,
                            slamon::GetTransform::Response& res);


    private:

        /** \brief Name of the reference world coordinate frame (e.g., `"world_ned"`). */
        std::string world_frame_;

        /** \brief Name of the robot base coordinate frame (e.g., `"base_link"`). */
        std::string robot_frame_;

        /** \brief Public ROS node handle. */
        ros::NodeHandle nh_;

        /** \brief Private ROS node handle (`"~"`). */
        ros::NodeHandle nhp_;

        /** \brief ROS service server for providing the initial pose. */
        ros::ServiceServer i_pose_server_;

        /** \brief Fallback static transform constructed from ROS parameters. */
        geometry_msgs::TransformStamped param_tf_;

        /** \brief TF2 buffer used to store and look up coordinate frame transforms. */
        tf2_ros::Buffer tf_buffer_;

        /** \brief TF2 transform listener that populates the transform buffer. */
        tf2_ros::TransformListener tf_listener_;
};


#endif // INITIAL_POSE_SERVER_HPP