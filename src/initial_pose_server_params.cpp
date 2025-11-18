/**
 *  @file initial_pose_server_params.cpp
 *  @brief Obtain the initial pose from .launch or .yaml and publish it.
 *  @package slamon
 *  @project SLAMON
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @license BSD-4-Clause
 *    This file is part of the SLAMON project and is released under
 *    the BSD 4-Clause License. See the LICENSE file for details.
 */

 
#include <ros/ros.h>

#include "slamon/GetTransform.h"


double tx_, ty_, tz_, qx_, qy_, qz_, qw_;

bool getPoseCallback(slamon::GetTransform::Request  &req,
                     slamon::GetTransform::Response &res)
{
    res.tf.pose.position.x = tx_;
    res.tf.pose.position.y = ty_;
    res.tf.pose.position.z = tz_;
    res.tf.pose.orientation.x = qx_;
    res.tf.pose.orientation.y = qy_;
    res.tf.pose.orientation.z = qz_;
    res.tf.pose.orientation.w = qw_;
    
    return true;
}


void readParams()
{
    // Getting parameters from parameter server.
    ros::NodeHandle nhp("~");
    nhp.param("tx", tx_, 0.0);
    nhp.param("ty", ty_, 0.0);
    nhp.param("tz", tz_, 0.0);
    nhp.param("qx", qx_, 0.0);
    nhp.param("qy", qy_, 0.0);
    nhp.param("qz", qz_, 0.0);
    nhp.param("qw", qw_, 1.0);

    ROS_INFO_STREAM("[initial_pose_server:]" << std::endl <<
                    "tx = " << tx_ << std::endl << 
                    "ty = " << ty_ << std::endl << 
                    "tz = " << tz_ << std::endl << 
                    "qx = " << qx_ << std::endl << 
                    "qy = " << qy_ << std::endl << 
                    "qz = " << qz_ << std::endl << 
                    "qw = " << qw_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_server");
    ros::start();

    ros::NodeHandle nh;
    ros::ServiceServer initial_pose_server_ = nh.advertiseService("get_pose", getPoseCallback);

    readParams();

    ROS_INFO_STREAM("Ready to send punctual pose.");
    ros::spin();

    return 0;
}