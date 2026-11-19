/**
 *  \file initial_pose_server_node.cpp
 *  \brief Launch the initial_pose_server
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#include "initial_pose_server.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_server_node");
    ros::start();

    InitialPoseServer node;

    ros::spin();

    return 0;
}