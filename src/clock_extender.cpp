/**
 *  @file clock_extender.cpp
 *  @brief Extend the ROS clock by a few seconds after a bagfile has finished.
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


#include "clock_extender.hpp"


ClockExtender::ClockExtender(): clock_received_{false}
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Get param
    nhp.param<double>("extra_time", extra_time_, 1);
    ROS_WARN_STREAM("Get " << extra_time_ << " seconds of extra time");

    // Publishers.
    pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // Subscribers.
    sub_ = nh.subscribe("/clock", 10, &ClockExtender::clockCallback, this);

    // Timers.
    timer_ = nh.createWallTimer(ros::WallDuration(1.5), &ClockExtender::timerCallback, this);

    last_msg_wall_time_ = std::chrono::high_resolution_clock::now();

    extending_ = false;
}


void ClockExtender::clockCallback(const rosgraph_msgs::ClockConstPtr& msg)
{
    clock_received_ = true;
    last_clock_time_ = msg->clock;
    last_msg_wall_time_ = std::chrono::high_resolution_clock::now();
}


void ClockExtender::timerCallback(const ros::WallTimerEvent& event)
{
    ROS_WARN_STREAM("[CE->timerCallback:] extending: " << extending_);

    if (extending_)
        return; 
    
    std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - last_msg_wall_time_;

    if (diff.count() > 1)
    {
        extending_ = true;
        extendClock();
        ros::requestShutdown();
    }
}


void ClockExtender::extendClock()
{
    if (!clock_received_)
    {
        ROS_WARN("[CE->extendClock:] No /clock received. Nothing to extend.");
        return;
    }

    ROS_INFO_STREAM("Extending /clock for " << extra_time_ << " seconds ... ");

    // Frequency.
    ros::Rate rate(50);

    // Delta time.
    double dt = 1.0 / 50;

    ros::Time t = last_clock_time_;
    std::chrono::high_resolution_clock::time_point end_wall = std::chrono::high_resolution_clock::now() + 
                                                              std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::duration<double>(extra_time_));

    while (ros::ok() && std::chrono::high_resolution_clock::now() < end_wall)
    {
        t += ros::Duration(dt);

        rosgraph_msgs::Clock clk_msg;
        clk_msg.clock = t;
        pub_.publish(clk_msg);

        rate.sleep();
    }

    ROS_INFO("/clock extension finished.");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "clock_extender");

    ClockExtender clock_extender;

    ros::spin();

    return 0;
}