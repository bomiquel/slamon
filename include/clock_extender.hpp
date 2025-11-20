/**
 *  @file clock_extender.hpp
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


#ifndef CLOCK_EXTENDER_HPP
#define CLOCK_EXTENDER_HPP

#include <chrono>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class ClockExtender
{
    public:

        /** \brief Class constructor.
         */
        ClockExtender();


    protected:

        /** \brief Clock callback.
         *  \param[in] msg ROS clock message.
         */
        void clockCallback(const rosgraph_msgs::ClockConstPtr& msg);


        /** \brief Callback that is called when the timer is triggered.
         *  \param[in] event ros::VallTimerEvent.
         */
        void timerCallback(const ros::WallTimerEvent& event);


        /** \brief Extend clock.
         */
        void extendClock();


    private:

        double extra_time_; //!> The additional time you want to extend the clock.

        bool clock_received_; //!> Have I received any messages from the clock?
    
        ros::Publisher pub_; //!> Clock publisher.
    
        ros::Subscriber sub_; //!> Clock subscriber.
    
        ros::WallTimer timer_; //!> Timer to extend the clock.
    
        ros::Time last_clock_time_; //!> Last ROS time.

        std::chrono::high_resolution_clock::time_point last_msg_wall_time_; //!> Last PC time.

};


#endif // CLOCK_EXTENDER_HPP