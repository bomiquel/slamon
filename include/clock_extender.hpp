/**
 *  \file clock_extender.hpp
 *  \brief Extend the ROS clock by a few seconds after a bagfile has finished.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */


#ifndef CLOCK_EXTENDER_HPP
#define CLOCK_EXTENDER_HPP

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


/** \class ClockExtender
 *  \brief Monitors the `/clock` topic and continues publishing simulated time 
 *         for a set duration after a rosbag playback ends.
 *  
 *  This is particularly useful in SLAM pipelines to allow pending backend optimizations,
 *  loop closures, or map-saving routines to finish gracefully once the sensor stream stops.
 */
class ClockExtender
{
    public:

        /** \brief Class constructor. Initializes ROS handles, parameters, and the watchdog timer.
         */
        ClockExtender();


        /** \brief Class destructor. Ensures the worker thread is safely joined before shutdown.
         */
        ~ClockExtender();


    protected:

        /** \brief Callback for the `/clock` topic subscriber.
         *  \param[in] msg Incoming ROS clock message.
         */
        void clockCallback(const rosgraph_msgs::ClockConstPtr& msg);


        /** \brief Watchdog callback triggered periodically to check if the clock has stalled.
         *  \param[in] event The wall-clock timer event structure (`ros::WallTimerEvent`).
         */
        void timerCallback(const ros::WallTimerEvent& event);


        /** \brief Worker thread routine that artificially publishes and extends the ROS clock.
         */
        void extendClockWorker();


        /** \brief Safely checks and launches the clock-extension worker thread if none is currently active.
         *  \return `true` if a new worker thread was successfully launched, `false` otherwise.
         */    
        bool launchWorkerIfNeeded();


    private:
    
        // --- ROS Handles ---

        /** \brief Publisher for the extended `/clock` topic. */
        ros::Publisher pub_;

        /** \brief Subscriber to monitor the active `/clock` topic. */
        ros::Subscriber sub_;

        /** \brief Wall-time timer used to periodically check for clock inactivity. */
        ros::WallTimer timer_; 


        // --- Configuration Parameters ---

        /** \brief Additional duration to extend the simulated clock [s]. */
        double extra_time_; 

        /** \brief Time threshold of inactivity required to consider the bagfile finished [s]. */
        double gap_threshold_; 

        /** \brief Period of the watchdog timer [s]. */
        double watch_interval_;


        // --- Time Tracking ---

        /** \brief Timestamp of the last received ROS clock message. */
        ros::Time last_clock_time_; 

        /** \brief System wall-clock timestamp of when the last `/clock` message was received. */
        std::chrono::steady_clock::time_point last_msg_wall_time_;


        // --- Multi-threading & Synchronization ---

        /** \brief Mutex protecting access to `last_clock_time_` and `last_msg_wall_time_`. */
        std::mutex clock_mutex_;

        /** \brief Worker thread responsible for publishing the extended clock. */
        std::thread worker_thread_;

        /** \brief Flag indicating whether the clock is currently being extended. */
        std::atomic<bool> extending_;

        /** \brief Flag indicating whether at least one `/clock` message has been received. */
        std::atomic<bool> clock_received_;

        /** \brief Flag to signal the worker thread to abort execution prematurely. */
        std::atomic<bool> worker_stop_requested_;

};


#endif // CLOCK_EXTENDER_HPP