/**
 *  \file queue.hpp
 *  \brief Thread-safe FIFO queue to be used as a shared resource between threads.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#ifndef QUEUE_HPP
#define QUEUE_HPP


#include <deque>
#include <mutex>


/** \namespace slamon
 *  \brief Main namespace for the SLAMON stereo visual SLAM system.
 */
namespace slamon
{


    /** \class Queue
     *  \brief Thread-safe First-In-First-Out (FIFO) queue wrapper around `std::deque`.
     *  \tparam T Type of the elements stored in the queue.
     *  
     *  This template class provides mutex-protected operations to safely share data 
     *  (such as `Cluster` objects) between producer and consumer threads.
     */
    template<typename T>
    class Queue
    {
        public:


            /** \brief Default class constructor.
             */
            Queue() {}


            /** \brief Appends an item to the back of the queue (copy insertion).
             *  \param[in] item The item to be inserted.
             */
            inline void pushBack(const T& item)
            {
                std::scoped_lock lock(mutex_queue_);
                queue_.push_back(item);
            }


            /** \brief Appends an item to the back of the queue (in-place construction).
             *  \param[in] item The item to be inserted.
             */
            inline void emplaceBack(const T& item)
            {
                std::scoped_lock lock(mutex_queue_);
                queue_.emplace_back(item);
            }


            /** \brief Checks whether the queue is currently empty.
             *  \return `true` if the queue contains no elements, `false` otherwise.
             */
            inline bool isEmpty()
            {
                std::scoped_lock lock(mutex_queue_);
                return (queue_.empty());
            }


            /** \brief Retrieves and removes the first element from the front of the queue.
             *  \return The element removed from the front of the queue.
             */
            inline T getFront()
            {
                std::scoped_lock lock(mutex_queue_);
                T item = queue_.front();
                queue_.pop_front();
                return item;
            }


        private:


            /** \brief Underlying double-ended queue container storing the elements. */
            std::deque<T> queue_;

            /** \brief Mutex protecting concurrent access to the queue. */
            std::mutex mutex_queue_;


    };


} // namespace slamon


#endif // QUEUE_HPP