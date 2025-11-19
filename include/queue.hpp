/**
 *  @file queue.hpp
 *  @brief Protected queue to be used as a shared resource between threads. 
 *         Implements a simple FIFO.
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

 
#ifndef QUEUE_HPP
#define QUEUE_HPP


#include <deque>
#include <mutex>


namespace slamon
{


    template<typename T>
    class Queue
    {
        public:


            /** \brief Empty class constructor.
             */
            Queue() {}


            /** \brief Adds an item to the end of the queue.
             *  \param[in] item The item to be inserted.
             */
            inline void pushBack(T item)
            {
                std::scoped_lock lock(mutex_queue_);
                queue_.push_back(item);
            }


            /** \brief Adds an item to the end of the queue.
             *  \param[in] item The item to be inserted.
             */
            inline void emplaceBack(T item)
            {
                std::scoped_lock lock(mutex_queue_);
                queue_.emplace_back(item);
            }


            /** \brief Checks if there are items in the queue.
             *  @return True if queue is empty.
             */
            inline bool isEmpty()
            {
                std::scoped_lock lock(mutex_queue_);
                return (queue_.empty());
            }


            /** \brief Gets the first element of the queue.
             *  @return First element.
             */
            inline T getFront()
            {
                std::scoped_lock lock(mutex_queue_);
                T item = queue_.front();
                queue_.pop_front();
                return item;
            }


        private:


            // Attributes:

            std::deque<T> queue_; //!> The queue in question.

            std::mutex mutex_queue_; //!> Mutex for queue manipulation.

            
    };


}  // End namespace


#endif // QUEUE_HPP