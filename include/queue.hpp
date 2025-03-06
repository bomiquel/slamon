/**
 *  @file
 *  @brief Protected queue to be used as a shared resource between threads. Implements a simple FIFO.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2025
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


            Queue() {}


            /** \brief Adds an item to the end of the queue.
             *  \param[in] item The item to be inserted.
             */
            inline void pushBack(T item)
            {
                std::scoped_lock lock(mutex_queue_);
                queue_.push_back(item);
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