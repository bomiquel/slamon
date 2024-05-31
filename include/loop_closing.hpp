/**
 *  @file
 *  @brief Loop closing class
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol Winter-Spring 2024
 */

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H


#include <string>
#include <vector>
#include <filesystem>

#include <ros/ros.h>

#include "caster.hpp"
#include "cluster.hpp"
#include "publisher.hpp"
#include "stereo_modules.hpp"


class LoopClosing: protected Caster, protected Publisher
{
    public:

        /** \brief Class constructor
         *  \param min_lc_inliers Minimum number of inliers for loop closing.
         *  \param enable_debugging Enabling or disabling debugging.
         *  \param w_path Working path for debugging purpose.
         */
        LoopClosing(int min_lc_inliers,
                    bool enable_debugging,
                    const std::string& w_path);


    protected:


    private:

        bool enable_debugging_; //!> Enabling or disabling debugging messages.

        int min_lc_inliers_; //!> Minimum number of inliers to close a loop.

        std::string w_path_; //!> Path to working directory.
};

#endif // LOOP_CLOSING_HPP