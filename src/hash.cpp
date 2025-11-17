/**
 *  @file hash.cpp
 *  @brief Hash-based Loop Closure detection.
 *
 *  This file contains modified portions of code derived from the originals
 *  libhaloc and stereo_slam projects developed by the Systems, Robotics and
 *  Vision Group (srv.uib.es), University of the Balearic Islands.
 *  The original work is licensed under the BSD 4-Clause License (see below).
 *
 *  @author
 *  Original: 
 *    Pep Lluís Negre-Carrasco
 *  Modified by: 
 *    Bo Miquel Nordfeldt-Fiol
 * 
 *  @copyright
 *  Original copyright holder:
 *      Systems, Robotics and Vision Group (srv.uib.es)
 *      University of the Balearic Islands
 *      Copyright (c) 2013-2014
 *  Modified by:
 *      Bo Miquel Nordfeldt-Fiol (2024–2025)
 *      Copyright (c) 2025
 *      Released under the BSD 3-Clause License.
 *
 *  ---------------------------------------------------------------------------
 *  Original BSD 4-Clause License Notice:
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * All advertising materials mentioning features or use of this software
 *        must display the following acknowledgement:
 *          "This product includes software developed by the
 *           Systems, Robotics and Vision Group,
 *           University of the Balearic Islands"
 *      * Neither the name of the Systems, Robotics and Vision Group,
 *        University of the Balearic Islands nor the names of its contributors
 *        may be used to endorse or promote products derived from this software
 *        without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 *  ---------------------------------------------------------------------------
 *
 *  Additional license for modifications:
 *      The modifications made by Bo Miquel Nordfeldt-Fiol are released under
 *      the BSD 3-Clause License. See the top-level LICENSE file for details.
 */

#include <ros/ros.h>
#include "hash.hpp"
#include <opencv2/core/eigen.hpp>


haloc::Hash::Params::Params() :
  proj_num(DEFAULT_PROJ_NUM)
{}


haloc::Hash::Hash(): h_size_{-1}, initialized_{false} {}


bool haloc::Hash::isInitialized()
{
  // Locks haloc.
  std::scoped_lock lock(mutex_haloc_);

  return initialized_;
}


void haloc::Hash::setParams(const Params& params)
{
  params_ = params;
}


void haloc::Hash::init(cv::Mat desc)
{
  // Locks haloc.
  std::scoped_lock lock(mutex_haloc_);

  // Create the random projections vectors
  initProjections(desc.rows);

  // Set the size of the descriptors
  h_size_ = params_.proj_num * desc.cols;

  // Set initialized to true
  initialized_ = true;
}


std::vector<float> haloc::Hash::getHash(cv::Mat desc)
{
  // Locks haloc.
  std::scoped_lock lock(mutex_haloc_);

  // Initialize the histogram with 0's
  std::vector<float> hash(h_size_, 0.0);

  // Sanity check
  if (desc.rows == 0) return hash;

  // Project the descriptors
  size_t k = 0;
  for (size_t i=0; i<r_.size(); i++)
  {
    for (int n=0; n<desc.cols; n++)
    {
      float desc_sum = 0.0;
      for (int m=0; m<desc.rows; m++)
        desc_sum += r_[i][m]*desc.at<float>(m, n);

      hash[k] = desc_sum/(float)desc.rows;
      k++;
    }
  }

  return hash;
}


float haloc::Hash::match(std::vector<float> hash_1, std::vector<float> hash_2)
{
  // Compute the distance
  float sum = 0.0;
  for (size_t i=0; i<hash_1.size(); i++)
    sum += fabs(hash_1[i] - hash_2[i]);

  return sum;
}


void haloc::Hash::initProjections(int desc_size)
{
  // Initializations
  int seed = time(NULL);
  r_.clear();

  // The size of the descriptors may vary... We multiply the current descriptor size
  // for a scalar to handle the larger cases.
  int v_size = 6*desc_size;

  // We will generate N-orthogonal vectors creating a linear system of type Ax=b.
  // Generate a first random vector
  std::vector<float> r = compute_random_vector(seed, v_size);
  r_.push_back(unit_vector(r));

  std::cout << "params_.proj_num: " <<  params_.proj_num << std::endl;

  // Generate the set of orthogonal vectors
  for (int i=1; i<params_.proj_num; i++)
  {
    // Generate a random vector of the correct size
    std::vector<float> new_v = compute_random_vector(seed + i, v_size - i);

    // Get the right terms (b)
    Eigen::VectorXf b(r_.size());
    for (size_t n=0; n<r_.size(); n++)
    {
      std::vector<float> cur_v = r_[n];
      float sum = 0.0;
      for (size_t m=0; m<new_v.size(); m++)
      {
        sum += new_v[m]*cur_v[m];
      }
      b(n) = -sum;
    }

    // Get the matrix of equations (A)
    Eigen::MatrixXf A(i, i);
    for (size_t n=0; n<r_.size(); n++)
    {
      size_t k=0;
      for (size_t m=r_[n].size()-i; m<r_[n].size(); m++)
      {
        A(n,k) = r_[n][m];
        k++;
      }
    }

    // Apply the solver
    Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

    // Add the solutions to the new vector
    for (size_t n=0; n<r_.size(); n++)
      new_v.push_back(x(n));
    new_v = unit_vector(new_v);

    // Push the new vector
    r_.push_back(new_v);
  }
}


std::vector<float> haloc::Hash::compute_random_vector(unsigned int seed, int size)
{
  srand(seed);
  std::vector<float> h;
  for (int i=0; i<size; i++)
    h.push_back( (float)rand()/RAND_MAX );
  return h;
}


std::vector<float> haloc::Hash::unit_vector(std::vector<float> x)
{
  // Compute the norm
  float sum = 0.0;
  for (size_t i=0; i<x.size(); i++)
    sum += pow(x[i], 2.0);
  float x_norm = sqrt(sum);

  // x^ = x/|x|
  for (size_t i=0; i<x.size(); i++)
    x[i] = x[i] / x_norm;

  return x;
}