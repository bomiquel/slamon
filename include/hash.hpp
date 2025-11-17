/**
 *  @file hash.hpp
 *  @brief Hash-based Loop Closure detection.
 *  @package slamon
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

#ifndef HASH_HPP
#define HASH_HPP


#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Eigen>
#include <Eigen/Dense>


namespace haloc
{

  class Hash
  {

  public:

    /** \brief Parameter constructor. Sets the parameter struct to default values.
     */
    struct Params
    {
      //Default constructor sets all values to defaults.
      Params();

      // Class parameters
      int proj_num;                   //!> Number of projections required

      // Default values
      static const int              DEFAULT_PROJ_NUM = 3;
    };


    /** \brief Hash class constructor
     */
    Hash();


    /** \brief Return true if the class has been initialized.
     */
    bool isInitialized();


    /** \brief Sets the class parameters.
     *  \param[in] stuct of parameters.
     */
    void setParams(const Params& params);


    /** \brief Return current parameters
     */
    inline Params params() const { return params_; }


    /** \brief Class initialization.
     *  \param[in] desc Image keypoint descriptors.
     */
    void init(cv::Mat desc);


    /** \brief Compute the hash.
     *  \param[in] desc Containing the descriptors of the image.
     *  @return hash vector.
     */
    std::vector<float> getHash(cv::Mat desc);


    /** \brief Compute the distance between 2 hashes.
     * \param[in] hash_1 First hash.
     * \param[in] hash_2 Second hash.
     * @return the distance.
     */
    float match(std::vector<float> hash_1, std::vector<float> hash_2);


  private:


    /** \brief Compute the random vector/s needed to generate the hash.
     *  \param[in] desc_size of the initial descriptor matrix.
     */
    void initProjections(int desc_size);


    /** \brief Computes a random vector of some size.
     *  \param[in] seed to generate the random values.
     *  \param[in] desired size.
     *  @return random vector.
     */
    std::vector<float> compute_random_vector(unsigned int seed, int size);


    /** \brief Make a vector unit.
     *  \param[in] the "non-unitized" vector.
     *  @return the "unitized" vector.
     */
    std::vector<float> unit_vector(std::vector<float> x);

    
    // Attributes:

    int h_size_; //!> Size of the hashn

    Params params_; //!> Stores parametersn
    
    bool initialized_; //!> True when class has been initializedn

    std::mutex mutex_haloc_; //!> Mutex for haloc.
    
    std::vector<std::vector<float>> r_; //!> Vector of random valuesn


  };


} // namespace

#endif // HASH_HPP