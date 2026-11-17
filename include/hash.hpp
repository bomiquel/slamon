/**
 *  \file hash.hpp
 *  \brief HAsh-based LOop Closure detection.
 *
 *  This file contains modified portions of code derived from the original
 *  libhaloc and stereo_slam projects developed by the Systems, Robotics and
 *  Vision Group (srv.uib.es), University of the Balearic Islands.
 *
 *  \author Original: Pep Lluís Negre-Carrasco (2013)
 *  \author Modified: Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon). Both the original 
 *  work and the modifications are released under the BSD 4-Clause License. 
 *  See the top-level LICENSE file for details.
 */

 
#ifndef HASH_HPP
#define HASH_HPP

#include <vector>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Eigen>
#include <Eigen/Dense>


/** \namespace haloc
 *  \brief Namespace for the HAsh-based LOop Closure (HALOC) library.
 */
namespace haloc
{


  /** \class Hash
   *  \brief Computes compact image signatures (hashes) from feature descriptors 
   *         for fast loop closure candidate detection.
   *  
   *  This class projects a matrix of 2D feature descriptors onto a set of random 
   *  orthogonal directions to generate a global descriptor vector (hash) representing 
   *  the frame or cluster.
   */
  class Hash
  {

    public:

      /** \struct Params
       *  \brief Configuration parameters for the Hash computation.
       */
      struct Params
      {
        /** \brief Default constructor. Sets all parameters to their default values.
         */
        Params();

        /** \brief Number of random projections required to compute the hash. */
        int proj_num;

        /** \brief Default number of random projections. */
        static const int DEFAULT_PROJ_NUM = 3;
      };


      /** \brief Default class constructor.
       */
      Hash();


      /** \brief Checks whether the Hash class has been initialized.
       *  \return `true` if the class is initialized, `false` otherwise.
       */
      bool isInitialized();


      /** \brief Sets the configuration parameters for the class.
       *  \param[in] params Struct containing the configuration parameters.
       */
      void setParams(const Params& params);


      /** \brief Gets the current configuration parameters.
       *  \return Constant reference to the current parameters struct.
       */
      inline const Params& params() const {return params_;}


      /** \brief Initializes the hash projection vectors based on the input descriptor size.
       *  \param[in] desc Matrix of image keypoint descriptors.
       */
      void init(cv::Mat desc);


      /** \brief Computes the global hash vector for a given set of feature descriptors.
       *  \param[in] desc Matrix containing the feature descriptors of the image or cluster.
       *  \return Vector of floats representing the computed hash.
       */
      std::vector<float> getHash(cv::Mat desc);


      /** \brief Computes the distance between two hash vectors.
       *  \param[in] hash_1 First hash vector.
       *  \param[in] hash_2 Second hash vector.
       *  \return The computed distance between the two hashes.
       */
      float match(std::vector<float> hash_1, std::vector<float> hash_2);


    private:


      /** \brief Computes the random orthogonal projection vectors required to generate the hash.
       *  \param[in] desc_size Number of columns (dimensions) of the initial descriptor matrix.
       */
      void initProjections(int desc_size);


      /** \brief Generates a random vector of a specified size using a given seed.
       *  \param[in] seed Seed value to initialize the random number generator.
       *  \param[in] size Desired size of the random vector.
       *  \return Generated random vector.
       */
      std::vector<float> compute_random_vector(unsigned int seed, int size);


      /** \brief Normalizes a vector to unit length.
       *  \param[in] x The input non-normalized vector.
       *  \return The normalized unit vector.
       */
      std::vector<float> unit_vector(std::vector<float> x);


      /** \brief Size (dimensionality) of the computed hash vector. */
      int h_size_;

      /** \brief Stores the class configuration parameters. */
      Params params_;
      
      /** \brief Flag indicating whether the class has been initialized. */
      bool initialized_;

      /** \brief Mutex for thread-safe hash initialization and computation. */
      std::mutex mutex_haloc_;
      
      /** \brief Set of random orthogonal projection vectors. */
      std::vector<std::vector<float>> r_;

  };

} // namespace haloc

#endif // HASH_HPP