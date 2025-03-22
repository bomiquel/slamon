/**
 *  @file
 *  @brief HAsh-based LOop Closure detection.
 *  @author Pep Lluís Negre-Carrasco
 *  @date 2014-2016
 *  @modified Bo Miquel Nordfeldt-Fiol 2024-2025
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