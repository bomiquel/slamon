/**
 *  @file
 *  @brief The Stereo Motion Estimation Modules class provides all the modules (feature-baseds)
 *  needed to calculate the motion between two pairs of stereoscopic images.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter 2024
 */

#ifndef VISION_MODULES_HPP
#define VISION_MODULES_HPP


#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class VisionModules
{
    public:

        // Matching types between pairs of stereo images.
        enum MatchType
        {
            RATIO_MATCHING               = 1,
            ESSENTIAL_MATCHING           = 2,
            HOMOGRAPHY_MATCHING          = 3,
            CIRCULAR_RATIO_MATCHING      = 4,
            CIRCULAR_ESSENTIAL_MATCHING  = 5,
            CIRCULAR_HOMOGRAPHY_MATCHING = 6
        };


        // Possible modules results.
        enum VisionResult
        {
            NOT_ENOUGH_KPS      = 0,
            NOT_ENOUGH_F_KPS    = 1,
            NOT_ENOUGH_E_KPS    = 2,
            GEOMETRIC_PROBLEM   = 3,
            DIFFERENT_PROBLEMS  = 4,
            NOT_ENOUGH_CIRCLES  = 5,
            SUCCESSFUL_MATCHING = 6
        };
        

        /** \brief Class constructor.
         *  \param ratio The ratio for Lowe's ratio test.
         *  \param min_num_kps The minimum number of keypoints to estimate the movement.
         *  \param matching_type Matching type.
         */
        VisionModules(double ratio,
                      int min_num_kps,
                      const std::string& matching_type);


        /** \brief Detects keypoints of an image.
         *  \param img The image (input).
         *  \param descs The image keypoints descriptors (output).
         *  \param kps The image keypoints (output).
         */
        void detectKeypoints(const cv::Mat& img,
                             cv::Mat& descs,
                             std::vector<cv::KeyPoint>& kps);


        /** \brief Parallelizes stereo image keypoint detection.
         *  \param l_img The left image (input).
         *  \param r_img The right image (input).
         *  \param l_descs The left image keypoints descriptors (output).
         *  \param r_descs The right image keypoints descriptors (output).
         *  \param l_kps The left image keypoints (output).
         *  \param r_kps The right image keypoints (output).
         */
        void detectStereoKeypoints(const cv::Mat& l_img,
                                   const cv::Mat& r_img,
                                   cv::Mat& l_descs,
                                   cv::Mat& r_descs,
                                   std::vector<cv::KeyPoint>& l_kps,
                                   std::vector<cv::KeyPoint>& r_kps); 


        /** \brief Matches keypoints of the same stereo pair and removes outliers.
         *  To eliminate outliers, the Lowe's ratio is applied first, followed by
         *  an epipolar constraint with a maximum error of 1 pixel.
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kps Left image keypoints (input).
         *  \param r_kps Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kps Filtered left image keypoints (output).
         *  \param r_l_kps Filtered right image keypoints (output).
         *  @return an enumerated type (VisionResult) encoding the result of the matching.
         */
        VisionModules::VisionResult matchStereoKeypoints(const cv::Mat& l_descs,
                                                         const cv::Mat& r_descs,
                                                         const std::vector<cv::KeyPoint>& l_kps,
                                                         const std::vector<cv::KeyPoint>& r_kps,
                                                         cv::Mat& f_l_descs,
                                                         cv::Mat& f_r_descs,
                                                         std::vector<cv::KeyPoint>& f_l_kps,
                                                         std::vector<cv::KeyPoint>& f_r_kps);


        /** \brief Matches keypoints of differents stereo pairs and removes outliers.
         *  To remove outliers, the user can choose between these different techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Plain essential matrix + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *      - Circular essential matrix + RANSAC.
         *  "Plain" involves only left images, "Circular" all 4 images. To select the
         *  desired outlier eliminator, the appropriate matcher (enum type: MatchType)
         *  must be chosen.
         *  \param c_l_descs Current left image descriptors (input).
         *  \param c_r_descs Current right image descriptors (input).
         *  \param p_l_descs Previous left image descriptors (input).
         *  \param p_r_descs Previous right image descriptors (input).
         *  \param c_l_kps Current left image keypoints (input).
         *  \param c_r_kps Current right image keypoints (input).
         *  \param p_l_kps Previous left image keypoints (input).
         *  \param p_r_kps Previous right image keypoints (input).
         *  \param c_camera_points Current image 3D points (input).
         *  \param p_camera_points Previous image 3D points (input).
         *  \param f_c_l_kps Current left image keypoints filtered out (output).
         *  \param f_c_r_kps Current right image keypoints filtered out (output).
         *  \param f_p_l_kps Previous left image keypoints filtered out (output).
         *  \param f_p_r_kps Previous right image keypoints filtered out (output).
         *  \param f_c_camera_points Current image 3D points filtered out (output).
         *  \param f_p_camera_points Previous image 3D points filtered out (output).
         *  @return an enumerated type (VisionResult) encoding the result of the matching.
         */
        VisionModules::VisionResult matchNonStereoKeypoints(const cv::Mat& c_l_descs,
                                                            const cv::Mat& c_r_descs,
                                                            const cv::Mat& p_l_descs,
                                                            const cv::Mat& p_r_descs,
                                                            const std::vector<cv::KeyPoint>& c_l_kps,
                                                            const std::vector<cv::KeyPoint>& c_r_kps,
                                                            const std::vector<cv::KeyPoint>& p_l_kps,
                                                            const std::vector<cv::KeyPoint>& p_r_kps,
                                                            const std::vector<cv::Point3f>& c_camera_points,
                                                            const std::vector<cv::Point3f>& p_camera_points,
                                                            std::vector<cv::KeyPoint>& f_c_l_kps,
                                                            std::vector<cv::KeyPoint>& f_c_r_kps,
                                                            std::vector<cv::KeyPoint>& f_p_l_kps,
                                                            std::vector<cv::KeyPoint>& f_p_r_kps,
                                                            std::vector<cv::Point3f>& f_c_camera_points,
                                                            std::vector<cv::Point3f>& f_p_camera_points);


    protected:


        /** \brief Matches visual keypoints
         *  \param c_descs Current image descriptors (input).
         *  \param q_descs Query image descriptors (input).
         *  @return The matches
         */
        std::vector<std::vector<cv::DMatch>> matching(const cv::Mat& c_descs, 
                                                      const cv::Mat& q_descs);


        /** \brief Matches visual keypoints and applies Lowe's ratio to remove
         *  outliers.
         *  \param c_descs Current image descriptors (input).
         *  \param q_descs Query image descriptors (input).
         *  \param c_kps Current image keypoints (input).
         *  \param q_kps Query image keypoints (input).
         *  \param c_camera_points Current image 3D points (input).
         *  \param q_camera_points Query image 3D points (input).
         *  \param f_c_descs Current image descriptors filtered out (output).
         *  \param f_q_descs Query image descriptors filtered out (output).
         *  \param f_c_kps Current image keypoints filtered out (output).
         *  \param f_q_kps Query image keypoints filtered out (output).
         *  \param f_c_camera_points Current image 3D points filtered out (output).
         *  \param f_q_camera_points Query image 3D points filtered out (output).
         *  @return Success or failure type.
         */
        VisionResult ratioMatching(const cv::Mat& c_descs, 
                                  const cv::Mat& q_descs,
                                  const std::vector<cv::KeyPoint>& c_kps,
                                  const std::vector<cv::KeyPoint>& q_kps,
                                  const std::vector<cv::Point3f>& c_camera_points,
                                  const std::vector<cv::Point3f>& q_camera_points,
                                  cv::Mat& f_c_descs,
                                  cv::Mat& f_q_descs,
                                  std::vector<cv::KeyPoint>& f_c_kps,
                                  std::vector<cv::KeyPoint>& f_q_kps,
                                  std::vector<cv::Point3f>& f_c_camera_points,
                                  std::vector<cv::Point3f>& f_q_camera_points);


        /** \brief Matches visual keypoints and applies a geometrical restriction 
         *  (essential matrix or homography) wrapped in a RANSAC scheme to remove
         *  outliers.
         *  \param coplanar_points If true, the points are assumed to be coplanar
         *  (or can be approximated to be in the same plane) and a homography is
         *  calculated to eliminate outliers. Otherwise, an essential matrix is 
         *  computed to remove outliers.
         *  \param c_descs Current image descriptors (input).
         *  \param q_descs Query image descriptors (input).
         *  \param c_kps Current image keypoints (input).
         *  \param q_kps Query image keypoints (input).
         *  \param c_camera_points Current image 3D points (input).
         *  \param q_camera_points Query image 3D points (input).
         *  \param f_c_descs Current image descriptors filtered out (output).
         *  \param f_q_descs Query image descriptors filtered out (output).
         *  \param f_c_kps Current image keypoints filtered out (output).
         *  \param f_q_kps Query image keypoints filtered out (output).
         *  \param f_c_camera_points Current image 3D points filtered out (output).
         *  \param f_q_camera_points Query image 3D points filtered out (output).
         *  @return Success or failure type.
         */
        VisionResult geometricalMatching(bool coplanar_points,
                                        const cv::Mat& c_descs, 
                                        const cv::Mat& q_descs,
                                        const std::vector<cv::KeyPoint>& c_kps,
                                        const std::vector<cv::KeyPoint>& q_kps,
                                        const std::vector<cv::Point3f>& c_camera_points,
                                        const std::vector<cv::Point3f>& q_camera_points,
                                        cv::Mat& f_c_descs,
                                        cv::Mat& f_q_descs,
                                        std::vector<cv::KeyPoint>& f_c_kps,
                                        std::vector<cv::KeyPoint>& f_q_kps,
                                        std::vector<cv::Point3f>& f_c_camera_points,
                                        std::vector<cv::Point3f>& f_q_camera_points);


        /** \brief Assemble the four matchings to build circular matches.
         *  \param c_0_stereo_kps Keypoints of the current image matched in stereo matching. 
         *  Must belong to the current image with the lowest number of points after the successive matching.
         *  \param c_1_stereo_kps Keypoints of the current image matched in stereo matching.
         *  Must belong to the current image with the highest number of points after the successive matching.
         *  \param p_0_stereo_kps Keypoints of the previous image matched in stereo matching.
         *  Must belong to the previous image with the lowest number of points after the successive matching.
         *  \param p_1_stereo_kps Keypoints of the previous image matched in stereo matching.
         *  Must belong to the previous image with the highest number of points after the successive matching.
         *  \param c_0_successive_kps Current keypoint vector with the lowest number of points
         *  after successive matching.
         *  \param c_1_successive_kps Current keypoint vector with the highest number of points
         *  after successive matching.
         *  \param p_0_successive_kps Previous keypoint vector with the lowest number of points
         *  after successive matching.
         *  \param p_1_successive_kps Previous keypoint vector with the highest number of points
         *  after successive matching.
         *  \param f_c_0_kps Filtered keypoints of current image with lowest number of points after successive matching.
         *  \param f_c_1_kps Filtered keypoints of current image with highest number of points after successive matching.
         *  \param f_p_0_kps Filtered keypoints of previous image with lowest number of points after successive matching.
         *  \param f_p_1_kps Filtered keypoints of previous image with highest number of points after successive matching.
         */
        VisionResult assembleCircularMatchings(const std::vector<cv::KeyPoint>& c_0_stereo_kps,
                                              const std::vector<cv::KeyPoint>& c_1_stereo_kps,
                                              const std::vector<cv::KeyPoint>& p_0_stereo_kps,
                                              const std::vector<cv::KeyPoint>& p_1_stereo_kps,
                                              const std::vector<cv::KeyPoint>& c_0_successive_kps,
                                              const std::vector<cv::KeyPoint>& c_1_successive_kps,
                                              const std::vector<cv::KeyPoint>& p_0_successive_kps,
                                              const std::vector<cv::KeyPoint>& p_1_successive_kps,
                                              std::vector<cv::KeyPoint>& f_c_0_kps,
                                              std::vector<cv::KeyPoint>& f_c_1_kps,
                                              std::vector<cv::KeyPoint>& f_p_0_kps,
                                              std::vector<cv::KeyPoint>& f_p_1_kps);


        double ratio_; //!> Lowe's ratio.

        int min_num_kps_; //!> The minimum number of keypoints to estimate the movement.

        MatchType matching_type_; //!> Matching type

        cv::Mat camera_matrix_; //!> Stereo camera matrix
};


#endif // VISION_MODULES_HPP