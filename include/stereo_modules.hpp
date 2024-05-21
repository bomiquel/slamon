/**
 *  @file
 *  @brief The Stereo Modules class provides all the modules (feature-baseds)
 *  needed to calculate the motion between two pairs of stereoscopic images.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter-Spring 2024
 */

#ifndef STEREO_MODULES_HPP
#define STEREO_MODULES_HPP


#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class StereoModules
{
    public:

        // Matching types between pairs of stereo images.
        enum MatchingType
        {
            RATIO_MATCHING               = 0,
            ESSENTIAL_MATCHING           = 1,
            HOMOGRAPHY_MATCHING          = 2,
            CIRCULAR_RATIO_MATCHING      = 3,
            CIRCULAR_ESSENTIAL_MATCHING  = 4,
            CIRCULAR_HOMOGRAPHY_MATCHING = 5
        };


        // Possible modules results.
        enum StereoResult
        {
            SUCCESSFUL          = 0,
            NOT_ENOUGH_KPTS     = 1,
            NOT_ENOUGH_F_KPTS   = 2,
            NOT_ENOUGH_E_KPTS   = 3,
            NOT_ENOUGH_3D_PTS   = 4,
            GEOMETRIC_PROBLEM   = 5,
            DIFFERENT_PROBLEMS  = 6,
            NOT_ENOUGH_CIRCLES  = 7,
            NOT_ENOUGH_INLIERS  = 8
        };
        

        /** \brief Class constructor. Projection matrices are not set.
         *  This constructor will be used only if is inheriting StereoModules
         *  class.
         *  \param ratio The ratio for Lowe's ratio test (input).
         *  \param min_inliers The minimum number of keypoints to estimate the movement (input).
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        StereoModules(const double ratio,
                      const int min_inliers,
                      std::string& matching_type,
                      std::string& motion_estimation_type);


        /** \brief Class constructor. Sets projection matrices.
         *  Assumes that left camera is the master.
         *  \param ratio The ratio for Lowe's ratio test (input).
         *  \param min_inliers The minimum number of keypoints to estimate the movement (input).
         *  \param l_proj_mat Left camera projection matrix (input).
         *  \param r_proj_mat Right camera projection matrix (input).
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        StereoModules(const double ratio,
                      const int min_inliers,
                      const cv::Mat& l_proj_mat,
                      const cv::Mat& r_proj_mat,
                      std::string& matching_type,
                      std::string& motion_estimation_type);


        /** \brief Class constructor. Builds projection matrices.
         *  Assumes that left camera is the master.
         *  \param b Stereo rig baseline in meters (input).
         *  \param f_x Focal length x (input).
         *  \param f_y Focal length y (input).
         *  \param c_x Principal point x (input).
         *  \param c_y Principal point y (input).
         *  \param ratio The ratio for Lowe's ratio test (input).
         *  \param min_inliers The minimum number of keypoints to estimate the movement (input).
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        StereoModules(const double b,
                      const double f_x,
                      const double f_y,
                      const double c_x,
                      const double c_y,
                      const double ratio,
                      const int min_inliers,
                      std::string& matching_type,
                      std::string& motion_estimation_type);


        /** \brief Detects keypoints of an image.
         *  \param img The image (input).
         *  \param detector_name String containing the name of the 
         *  desired detector for detecting image keypoints (input).
         *  \param descs The image keypoints descriptors (output).
         *  \param kpts The image keypoints (output).
         */
        void detectKeypoints(const cv::Mat& img,
                             const std::string& detector_name,
                             cv::Mat& descs,
                             std::vector<cv::Point2f>& kpts);


        /** \brief Parallelizes stereo image keypoint detection.
         *  \param l_img The left image (input).
         *  \param r_img The right image (input).
         *  \param detector_name String containing the name of the 
         *  desired detector for detecting image keypoints (input).
         *  \param l_descs The left image keypoints descriptors (output).
         *  \param r_descs The right image keypoints descriptors (output).
         *  \param l_kpts The left image keypoints (output).
         *  \param r_kpts The right image keypoints (output).
         */
        void detectStereoKeypoints(const cv::Mat& l_img,
                                   const cv::Mat& r_img,
                                   const std::string& detector_name,
                                   cv::Mat& l_descs,
                                   cv::Mat& r_descs,
                                   std::vector<cv::Point2f>& l_kpts,
                                   std::vector<cv::Point2f>& r_kpts); 


        /** \brief Matches keypoints of the same stereo pair and removes outliers.
         *  To eliminate outliers, the Lowe's ratio is applied first, followed by
         *  an epipolar constraint with a maximum error of 1 pixel.
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kpts Left image keypoints (input).
         *  \param r_kpts Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kpts Filtered left image keypoints (output).
         *  \param f_r_kpts Filtered right image keypoints (output).
         *  @return an enumerated type (StereoResult) encoding matching result.
         */
        StereoModules::StereoResult matchStereoKeypoints(const cv::Mat& l_descs,
                                                         const cv::Mat& r_descs,
                                                         const std::vector<cv::Point2f>& l_kpts,
                                                         const std::vector<cv::Point2f>& r_kpts,
                                                         cv::Mat& f_l_descs,
                                                         cv::Mat& f_r_descs,
                                                         std::vector<cv::Point2f>& f_l_kpts,
                                                         std::vector<cv::Point2f>& f_r_kpts);


        /** \brief Computes 3D points using OpenCV functions. It then removes 
         *  points categorised as "far". A 3D point is considered "far away"
         *  when its z is more than 40 times the stereo baseline. "Far" 3D 
         *  points are removed because they do not provide accurate scale
         *  and translation information (Paz et al. 2008 and Mur-Artal et al.
         *  2017).
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kpts Left image keypoints (input).
         *  \param r_kpts Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kpts Filtered left image keypoints (output).
         *  \param f_r_kpts Filtered right image keypoints (output).
         *  \param f_obj_pts Filtered 3D points of the scene obtained from
         *  the stereo data (output).
         *  @return an enumerated type (StereoResult) encoding the result.
         */
        StereoModules::StereoResult compute3DPoints(const cv::Mat& l_descs,
                                                    const cv::Mat& r_descs,
                                                    const std::vector<cv::Point2f>& l_kpts,
                                                    const std::vector<cv::Point2f>& r_kpts,
                                                    cv::Mat& f_l_descs,
                                                    cv::Mat& f_r_descs,
                                                    std::vector<cv::Point2f>& f_l_kpts,
                                                    std::vector<cv::Point2f>& f_r_kpts,
                                                    std::vector<cv::Point3f>& f_obj_pts);


        /** \brief Matches keypoints of differents stereo pairs and removes outliers.
         *  To remove outliers, the user can choose between these different techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Plain essential matrix + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *      - Circular essential matrix + RANSAC.
         *  "Plain" involves only left images, "Circular" all 4 images. To select the
         *  desired outlier eliminator, the appropriate matcher (enum type: MatchingType)
         *  must be chosen.
         *  \param c_l_descs Current left image descriptors (input).
         *  \param c_r_descs Current right image descriptors (input).
         *  \param p_l_descs Previous left image descriptors (input).
         *  \param p_r_descs Previous right image descriptors (input).
         *  \param c_l_kpts Current left image keypoints (input).
         *  \param c_r_kpts Current right image keypoints (input).
         *  \param p_l_kpts Previous left image keypoints (input).
         *  \param p_r_kpts Previous right image keypoints (input).
         *  \param c_obj_pts Current image 3D points (input).
         *  \param p_obj_pts Previous image 3D points (input).
         *  \param f_c_l_kpts Current left image keypoints filtered out (output).
         *  \param f_c_r_kpts Current right image keypoints filtered out (output).
         *  \param f_p_l_kpts Previous left image keypoints filtered out (output).
         *  \param f_p_r_kpts Previous right image keypoints filtered out (output).
         *  \param f_c_obj_pts Current image 3D points filtered out (output).
         *  \param f_p_obj_pts Previous image 3D points filtered out (output).
         *  @return an enumerated type (StereoResult) encoding matching result.
         */
        StereoModules::StereoResult matchNonStereoKeypoints(const cv::Mat& c_l_descs,
                                                            const cv::Mat& c_r_descs,
                                                            const cv::Mat& p_l_descs,
                                                            const cv::Mat& p_r_descs,
                                                            const std::vector<cv::Point2f>& c_l_kpts,
                                                            const std::vector<cv::Point2f>& c_r_kpts,
                                                            const std::vector<cv::Point2f>& p_l_kpts,
                                                            const std::vector<cv::Point2f>& p_r_kpts,
                                                            const std::vector<cv::Point3f>& c_obj_pts,
                                                            const std::vector<cv::Point3f>& p_obj_pts,
                                                            std::vector<cv::Point2f>& f_c_l_kpts,
                                                            std::vector<cv::Point2f>& f_c_r_kpts,
                                                            std::vector<cv::Point2f>& f_p_l_kpts,
                                                            std::vector<cv::Point2f>& f_p_r_kpts,
                                                            std::vector<cv::Point3f>& f_c_obj_pts,
                                                            std::vector<cv::Point3f>& f_p_obj_pts);


        /** \brief Estimate motion between frames using 3D-to-2D method.
         *  To be robust, the motion estimation is wrapped in a RANSAC scheme.
         *  \param c_l_kpts Current left image keypoints (input).
         *  \param c_r_kpts Current right image keypoints (input).
         *  \param p_obj_pts Previous image 3D points(input).
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_c_l_kpts Current left image keypoints filtered by the 
         *  RANSAC scheme (output).
         *  \param f_c_r_kpts Current right iamge keypoints filtered by the
         *  RANSAC scheme (output).
         *  \param f_p_obj_pts Previous image 3D points filtered by the 
         *  RANSAC scheme (output).
         *  @return an enumerated type (StereoResult) encoding motion estimation result.
         */
        StereoModules::StereoResult estimateMotion(const std::vector<cv::Point2f>& c_l_kpts,
                                                   const std::vector<cv::Point2f>& c_r_kpts,
                                                   const std::vector<cv::Point3f>& p_obj_pts,
                                                   cv::Mat& r_vec,
                                                   cv::Mat& t_vec,
                                                   std::vector<cv::Point2f>& f_c_l_kpts,
                                                   std::vector<cv::Point2f>& f_c_r_kpts,
                                                   std::vector<cv::Point3f>& f_p_obj_pts);


        /** \brief Estimate motion between frames using 3D-to-2D method.
         *  To be robust, the motion estimation is wrapped in a RANSAC scheme.
         *  \param c_l_kpts Current left image keypoints (input).
         *  \param c_r_kpts Current right image keypoints (input).
         *  \param p_l_kpts Previous left image keypoints (input).
         *  \param p_r_kpts Previous right image keypoints (input).
         *  \param c_obj_pts Current image 3D points (input).
         *  \param p_obj_pts Previous image 3D points (input).
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_c_l_kpts Current left image keypoints filtered by the 
         *  RANSAC scheme (output).
         *  \param f_c_r_kpts Current right iamge keypoints filtered by the
         *  RANSAC scheme (output).
         *  \param f_p_l_kpts Previous left image keypoints filtered by the 
         *  RANSAC scheme (output).
         *  \param f_p_r_kpts Previous right iamge keypoints filtered by the
         *  RANSAC scheme (output).
         *  \param f_c_obj_pts Current image 3D points filtered by the 
         *  RANSAC scheme (output).
         *  \param f_p_obj_pts Previous image 3D points filtered by the 
         *  RANSAC scheme (output).
         *  @return an enumerated type (StereoResult) encoding motion estimation result.
         */
        StereoModules::StereoResult estimateMotion(const std::vector<cv::Point2f>& c_l_kpts,
                                                   const std::vector<cv::Point2f>& c_r_kpts,
                                                   const std::vector<cv::Point2f>& p_l_kpts,
                                                   const std::vector<cv::Point2f>& p_r_kpts,
                                                   const std::vector<cv::Point3f>& c_obj_pts,
                                                   const std::vector<cv::Point3f>& p_obj_pts,
                                                   cv::Mat& r_vec,
                                                   cv::Mat& t_vec,
                                                   std::vector<cv::Point2f>& f_c_l_kpts,
                                                   std::vector<cv::Point2f>& f_c_r_kpts,
                                                   std::vector<cv::Point2f>& f_p_l_kpts,
                                                   std::vector<cv::Point2f>& f_p_r_kpts,
                                                   std::vector<cv::Point3f>& f_c_obj_pts,
                                                   std::vector<cv::Point3f>& f_p_obj_pts);


    protected:

        /** \brief Transforms a string into the correct MatchingType value.
         *  \param matching_type Desired matching type name (input).
         *  @return a MatchingType value.
         */
        StereoModules::MatchingType selectMatchingType(std::string& matching_type);


        /** \brief Transforms a string into the correct SolvePnPMethod value.
         *  \param motion_type Desired motion estimation type name (input).
         *  @return a SolvePnPMethod value.
         */
        cv::SolvePnPMethod selectMotionEstimationType(std::string& motion_estimation_type);


        /** \brief Computes the projection matrix of a stereo image.
         *  \param b Stereo rig baseline in meters (input).
         *  \param f_x Focal length x in pixels (input).
         *  \param f_y Focal length y in pixels (input).
         *  \param c_x Principal point x in pixels (input).
         *  \param c_y Principal point y in pixels (input).
         *  @return a projection matrix.
         */
        cv::Mat computeProjectionMatrix(const double b,
                                        const double f_x, 
                                        const double f_y, 
                                        const double c_x, 
                                        const double c_y);


        /** \brief Matches visual keypoints
         *  \param descs_0 First image descriptors (input).
         *  \param descs_1 Second image descriptors (input).
         *  @return The matches
         */
        std::vector<std::vector<cv::DMatch>> matching(const cv::Mat& descs_0, 
                                                      const cv::Mat& descs_1);


        /** \brief Matches visual keypoints and applies Lowe's ratio to remove
         *  outliers.
         *  \param descs_0 First image descriptors (input).
         *  \param descs_1 Second image descriptors (input).
         *  \param kpts_0 First image keypoints (input).
         *  \param kpts_1 Second image keypoints (input).
         *  \param obj_pts_0 First image 3D points (input).
         *  \param obj_pts_1 Second image 3D points (input).
         *  \param f_descs_0 First image descriptors filtered out (output).
         *  \param f_descs_1 Second image descriptors filtered out (output).
         *  \param f_kpts_0 First image keypoints filtered out (output).
         *  \param f_kpts_1 Second image keypoints filtered out (output).
         *  \param f_obj_pts_0 First image 3D points filtered out (output).
         *  \param f_obj_pts_1 Second image 3D points filtered out (output).
         *  @return an enumerated type (StereoResult) encoding matching result.
         */
        StereoResult ratioMatching(const cv::Mat& descs_0, 
                                   const cv::Mat& descs_1,
                                   const std::vector<cv::Point2f>& kpts_0,
                                   const std::vector<cv::Point2f>& kpts_1,
                                   const std::vector<cv::Point3f>& obj_pts_0,
                                   const std::vector<cv::Point3f>& obj_pts_1,
                                   cv::Mat& f_descs_0,
                                   cv::Mat& f_descs_1,
                                   std::vector<cv::Point2f>& f_kpts_0,
                                   std::vector<cv::Point2f>& f_kpts_1,
                                   std::vector<cv::Point3f>& f_obj_pts_0,
                                   std::vector<cv::Point3f>& f_obj_pts_1);


        /** \brief Matches visual keypoints and applies a geometrical restriction 
         *  (essential matrix or homography) wrapped in a RANSAC scheme to remove
         *  outliers.
         *  \param coplanar_pts If true, the points are assumed to be coplanar
         *  (or can be approximated to be in the same plane) and a homography is
         *  calculated to eliminate outliers. Otherwise, an essential matrix is 
         *  computed to remove outliers.
         *  \param descs_0 Current image descriptors (input).
         *  \param descs_1 Previous image descriptors (input).
         *  \param kpts_0 Current image keypoints (input).
         *  \param kpts_1 Previous image keypoints (input).
         *  \param obj_pts_0 Current image 3D points (input).
         *  \param obj_pts_1 Previous image 3D points (input).
         *  \param f_descs_0 Current image descriptors filtered out (output).
         *  \param f_descs_1 Previous image descriptors filtered out (output).
         *  \param f_kpts_0 Current image keypoints filtered out (output).
         *  \param f_kpts_1 Previous image keypoints filtered out (output).
         *  \param f_obj_pts_0 Current image 3D points filtered out (output).
         *  \param f_obj_pts_1 Previous image 3D points filtered out (output).
         *  @return an enumerated type (StereoResult) encoding matching result.
         */
        StereoResult geometricalMatching(bool coplanar_pts,
                                         const cv::Mat& descs_0, 
                                         const cv::Mat& descs_1,
                                         const std::vector<cv::Point2f>& kpts_0,
                                         const std::vector<cv::Point2f>& kpts_1,
                                         const std::vector<cv::Point3f>& obj_pts_0,
                                         const std::vector<cv::Point3f>& obj_pts_1,
                                         cv::Mat& f_descs_0,
                                         cv::Mat& f_descs_1,
                                         std::vector<cv::Point2f>& f_kpts_0,
                                         std::vector<cv::Point2f>& f_kpts_1,
                                         std::vector<cv::Point3f>& f_obj_pts_0,
                                         std::vector<cv::Point3f>& f_obj_pts_1);


        /** \brief Assemble the four matchings to build circular matches.
         *  \param c_0_stereo_kpts Keypoints of the current image matched in stereo matching. 
         *  Must belong to the current image with the lowest number of points after the 
         *  successive matching (input).
         *  \param c_1_stereo_kpts Keypoints of the current image matched in stereo matching.
         *  Must belong to the current image with the highest number of points after the 
         *  successive matching (input).
         *  \param p_0_stereo_kpts Keypoints of the previous image matched in stereo matching.
         *  Must belong to the previous image with the lowest number of points after the 
         *  successive matching (input).
         *  \param p_1_stereo_kpts Keypoints of the previous image matched in stereo matching.
         *  Must belong to the previous image with the highest number of points after the 
         *  successive matching (input).
         *  \param c_0_successive_kpts Current keypoint vector with the lowest number of points
         *  after successive matching (input).
         *  \param c_1_successive_kpts Current keypoint vector with the highest number of points
         *  after successive matching (input).
         *  \param p_0_successive_kpts Previous keypoint vector with the lowest number of points
         *  after successive matching (input).
         *  \param p_1_successive_kpts Previous keypoint vector with the highest number of points
         *  after successive matching (input).
         *  \param c_successive_obj_pts Current camera point vector after successive
         *  matching (input).
         *  \param p_successive_obj_pts Previous camera point vector after successive
         *  matching (input).
         *  \param f_c_0_kpts Filtered keypoints of current image with lowest number of points
         *  after successive matching (output).
         *  \param f_c_1_kpts Filtered keypoints of current image with highest number of points
         *  after successive matching (output).
         *  \param f_p_0_kpts Filtered keypoints of previous image with lowest number of points
         *  after successive matching (output).
         *  \param f_p_1_kpts Filtered keypoints of previous image with highest number of points
         *  after successive matching (output).
         *  \param f_c_obj_pts Filtered camera points of current stereo image.
         *  \param f_p_obj_pts Fultered camera points of previous stereo image.
         *  @return an enumerated type (StereoResult) encoding matching result.
         */
        StereoResult assembleCircularMatchings(const std::vector<cv::Point2f>& c_0_stereo_kpts,
                                               const std::vector<cv::Point2f>& c_1_stereo_kpts,
                                               const std::vector<cv::Point2f>& p_0_stereo_kpts,
                                               const std::vector<cv::Point2f>& p_1_stereo_kpts,
                                               const std::vector<cv::Point2f>& c_0_successive_kpts,
                                               const std::vector<cv::Point2f>& c_1_successive_kpts,
                                               const std::vector<cv::Point2f>& p_0_successive_kpts,
                                               const std::vector<cv::Point2f>& p_1_successive_kpts,
                                               const std::vector<cv::Point3f>& c_successive_obj_pts,
                                               const std::vector<cv::Point3f>& p_successive_obj_pts,
                                               std::vector<cv::Point2f>& f_c_0_kpts,
                                               std::vector<cv::Point2f>& f_c_1_kpts,
                                               std::vector<cv::Point2f>& f_p_0_kpts,
                                               std::vector<cv::Point2f>& f_p_1_kpts,
                                               std::vector<cv::Point3f>& f_c_obj_pts,
                                               std::vector<cv::Point3f>& f_p_obj_pts);


        double ratio_; //!> Lowe's ratio.

        int min_inliers_; //!> The minimum number of inliers to estimate the movement.
        
        cv::Mat l_proj_mat_ = cv::Mat::eye(3, 3, CV_64F); //!> Left camera projection matrix.
        
        cv::Mat r_proj_mat_ = cv::Mat::eye(3, 3, CV_64F); //!> Right camera projection matrix.

        MatchingType matching_type_; //!> Matching type.

        cv::SolvePnPMethod motion_estimation_type_; //!> Motion estimation type.
};


#endif // STEREO_MODULES_HPP