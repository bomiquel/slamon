/**
 *  @file motion_estimator.hpp
 *  @brief The MotionEstimtor class provides tools to compute the motion between stereo images.
 *  @package slamon
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2024–2025)
 *
 *  @copyright
 *    Copyright (c) 2025 Bo Miquel Nordfeldt-Fiol
 *
 *  @license BSD-3-Clause
 *    This software is released under the BSD 3-Clause License.
 *    See the LICENSE file for full text.
 */

#ifndef MOTION_ESTIMATOR_HPP
#define MOTION_ESTIMATOR_HPP


#include <vector>
#include <string>
#include <random>
#include <numeric>
#include <iostream>
#include <algorithm>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>


class MotionEstimator
{

    public:


        enum Type
        {
            EPNP = cv::SOLVEPNP_EPNP,
            P3P = cv::SOLVEPNP_P3P,
            AP3P = cv::SOLVEPNP_AP3P,
        };


        /** \brief Empty class constructor.
         */
        MotionEstimator() {};


        /** \brief Class constructor.
         *  \param[in] motion_estimation_type Desired motion type name.
         */
        MotionEstimator(const std::string& motion_estimation_type);


        /** \brief Builds a projection matrix from a stereo camera from its
         *  calibration parameters.
         *  \param[in] b Stereo rig baseline in meters.
         *  \param[in] f_x Focal length x in pixels.
         *  \param[in] f_y Focal length y in pixels.
         *  \param[in] c_x Principal point x in pixels.
         *  \param[in] c_y Principal point y in pixels.
         *  @return a projection matrix.
         */
        cv::Mat buildProjectionMatrix(const double b,
                                      const double f_x, 
                                      const double f_y, 
                                      const double c_x, 
                                      const double c_y);

        
        /** \brief Computes 3D points using OpenCV functions. It then removes points
         *  categorised as "far". A 3D point is considered "far away" when its z is
         *  more than 40 times the stereo baseline. "Far" 3D points are removed because
         *  they do not provide accurate scale and translation information (Paz et al.
         *  2008 and Mur-Artal et al. 2017).
         *  \param[in] l_proj_mat Left projection matrix.
         *  \param[in] r_proj_mat Right projection matrix.
         *  \param[in] l_descs Left image descriptors.
         *  \param[in] r_descs Right image descriptors.
         *  \param[in] l_kpts Left image keypoints.
         *  \param[in] r_kpts Right image keypoints.
         *  \param[out] f_l_descs Filtered left image descriptors.
         *  \param[out] f_r_descs Filtered right image descriptors.
         *  \param[out] f_l_kpts Filtered left image keypoints.
         *  \param[out] f_r_kpts Filtered right image keypoints.
         *  \param[out] f_l_cam_pts Filtered 3D points of the scene obtained from
         *  the stereo data and relative to left camera.
         */
        void compute3DPoints(const cv::Mat& l_proj_mat,
                             const cv::Mat& r_proj_mat,
                             const cv::Mat& l_descs,
                             const cv::Mat& r_descs,
                             const std::vector<cv::KeyPoint>& l_kpts,
                             const std::vector<cv::KeyPoint>& r_kpts,
                             cv::Mat& f_l_descs,
                             cv::Mat& f_r_descs,
                             std::vector<cv::KeyPoint>& f_l_kpts,
                             std::vector<cv::KeyPoint>& f_r_kpts,
                             std::vector<cv::Point3f>& f_l_cam_pts);   


        /** \brief Homogenises the distribution of keypoints over the spatial domain of the image.
         *  For this purpose, the current left image is divided into a grid and a maximum of n 
         *  keypoints per cell is imposed. The following procedure is used to decide which keypoints
         *  to keep:
         *      1. A line is calculated between each feature of the bucket and its correspondence in
         *         the previous image.
         *      2. The slopes of the straight lines are ordered in order from the lowest to the 
         *         highest slope.
         *      3. The median of the set of slopes is computed.
         *      4. Finally, the keypoints that generate the lines with slopes most similar to the
         *         median are retained.
         *  \param[in] max_kpts Maximum number of keypoints per bucket.
         *  \param[in] img_width Number of columns in the image.
         *  \param[in] img_height Number of rows in the image.
         *  \param[in] c_l_kpts Current left keypoints.
         *  \param[in] c_r_kpts Current right keypoints.
         *  \param[in] p_l_kpts Previous left keypoints.
         *  \param[in] p_r_kpts Previous right keypoints.
         *  \param[in] c_l_cam_pts Current 3D points relative to left camera.
         *  \param[in] p_l_cam_pts Previous 3D points relative to left camera.
         *  \param[out] f_c_l_kpts Bucketed current left keypoints.
         *  \param[out] f_c_r_kpts Bucketed current right keypoints. 
         *  \param[out] f_p_l_kpts Bucketed previous left keypoints. 
         *  \param[out] f_p_r_kpts Bucketed previous right keypoints. 
         *  \param[out] f_c_l_cam_pts Bucketed current 3D points relative to left camera.
         *  \param[out] f_p_l_cam_pts Bucketed previous 3D points relative to left camera.
         */
        std::vector<std::vector<std::pair<int, cv::KeyPoint>>> bucketKeypoints(const size_t max_kpts,
                                                                               const float img_width,
                                                                               const float img_height,
                                                                               const std::vector<cv::KeyPoint>& c_l_kpts,
                                                                               const std::vector<cv::KeyPoint>& c_r_kpts,
                                                                               const std::vector<cv::KeyPoint>& p_l_kpts,
                                                                               const std::vector<cv::KeyPoint>& p_r_kpts,
                                                                               const std::vector<cv::Point3f>& c_l_cam_pts,
                                                                               const std::vector<cv::Point3f>& p_l_cam_pts,
                                                                               std::vector<cv::KeyPoint>& f_c_l_kpts,
                                                                               std::vector<cv::KeyPoint>& f_c_r_kpts,
                                                                               std::vector<cv::KeyPoint>& f_p_l_kpts,
                                                                               std::vector<cv::KeyPoint>& f_p_r_kpts,
                                                                               std::vector<cv::Point3f>& f_c_l_cam_pts,
                                                                               std::vector<cv::Point3f>& f_p_l_cam_pts);


        /** \brief Estimates motion between two stereo images using keypoints and 3D-to-2D method.
         *  The user can choose between the following methods to estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param[in] l_proj_mat Left projection matrix.
         *  \param[in] r_proj_mat Right projection matrix.
         *  \param[in] l_kpts_0 Left keypoints of the first stereo image. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param[in] r_kpts_0 Right keypoints of the first stereo image. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints. 
         *  \param[in] l_kpts_1 Left keypoints of the second stereo image. In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param[in] r_kpts_1 Right keypoints of the second stereo image. In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param[in] scene_pts_0 3D points of the first stereo image. In consecutive images: 
         *  current 3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera.
         *  \param[in] scene_pts_1 3D points of the second stereo image. In consecutive images: 
         *  previous 3D points relative to left camera. In loop closing images: candidate 3D
         *  points relative to world.
         *  \param[out] r_vec Estimated rotation vector.
         *  \param[out] t_vec Estimated translation matrix.
         *  \param[out] f_l_kpts_0 Filtered left keypoints of the first stereo image. In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_0 Filtered right keypoints of the first stereo image. In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param[out] f_l_kpts_1 Filtered left keypoints of the second stereo image. In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_1 Filtered right keypoints of the second stereo image. In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first stereo image. In consecutive images:
         *  current 3D points relative to left camera filtered out. In loop closing images: query
         *  3D points relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second stereo image. In consecutive images:
         *  previous 3D points relative to left camera filtered out. In loop closing images: candidate
         *  3D points relative to world filtered out.
         */
        void estimateMotion(const cv::Mat& l_proj_mat,
                            const cv::Mat& r_proj_mat,
                            const std::vector<cv::KeyPoint>& l_kpts_0,
                            const std::vector<cv::KeyPoint>& r_kpts_0,
                            const std::vector<cv::KeyPoint>& l_kpts_1,
                            const std::vector<cv::KeyPoint>& r_kpts_1,
                            const std::vector<cv::Point3f>& scene_pts_0,
                            const std::vector<cv::Point3f>& scene_pts_1,
                            cv::Mat& r_vec,
                            cv::Mat& t_vec,
                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                            std::vector<cv::Point3f>& f_scene_pts_0,
                            std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Estimate motion between two sets of keypoints using 3D-to-2D method. These sets 
         *  must come from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the sets are 
         *  divided into clusters, the cluster_ids_0 and cluster_ids_1 variables must be filled
         *  appropriately. If the sets are not divided into clusters, the cluster_ids_0 and 
         *  cluster_ids_1 variables may be empty. The user can choose between the following methods
         *  to estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param[in] l_proj_mat Left projection matrix.
         *  \param[in] r_proj_mat Right projection matrix.
         *  \param[in] cluster_ids_0 Subset identifiers of the first set. In consecutive images:
         *   current identifiers. In loop closing images: query identifiers.
         *  \param[in] cluster_ids_1 Subset identifiers of the second set. In consecutive images: 
         *  previous identifiers. In loop closing images: candidate identifiers.
         *  \param[in] l_kpts_0 Left keypoints of the first set In consecutive images: current
         *  left keypoints. In loop closing images: query left keypoints.
         *  \param[in] r_kpts_0 Right keypoints of the first set. In consecutive images: current 
         *  right keypoints. In loop closing images: query right keypoints.
         *  \param[in] l_kpts_1 Left keypoints of the second set. In consecutive images: previous
         *  left keypoints. In loop closing images: candidate left keypoints.
         *  \param[in] r_kpts_1 Right keypoints of the second set. In consecutive images: previous
         *  right keypoints. In loop closing images: candidate right keypoints.
         *  \param[in] scene_pts_0 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera.
         *  \param[in] scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world.
         *  \param[out] r_vec Estimated rotation vector.
         *  \param[out] t_vec Estimated translation matrix.
         *  \param[out] f_cluster_ids_0 Filtered subset identifiers of the first set. In consecutive
         *  images: current identifiers filtered out. In loop closing images: query identifiers
         *  filtered out.
         *  \param[out] f_cluster_ids_1 Filtered subset identifiers of the second set In consecutive
         *  images: previous identifiers filtered out. In loop closing images: candidate identifiers 
         *  filtered out.
         *  \param[out] f_l_kpts_0 Filtered left keypoints of the first set. In consecutive images: 
         *  current left keypoints filtered out. In loop closing images: query left keypoints
         *  filtered out.
         *  \param[out] f_r_kpts_0 Filtered right keypoints of the first set. In consecutive images: 
         *  current right keypoints filtered out. In loop closing images: query right keypoints
         *  filtered out.
         *  \param[out] f_l_kpts_1 Filtered left keypoints of the second set. In consecutive images:
         *  previous left keypoints filtered out. In loop closing images: candidate left keypoints
         *  filtered out.
         *  \param[out] f_r_kpts_1 Filtered right keypoints of the second set. In consecutive images:
         *  previous right keypoints filtered out. In loop closing images: candidate right keypoints
         *  filtered out.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera filtered out. In loop closing images: query 3D points 
         *  relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera filtered out. In loop closing images: candidate 3D points 
         *  relative to world filtered out.
         */
        void estimateMotion(const cv::Mat& l_proj_mat,
                            const cv::Mat& r_proj_mat,
                            const std::vector<int>& cluster_ids_0,
                            const std::vector<int>& cluster_ids_1,
                            const std::vector<cv::KeyPoint>& l_kpts_0,
                            const std::vector<cv::KeyPoint>& r_kpts_0,
                            const std::vector<cv::KeyPoint>& l_kpts_1,
                            const std::vector<cv::KeyPoint>& r_kpts_1,
                            const std::vector<cv::Point3f>& scene_pts_0,
                            const std::vector<cv::Point3f>& scene_pts_1,
                            cv::Mat& r_vec,
                            cv::Mat& t_vec,
                            std::vector<int>& f_cluster_ids_0,
                            std::vector<int>& f_cluster_ids_1,
                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                            std::vector<cv::Point3f>& f_scene_pts_0,
                            std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Estimates motion between two stereo images using keypoints and 3D-to-2D method.
         *  The user can choose between the following methods to estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  Then the motion estimated is refined.
         *  \param[in] l_proj_mat Left projection matrix.
         *  \param[in] r_proj_mat Right projection matrix.
         *  \param[in] l_kpts_0 Left keypoints of the first stereo image. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param[in] r_kpts_0 Right keypoints of the first stereo image. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints. 
         *  \param[in] l_kpts_1 Left keypoints of the second stereo image. In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param[in] r_kpts_1 Right keypoints of the second stereo image. In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param[in] scene_pts_0 3D points of the first stereo image. In consecutive images: 
         *  current 3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera.
         *  \param[in] scene_pts_1 3D points of the second stereo image. In consecutive images: 
         *  previous 3D points relative to left camera. In loop closing images: candidate 3D
         *  points relative to world.
         *  \param[out] r_vec Estimated rotation vector.
         *  \param[out] t_vec Estimated translation matrix.
         *  \param[out] f_l_kpts_0 Filtered left keypoints of the first stereo image. In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_0 Filtered right keypoints of the first stereo image. In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param[out] f_l_kpts_1 Filtered left keypoints of the second stereo image. In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_1 Filtered right keypoints of the second stereo image. In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first stereo image. In consecutive images:
         *  current 3D points relative to left camera filtered out. In loop closing images: query
         *  3D points relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second stereo image. In consecutive images:
         *  previous 3D points relative to left camera filtered out. In loop closing images: candidate
         *  3D points relative to world filtered out.
         */
        void estimateRefinedMotion(const cv::Mat& l_proj_mat,
                                   const cv::Mat& r_proj_mat,
                                   const std::vector<cv::KeyPoint>& l_kpts_0,
                                   const std::vector<cv::KeyPoint>& r_kpts_0,
                                   const std::vector<cv::KeyPoint>& l_kpts_1,
                                   const std::vector<cv::KeyPoint>& r_kpts_1,
                                   const std::vector<cv::Point3f>& scene_pts_0,
                                   const std::vector<cv::Point3f>& scene_pts_1,
                                   cv::Mat& r_vec,
                                   cv::Mat& t_vec,
                                   std::vector<cv::KeyPoint>& r_l_kpts_0,
                                   std::vector<cv::KeyPoint>& r_r_kpts_0,
                                   std::vector<cv::KeyPoint>& r_l_kpts_1,
                                   std::vector<cv::KeyPoint>& r_r_kpts_1,
                                   std::vector<cv::Point3f>& r_scene_pts_0,
                                   std::vector<cv::Point3f>& r_scene_pts_1);


        /** \brief Estimate motion between two sets of keypoints using 3D-to-2D method. These sets 
         *  must come from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the sets are 
         *  divided into clusters, the cluster_ids_0 and cluster_ids_1 variables must be filled
         *  appropriately. If the sets are not divided into clusters, the cluster_ids_0 and 
         *  cluster_ids_1 variables may be empty. The user can choose between the following methods
         *  to estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  Then the motion estimated is refined.
         *  \param[in] l_proj_mat Left projection matrix.
         *  \param[in] r_proj_mat Right projection matrix.
         *  \param[in] cluster_ids_0 Subset identifiers of the first set. In consecutive images:
         *   current identifiers. In loop closing images: query identifiers.
         *  \param[in] cluster_ids_1 Subset identifiers of the second set. In consecutive images: 
         *  previous identifiers. In loop closing images: candidate identifiers.
         *  \param[in] l_kpts_0 Left keypoints of the first set In consecutive images: current
         *  left keypoints. In loop closing images: query left keypoints.
         *  \param[in] r_kpts_0 Right keypoints of the first set. In consecutive images: current 
         *  right keypoints. In loop closing images: query right keypoints.
         *  \param[in] l_kpts_1 Left keypoints of the second set. In consecutive images: previous
         *  left keypoints. In loop closing images: candidate left keypoints.
         *  \param[in] r_kpts_1 Right keypoints of the second set. In consecutive images: previous
         *  right keypoints. In loop closing images: candidate right keypoints.
         *  \param[in] scene_pts_0 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera.
         *  \param[in] scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world.
         *  \param[out] r_vec Estimated rotation vector.
         *  \param[out] t_vec Estimated translation matrix.
         *  \param[out] f_cluster_ids_0 Filtered subset identifiers of the first set. In consecutive
         *  images: current identifiers filtered out. In loop closing images: query identifiers
         *  filtered out.
         *  \param[out] f_cluster_ids_1 Filtered subset identifiers of the second set In consecutive
         *  images: previous identifiers filtered out. In loop closing images: candidate identifiers 
         *  filtered out.
         *  \param[out] f_l_kpts_0 Filtered left keypoints of the first set. In consecutive images: 
         *  current left keypoints filtered out. In loop closing images: query left keypoints
         *  filtered out.
         *  \param[out] f_r_kpts_0 Filtered right keypoints of the first set. In consecutive images: 
         *  current right keypoints filtered out. In loop closing images: query right keypoints
         *  filtered out.
         *  \param[out] f_l_kpts_1 Filtered left keypoints of the second set. In consecutive images:
         *  previous left keypoints filtered out. In loop closing images: candidate left keypoints
         *  filtered out.
         *  \param[out] f_r_kpts_1 Filtered right keypoints of the second set. In consecutive images:
         *  previous right keypoints filtered out. In loop closing images: candidate right keypoints
         *  filtered out.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera filtered out. In loop closing images: query 3D points 
         *  relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera filtered out. In loop closing images: candidate 3D points 
         *  relative to world filtered out.
         */
        void estimateRefinedMotion(const cv::Mat& l_proj_mat,
                                   const cv::Mat& r_proj_mat,
                                   const std::vector<int>& cluster_ids_0,
                                   const std::vector<int>& cluster_ids_1,
                                   const std::vector<cv::KeyPoint>& l_kpts_0,
                                   const std::vector<cv::KeyPoint>& r_kpts_0,
                                   const std::vector<cv::KeyPoint>& l_kpts_1,
                                   const std::vector<cv::KeyPoint>& r_kpts_1,
                                   const std::vector<cv::Point3f>& scene_pts_0,
                                   const std::vector<cv::Point3f>& scene_pts_1,
                                   cv::Mat& r_vec,
                                   cv::Mat& t_vec,
                                   std::vector<int>& f_cluster_ids_0,
                                   std::vector<int>& f_cluster_ids_1,
                                   std::vector<cv::KeyPoint>& f_l_kpts_0,
                                   std::vector<cv::KeyPoint>& f_r_kpts_0,
                                   std::vector<cv::KeyPoint>& f_l_kpts_1,
                                   std::vector<cv::KeyPoint>& f_r_kpts_1,
                                   std::vector<cv::Point3f>& f_scene_pts_0,
                                   std::vector<cv::Point3f>& f_scene_pts_1);

    
    private:

        struct SortBucketsByMatchesSlopesAscending 
        {
            inline bool operator() (const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_0, 
                                    const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_1) const
            {
                return pair_0.first < pair_1.first;
            }
        };


        /** \brief Transforms a string into the correct SolvePnPMethod value.
         *  \param[in] motion_type Desired motion estimation type name.
         *  @return a MotionEstimator::Type value.
         */
        MotionEstimator::Type selectMotionEstimationType(const std::string& motion_estimation_type);

                                    
        // Attributes:

        Type motion_estimation_type_ = AP3P; //!> Motion estimation type.

};


#endif // MOTION_ESTIMATOR_HPP