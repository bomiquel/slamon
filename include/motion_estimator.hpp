/**
 *  \file motion_estimator.hpp
 *  \brief The MotionEstimator class provides tools to compute 3D points and 6-DoF motion between stereo images.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
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


/** \class MotionEstimator
 *  \brief Provides stereo 3D triangulation, spatial keypoint bucketing, and 3D-to-2D 6-DoF motion estimation.
 *  
 *  This class implements stereo projection matrix construction, 3D point triangulation with far-point 
 *  filtering, spatial homogenization of keypoints via grid bucketing, and robust RANSAC-based 
 *  Perspective-n-Point (PnP) motion estimation and refinement.
 */
class MotionEstimator
{

    public:


        /** \enum Type
         *  \brief Available Perspective-n-Point (PnP) solver algorithms for 3D-to-2D motion estimation.
         */
        enum Type
        {
            /** \brief Efficient Perspective-n-Point (EPnP) method. */
            EPNP = cv::SOLVEPNP_EPNP,

            /** \brief Classical Perspective-3-Point (P3P) method. */
            P3P = cv::SOLVEPNP_P3P,

            /** \brief Algebraic Perspective-3-Point (AP3P) method. */
            AP3P = cv::SOLVEPNP_AP3P,
        };


        /** \brief Default class constructor.
         */
        MotionEstimator() {};


        /** \brief Parameterized class constructor.
         *  \param[in] motion_estimation_type String identifier of the desired PnP motion estimation algorithm 
         *             (e.g., `"epnp"`, `"p3p"`, `"ap3p"`).
         */
        MotionEstimator(const std::string& motion_estimation_type);


        /** \brief Builds a 3x4 camera projection matrix from intrinsic calibration parameters and baseline.
         *  \param[in] b Stereo rig baseline [m] (set to 0.0 for the left camera).
         *  \param[in] f_x Horizontal focal length [px].
         *  \param[in] f_y Vertical focal length [px].
         *  \param[in] c_x Principal point x-coordinate [px].
         *  \param[in] c_y Principal point y-coordinate [px].
         *  \return The resulting 3x4 projection matrix (`cv::Mat`).
         */
        cv::Mat buildProjectionMatrix(const double b,
                                      const double f_x, 
                                      const double f_y, 
                                      const double c_x, 
                                      const double c_y);

        
        /** \brief Triangulates 3D points from stereo correspondences and filters out distant points.
         *  
         *  Computes 3D points using OpenCV triangulation and subsequently removes points categorized 
         *  as "far". A 3D point is considered "far" when its depth (z-coordinate) exceeds 40 times 
         *  the stereo baseline, as distant points fail to provide accurate scale and translation 
         *  information (Paz et al., 2008; Mur-Artal et al., 2017).
         *  
         *  \param[in] l_proj_mat Left camera 3x4 projection matrix.
         *  \param[in] r_proj_mat Right camera 3x4 projection matrix.
         *  \param[in] l_descs Left image feature descriptors.
         *  \param[in] r_descs Right image feature descriptors.
         *  \param[in] l_kpts Left image 2D keypoints.
         *  \param[in] r_kpts Right image 2D keypoints.
         *  \param[out] f_l_descs Filtered left image descriptors.
         *  \param[out] f_r_descs Filtered right image descriptors.
         *  \param[out] f_l_kpts Filtered left image keypoints.
         *  \param[out] f_r_kpts Filtered right image keypoints.
         *  \param[out] f_l_cam_pts Filtered triangulated 3D points relative to the left camera coordinate frame.
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


        /** \brief Homogenizes the spatial distribution of keypoints across the image domain via grid bucketing.
         *  
         *  The current left image is divided into a regular grid and a maximum of `max_kpts` keypoints 
         *  per cell (bucket) is enforced. Within each bucket, keypoints are selected according to the 
         *  following procedure:
         *  1. A 2D line is computed between each feature in the bucket and its matched correspondence in the previous image.
         *  2. The slopes of these lines are sorted in ascending order.
         *  3. The median slope of the set is computed.
         *  4. The keypoints whose optical flow slopes are closest to the median slope are retained.
         *  
         *  \param[in] max_kpts Maximum number of keypoints to retain per grid bucket.
         *  \param[in] img_width Image width (number of columns) [px].
         *  \param[in] img_height Image height (number of rows) [px].
         *  \param[in] c_l_kpts Current left image keypoints.
         *  \param[in] c_r_kpts Current right image keypoints.
         *  \param[in] p_l_kpts Previous left image keypoints.
         *  \param[in] p_r_kpts Previous right image keypoints.
         *  \param[in] c_l_cam_pts Current 3D points relative to the left camera frame.
         *  \param[in] p_l_cam_pts Previous 3D points relative to the left camera frame.
         *  \param[out] f_c_l_kpts Bucketed current left keypoints.
         *  \param[out] f_c_r_kpts Bucketed current right keypoints. 
         *  \param[out] f_p_l_kpts Bucketed previous left keypoints. 
         *  \param[out] f_p_r_kpts Bucketed previous right keypoints. 
         *  \param[out] f_c_l_cam_pts Bucketed current 3D points relative to the left camera frame.
         *  \param[out] f_p_l_cam_pts Bucketed previous 3D points relative to the left camera frame.
         *  \return Vector of grid buckets containing pairs of (keypoint index, `cv::KeyPoint`).
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


        /** \brief Estimates 6-DoF motion between two stereo frames using a 3D-to-2D PnP method within a RANSAC scheme.
         *  
         *  The user can choose among the following PnP solvers (via `MotionEstimator::Type`):
         *  - `P3P`
         *  - `AP3P`
         *  - `EPNP`
         *  
         *  \param[in] l_proj_mat Left camera 3x4 projection matrix.
         *  \param[in] r_proj_mat Right camera 3x4 projection matrix.
         *  \param[in] l_kpts_0 Left keypoints of the first stereo frame (current in odometry; query in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first stereo frame (current in odometry; query in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second stereo frame (previous in odometry; candidate in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second stereo frame (previous in odometry; candidate in loop closure).
         *  \param[in] scene_pts_0 3D points of the first stereo frame (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second stereo frame (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] r_vec Estimated 3x1 Rodrigues rotation vector.
         *  \param[out] t_vec Estimated 3x1 translation vector.
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first stereo frame.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first stereo frame.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second stereo frame.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second stereo frame.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first stereo frame.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second stereo frame.
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


        /** \brief Estimates 6-DoF motion between two sets of keypoint clusters using a 3D-to-2D PnP method within a RANSAC scheme.
         *  
         *  These sets must come from at least two different stereo pairs and may consist of multiple
         *  keypoint clusters belonging to different stereo frames. If the sets are divided into clusters,
         *  `cluster_ids_0` and `cluster_ids_1` must be populated accordingly; otherwise, they may be left empty.
         *  
         *  The user can choose among the following PnP solvers (via `MotionEstimator::Type`):
         *  - `P3P`
         *  - `AP3P`
         *  - `EPNP`
         *  
         *  \param[in] l_proj_mat Left camera 3x4 projection matrix.
         *  \param[in] r_proj_mat Right camera 3x4 projection matrix.
         *  \param[in] cluster_ids_0 Cluster IDs of the first set (current in odometry; query in loop closure).
         *  \param[in] cluster_ids_1 Cluster IDs of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] l_kpts_0 Left keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] scene_pts_0 3D points of the first set (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second set (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] r_vec Estimated 3x1 Rodrigues rotation vector.
         *  \param[out] t_vec Estimated 3x1 translation vector.
         *  \param[out] f_cluster_ids_0 Filtered (inlier) cluster IDs of the first set.
         *  \param[out] f_cluster_ids_1 Filtered (inlier) cluster IDs of the second set.
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first set.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first set.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second set.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second set.
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


        /** \brief Estimates and refines 6-DoF motion between two stereo frames using a 3D-to-2D PnP method within a RANSAC scheme.
         *  
         *  The user can choose among the following PnP solvers (via `MotionEstimator::Type`):
         *  - `P3P`
         *  - `AP3P`
         *  - `EPNP`
         *  
         *  After initial RANSAC PnP estimation, the pose is refined over the inlier set.
         *  
         *  \param[in] l_proj_mat Left camera 3x4 projection matrix.
         *  \param[in] r_proj_mat Right camera 3x4 projection matrix.
         *  \param[in] l_kpts_0 Left keypoints of the first stereo frame (current in odometry; query in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first stereo frame (current in odometry; query in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second stereo frame (previous in odometry; candidate in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second stereo frame (previous in odometry; candidate in loop closure).
         *  \param[in] scene_pts_0 3D points of the first stereo frame (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second stereo frame (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] r_vec Estimated and refined 3x1 Rodrigues rotation vector.
         *  \param[out] t_vec Estimated and refined 3x1 translation vector.
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first stereo frame.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first stereo frame.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second stereo frame.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second stereo frame.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first stereo frame.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second stereo frame.
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
                                   std::vector<cv::KeyPoint>& f_l_kpts_0,
                                   std::vector<cv::KeyPoint>& f_r_kpts_0,
                                   std::vector<cv::KeyPoint>& f_l_kpts_1,
                                   std::vector<cv::KeyPoint>& f_r_kpts_1,
                                   std::vector<cv::Point3f>& f_scene_pts_0,
                                   std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Estimates and refines 6-DoF motion between two sets of keypoint clusters using a 3D-to-2D PnP method within a RANSAC scheme.
         *  
         *  These sets must come from at least two different stereo pairs and may consist of multiple
         *  keypoint clusters belonging to different stereo frames. If the sets are divided into clusters,
         *  `cluster_ids_0` and `cluster_ids_1` must be populated accordingly; otherwise, they may be left empty.
         *  
         *  The user can choose among the following PnP solvers (via `MotionEstimator::Type`):
         *  - `P3P`
         *  - `AP3P`
         *  - `EPNP`
         *  
         *  After initial RANSAC PnP estimation, the pose is refined over the inlier set.
         *  
         *  \param[in] l_proj_mat Left camera 3x4 projection matrix.
         *  \param[in] r_proj_mat Right camera 3x4 projection matrix.
         *  \param[in] cluster_ids_0 Cluster IDs of the first set (current in odometry; query in loop closure).
         *  \param[in] cluster_ids_1 Cluster IDs of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] l_kpts_0 Left keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] scene_pts_0 3D points of the first set (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second set (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] r_vec Estimated and refined 3x1 Rodrigues rotation vector.
         *  \param[out] t_vec Estimated and refined 3x1 translation vector.
         *  \param[out] f_cluster_ids_0 Filtered (inlier) cluster IDs of the first set.
         *  \param[out] f_cluster_ids_1 Filtered (inlier) cluster IDs of the second set.
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first set.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first set.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second set.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second set.
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

        /** \struct SortBucketsByMatchesSlopesAscending
         *  \brief Comparator functor to sort bucketed keypoints in ascending order of their optical flow line slope.
         */
        struct SortBucketsByMatchesSlopesAscending 
        {
            /** \brief Compares the slope values of two bucketed keypoint entries.
             *  \param[in] pair_0 First pair (slope, (keypoint index, `cv::KeyPoint`)).
             *  \param[in] pair_1 Second pair (slope, (keypoint index, `cv::KeyPoint`)).
             *  \return `true` if the slope of `pair_0` is strictly less than that of `pair_1`, `false` otherwise.
             */
            inline bool operator() (const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_0, 
                                    const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_1) const
            {
                return pair_0.first < pair_1.first;
            }
        };


        /** \brief Converts a motion estimation type string into the corresponding `MotionEstimator::Type` enum value.
         *  \param[in] motion_estimation_type String identifier of the desired motion estimation algorithm.
         *  \return The corresponding `MotionEstimator::Type` value.
         */
        MotionEstimator::Type selectMotionEstimationType(const std::string& motion_estimation_type);


        /** \brief Selected Perspective-n-Point (PnP) motion estimation algorithm. */
        Type motion_estimation_type_ = AP3P;

};


#endif // MOTION_ESTIMATOR_HPP