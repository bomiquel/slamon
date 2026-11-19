/**
 *  \file keypoint_matcher.hpp
 *  \brief The KeypointMatcher class provides tools for visual keypoint matching and outlier rejection.
 *
 *  \author Bo Miquel Nordfeldt-Fiol (2024–2026)
 *
 *  \copyright BSD-4-Clause
 *  This file is part of the SLAMON project (package: slamon) and is released under
 *  the BSD 4-Clause License. See the top-level LICENSE file for details.
 */

 
#ifndef KEYPOINT_MATCHER_HPP
#define KEYPOINT_MATCHER_HPP

#include <vector>
#include <string>
#include <thread>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>


/** \class KeypointMatcher
 *  \brief Matches 2D visual keypoints and filters out outliers across stereo and temporal image pairs.
 *  
 *  This class implements intra-frame stereo matching (using Lowe's ratio test and epipolar constraints)
 *  as well as inter-frame (temporal or loop closure) matching using plain or circular stereo consistency
 *  combined with Lowe's ratio test or RANSAC-based homography filtering.
 */
class KeypointMatcher
{

    public:

        /** \enum Type
         *  \brief Available keypoint matching and outlier rejection strategies.
         *  
         *  For circular matching methods, both left and right stereo images are required.
         */ 
        enum Type
        {
            /** \brief Left-only matching filtered with Lowe's ratio test. */
            RATIO_MATCHING               = 0,

            /** \brief Left-only matching filtered with RANSAC homography. */
            HOMOGRAPHY_MATCHING          = 1,

            /** \brief Circular stereo matching filtered with Lowe's ratio test. */
            CIRCULAR_RATIO_MATCHING      = 2,

            /** \brief Circular stereo matching filtered with RANSAC homography. */
            CIRCULAR_HOMOGRAPHY_MATCHING = 3
        };


        /** \brief Default class constructor.
         */
        KeypointMatcher() {}


        /** \brief Parameterized class constructor.
         *  \param[in] matching_type String identifier of the desired matching strategy 
         *             (e.g., `"ratio_matching"`, `"circular_homography_matching"`).
         */
        KeypointMatcher(const std::string& matching_type);


        /** \brief Matches keypoints within the same stereo pair and removes outliers.
         *  
         *  To eliminate outliers, Lowe's ratio test is applied first, followed by an
         *  epipolar constraint enforcing a maximum vertical disparity error of 1.0 pixel.
         *  
         *  \param[in] l_descs Left image feature descriptors.
         *  \param[in] r_descs Right image feature descriptors.
         *  \param[in] l_kpts Left image 2D keypoints.
         *  \param[in] r_kpts Right image 2D keypoints.
         *  \param[out] f_l_descs Filtered (inlier) left image descriptors.
         *  \param[out] f_r_descs Filtered (inlier) right image descriptors.
         *  \param[out] f_l_kpts Filtered (inlier) left image keypoints.
         *  \param[out] f_r_kpts Filtered (inlier) right image keypoints.
         */
        void matchStereoKeypoints(const cv::Mat& l_descs,
                                  const cv::Mat& r_descs,
                                  const std::vector<cv::KeyPoint>& l_kpts,
                                  const std::vector<cv::KeyPoint>& r_kpts,
                                  cv::Mat& f_l_descs,
                                  cv::Mat& f_r_descs,
                                  std::vector<cv::KeyPoint>& f_l_kpts,
                                  std::vector<cv::KeyPoint>& f_r_kpts);


        /** \brief Matches visual keypoints between two different stereo frames.
         *  
         *  To remove outliers after descriptor matching, the user can choose among the following
         *  techniques (via `KeypointMatcher::Type`):
         *  - Plain Lowe's ratio test.
         *  - Plain homography + RANSAC.
         *  - Circular Lowe's ratio test.
         *  - Circular homography + RANSAC.
         *  
         *  "Plain" methods involve only left keypoints, whereas "Circular" methods enforce consistency 
         *  across both left and right stereo keypoints.
         *  
         *  \param[in] l_descs_0 Left descriptors of the first stereo frame (current frame in odometry; query frame in loop closure).
         *  \param[in] r_descs_0 Right descriptors of the first stereo frame (current frame in odometry; query frame in loop closure).
         *  \param[in] l_descs_1 Left descriptors of the second stereo frame (previous frame in odometry; candidate frame in loop closure).
         *  \param[in] r_descs_1 Right descriptors of the second stereo frame (previous frame in odometry; candidate frame in loop closure).
         *  \param[in] l_kpts_0 Left keypoints of the first stereo frame (current frame in odometry; query frame in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first stereo frame (current frame in odometry; query frame in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second stereo frame (previous frame in odometry; candidate frame in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second stereo frame (previous frame in odometry; candidate frame in loop closure).
         *  \param[in] scene_pts_0 3D points of the first stereo frame (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second stereo frame (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first stereo frame.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first stereo frame.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second stereo frame.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second stereo frame.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first stereo frame.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second stereo frame.
         */
        void matchTemporalKeypoints(const cv::Mat& l_descs_0,
                                    const cv::Mat& r_descs_0,
                                    const cv::Mat& l_descs_1,
                                    const cv::Mat& r_descs_1,
                                    const std::vector<cv::KeyPoint>& l_kpts_0,
                                    const std::vector<cv::KeyPoint>& r_kpts_0,
                                    const std::vector<cv::KeyPoint>& l_kpts_1,
                                    const std::vector<cv::KeyPoint>& r_kpts_1,
                                    const std::vector<cv::Point3f>& scene_pts_0,
                                    const std::vector<cv::Point3f>& scene_pts_1,
                                    std::vector<cv::KeyPoint>& f_l_kpts_0,
                                    std::vector<cv::KeyPoint>& f_r_kpts_0,
                                    std::vector<cv::KeyPoint>& f_l_kpts_1,
                                    std::vector<cv::KeyPoint>& f_r_kpts_1,
                                    std::vector<cv::Point3f>& f_scene_pts_0,
                                    std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Matches visual keypoints between two different sets of keypoint clusters.
         *  
         *  These sets must come from at least two different stereo pairs and may consist of multiple
         *  keypoint clusters belonging to different stereo frames. If the sets are divided into clusters,
         *  `cluster_ids_0` and `cluster_ids_1` must be populated accordingly; otherwise, they may be left empty.
         *  
         *  To remove outliers after descriptor matching, the user can choose among the following
         *  techniques (via `KeypointMatcher::Type`):
         *  - Plain Lowe's ratio test.
         *  - Plain homography + RANSAC.
         *  - Circular Lowe's ratio test.
         *  - Circular homography + RANSAC.
         *  
         *  "Plain" methods involve only left keypoints, whereas "Circular" methods enforce consistency 
         *  across both left and right stereo keypoints.
         *  
         *  \param[in] l_descs_0 Left descriptors of the first set (current in odometry; query in loop closure).
         *  \param[in] r_descs_0 Right descriptors of the first set (current in odometry; query in loop closure).
         *  \param[in] l_descs_1 Left descriptors of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] r_descs_1 Right descriptors of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] cluster_ids_0 Cluster IDs of the first set (current in odometry; query in loop closure).
         *  \param[in] cluster_ids_1 Cluster IDs of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] l_kpts_0 Left keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] r_kpts_0 Right keypoints of the first set (current in odometry; query in loop closure).
         *  \param[in] l_kpts_1 Left keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] r_kpts_1 Right keypoints of the second set (previous in odometry; candidate in loop closure).
         *  \param[in] scene_pts_0 3D points of the first set (relative to the left camera frame).
         *  \param[in] scene_pts_1 3D points of the second set (relative to the left camera in odometry; relative to the world frame in loop closure).
         *  \param[out] f_cluster_ids_0 Filtered (inlier) cluster IDs of the first set.
         *  \param[out] f_cluster_ids_1 Filtered (inlier) cluster IDs of the second set.
         *  \param[out] f_l_kpts_0 Filtered (inlier) left keypoints of the first set.
         *  \param[out] f_r_kpts_0 Filtered (inlier) right keypoints of the first set.
         *  \param[out] f_l_kpts_1 Filtered (inlier) left keypoints of the second set.
         *  \param[out] f_r_kpts_1 Filtered (inlier) right keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second set.
         */
        void matchTemporalClusters(const cv::Mat& l_descs_0,
                                   const cv::Mat& r_descs_0,
                                   const cv::Mat& l_descs_1,
                                   const cv::Mat& r_descs_1,
                                   const std::vector<int>& cluster_ids_0,
                                   const std::vector<int>& cluster_ids_1,
                                   const std::vector<cv::KeyPoint>& l_kpts_0,
                                   const std::vector<cv::KeyPoint>& r_kpts_0,
                                   const std::vector<cv::KeyPoint>& l_kpts_1,
                                   const std::vector<cv::KeyPoint>& r_kpts_1,
                                   const std::vector<cv::Point3f>& scene_pts_0,
                                   const std::vector<cv::Point3f>& scene_pts_1,
                                   std::vector<int>& f_cluster_ids_0,
                                   std::vector<int>& f_cluster_ids_1,
                                   std::vector<cv::KeyPoint>& f_l_kpts_0,
                                   std::vector<cv::KeyPoint>& f_r_kpts_0,
                                   std::vector<cv::KeyPoint>& f_l_kpts_1,
                                   std::vector<cv::KeyPoint>& f_r_kpts_1,
                                   std::vector<cv::Point3f>& f_scene_pts_0,
                                   std::vector<cv::Point3f>& f_scene_pts_1);

    protected:

        /** \brief Selected keypoint matching and outlier rejection strategy. */
        KeypointMatcher::Type matching_type_ = RATIO_MATCHING;


    private:


        /** \struct PointHash
         *  \brief Custom hash functor for `cv::Point2f` to be used in unordered associative containers.
         *  
         *  Combines the hash values of the `x` and `y` coordinates using a bitwise XOR and bit-shift
         *  to minimize hash collisions.
         */
        struct PointHash
        {
            /** \brief Computes the hash value for a 2D point.
             *  \param[in] p Input 2D point.
             *  \return Computed hash value.
             */
            std::size_t operator() (const cv::Point2f& p) const
            {
                std::size_t h0 = std::hash<float>{}(p.x);
                std::size_t h1 = std::hash<float>{}(p.y);
                return h0 ^ (h1 << 1);
            }
        };


        /** \struct PointEqual
         *  \brief Equality comparator functor for `cv::Point2f`.
         *  
         *  Required by `std::unordered_map` to resolve hash collisions by comparing exact 2D coordinates.
         */
        struct PointEqual
        {
            /** \brief Checks whether two 2D points have identical coordinates.
             *  \param[in] a First 2D point.
             *  \param[in] b Second 2D point.
             *  \return `true` if both coordinates are equal, `false` otherwise.
             */
            bool operator() (const cv::Point2f& a, const cv::Point2f& b) const
            {
                return a.x == b.x && a.y == b.y;
            }
        };


        /** \brief Performs brute-force k-NN (k = 2) descriptor matching between two sets.
         *  \param[in] descs_0 Feature descriptors of the first set.
         *  \param[in] descs_1 Feature descriptors of the second set.
         *  \return Vector of k-NN descriptor matches (`cv::DMatch`).
         */
        std::vector<std::vector<cv::DMatch>> matching(const cv::Mat& descs_0, 
                                                      const cv::Mat& descs_1);


        /** \brief Matches keypoints between two sets and filters outliers using Lowe's ratio test.
         *  
         *  If the sets are divided into clusters, `cluster_ids_0` and `cluster_ids_1` must be populated
         *  appropriately; otherwise, they may be left empty. Similarly, `scene_pts_0` and `scene_pts_1`
         *  may be empty when matching keypoints within a single stereo pair prior to triangulation.
         *  
         *  \param[in] descs_0 Descriptors of the first set.
         *  \param[in] descs_1 Descriptors of the second set.
         *  \param[in] cluster_ids_0 Cluster IDs of the first set.
         *  \param[in] cluster_ids_1 Cluster IDs of the second set.
         *  \param[in] kpts_0 Keypoints of the first set.
         *  \param[in] kpts_1 Keypoints of the second set.
         *  \param[in] scene_pts_0 3D points of the first set.
         *  \param[in] scene_pts_1 3D points of the second set.
         *  \param[out] f_descs_0 Filtered (inlier) descriptors of the first set.
         *  \param[out] f_descs_1 Filtered (inlier) descriptors of the second set.
         *  \param[out] f_cluster_ids_0 Filtered (inlier) cluster IDs of the first set.
         *  \param[out] f_cluster_ids_1 Filtered (inlier) cluster IDs of the second set.
         *  \param[out] f_kpts_0 Filtered (inlier) keypoints of the first set.
         *  \param[out] f_kpts_1 Filtered (inlier) keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second set.
         */
        void ratioMatching(const cv::Mat& descs_0, 
                           const cv::Mat& descs_1,
                           const std::vector<int>& cluster_ids_0,
                           const std::vector<int>& cluster_ids_1,
                           const std::vector<cv::KeyPoint>& kpts_0,
                           const std::vector<cv::KeyPoint>& kpts_1,
                           const std::vector<cv::Point3f>& scene_pts_0,
                           const std::vector<cv::Point3f>& scene_pts_1,
                           cv::Mat& f_descs_0,
                           cv::Mat& f_descs_1,
                           std::vector<int>& f_cluster_ids_0,
                           std::vector<int>& f_cluster_ids_1,
                           std::vector<cv::KeyPoint>& f_kpts_0,
                           std::vector<cv::KeyPoint>& f_kpts_1,
                           std::vector<cv::Point3f>& f_scene_pts_0,
                           std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Matches keypoints between two sets and filters outliers using a RANSAC homography.
         *  
         *  If the sets are divided into clusters, `cluster_ids_0` and `cluster_ids_1` must be populated
         *  appropriately; otherwise, they may be left empty.
         *  
         *  \param[in] descs_0 Descriptors of the first set.
         *  \param[in] descs_1 Descriptors of the second set.
         *  \param[in] cluster_ids_0 Cluster IDs of the first set.
         *  \param[in] cluster_ids_1 Cluster IDs of the second set.
         *  \param[in] kpts_0 Keypoints of the first set.
         *  \param[in] kpts_1 Keypoints of the second set.
         *  \param[in] scene_pts_0 3D points of the first set.
         *  \param[in] scene_pts_1 3D points of the second set.
         *  \param[out] f_descs_0 Filtered (inlier) descriptors of the first set.
         *  \param[out] f_descs_1 Filtered (inlier) descriptors of the second set.
         *  \param[out] f_cluster_ids_0 Filtered (inlier) cluster IDs of the first set.
         *  \param[out] f_cluster_ids_1 Filtered (inlier) cluster IDs of the second set.
         *  \param[out] f_kpts_0 Filtered (inlier) keypoints of the first set.
         *  \param[out] f_kpts_1 Filtered (inlier) keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered (inlier) 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered (inlier) 3D points of the second set.
         */
        void geometricalMatching(const cv::Mat& descs_0, 
                                 const cv::Mat& descs_1,
                                 const std::vector<int>& cluster_ids_0,
                                 const std::vector<int>& cluster_ids_1,
                                 const std::vector<cv::KeyPoint>& kpts_0,
                                 const std::vector<cv::KeyPoint>& kpts_1,
                                 const std::vector<cv::Point3f>& scene_pts_0,
                                 const std::vector<cv::Point3f>& scene_pts_1,
                                 cv::Mat& f_descs_0,
                                 cv::Mat& f_descs_1,
                                 std::vector<int>& f_cluster_ids_0,
                                 std::vector<int>& f_cluster_ids_1,
                                 std::vector<cv::KeyPoint>& f_kpts_0,
                                 std::vector<cv::KeyPoint>& f_kpts_1,
                                 std::vector<cv::Point3f>& f_scene_pts_0,
                                 std::vector<cv::Point3f>& f_scene_pts_1);


        /** \brief Assembles four pairwise matchings (two stereo and two temporal) into consistent circular matches.
         *  
         *  Uses hash maps (`std::unordered_map`) for O(1) average lookup time, reducing the circular 
         *  consistency verification complexity from O(N^4) to O(N).
         *  
         *  \param[in] stereo_kpts_0_lowest Stereo-matched keypoints of the first set belonging to the side (left/right) with the lowest number of temporal matches.
         *  \param[in] stereo_kpts_0_highest Stereo-matched keypoints of the first set belonging to the side (left/right) with the highest number of temporal matches.
         *  \param[in] stereo_kpts_1_lowest Stereo-matched keypoints of the second set belonging to the side (left/right) with the lowest number of temporal matches.
         *  \param[in] stereo_kpts_1_highest Stereo-matched keypoints of the second set belonging to the side (left/right) with the highest number of temporal matches.
         *  \param[in] temporal_kpts_0_lowest Temporally matched keypoints of the first set from the side with the lowest number of matches.
         *  \param[in] temporal_kpts_0_highest Temporally matched keypoints of the first set from the side with the highest number of matches.
         *  \param[in] temporal_kpts_1_lowest Temporally matched keypoints of the second set from the side with the lowest number of matches.
         *  \param[in] temporal_kpts_1_highest Temporally matched keypoints of the second set from the side with the highest number of matches.
         *  \param[in] temporal_scene_pts_0 Temporally matched 3D points of the first set corresponding to the lowest-count side.
         *  \param[in] temporal_scene_pts_1 Temporally matched 3D points of the second set corresponding to the lowest-count side.
         *  \param[in] temporal_cluster_ids_0 Temporally matched cluster IDs of the first set corresponding to the lowest-count side.
         *  \param[in] temporal_cluster_ids_1 Temporally matched cluster IDs of the second set corresponding to the lowest-count side.
         *  \param[out] f_kpts_0_lowest Circularly filtered `temporal_kpts_0_lowest`.
         *  \param[out] f_kpts_0_highest Circularly filtered `temporal_kpts_0_highest`.
         *  \param[out] f_kpts_1_lowest Circularly filtered `temporal_kpts_1_lowest`.
         *  \param[out] f_kpts_1_highest Circularly filtered `temporal_kpts_1_highest`.
         *  \param[out] f_scene_pts_0 Circularly filtered `temporal_scene_pts_0`.
         *  \param[out] f_scene_pts_1 Circularly filtered `temporal_scene_pts_1`.
         *  \param[out] f_cluster_ids_0 Circularly filtered `temporal_cluster_ids_0`.
         *  \param[out] f_cluster_ids_1 Circularly filtered `temporal_cluster_ids_1`.
         */
        void assembleCircularMatchings(const std::vector<cv::KeyPoint>& stereo_kpts_0_lowest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_0_highest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_1_lowest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_1_highest,
                                       const std::vector<cv::KeyPoint>& temporal_kpts_0_lowest,
                                       const std::vector<cv::KeyPoint>& temporal_kpts_0_highest,
                                       const std::vector<cv::KeyPoint>& temporal_kpts_1_lowest,
                                       const std::vector<cv::KeyPoint>& temporal_kpts_1_highest,
                                       const std::vector<cv::Point3f>& temporal_scene_pts_0,
                                       const std::vector<cv::Point3f>& temporal_scene_pts_1,
                                       const std::vector<int>& temporal_cluster_ids_0,
                                       const std::vector<int>& temporal_cluster_ids_1,
                                       std::vector<cv::KeyPoint>& f_kpts_0_lowest,
                                       std::vector<cv::KeyPoint>& f_kpts_0_highest,
                                       std::vector<cv::KeyPoint>& f_kpts_1_lowest,
                                       std::vector<cv::KeyPoint>& f_kpts_1_highest,
                                       std::vector<cv::Point3f>& f_scene_pts_0,
                                       std::vector<cv::Point3f>& f_scene_pts_1,
                                       std::vector<int>& f_cluster_ids_0,
                                       std::vector<int>& f_cluster_ids_1);


};


#endif // KEYPOINT_MATCHER_HPP