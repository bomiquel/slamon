/**
 *  @brief The KeypointMatcher class provides tools to match keypoints and filter out outliers.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2024-2025
 */

#ifndef KEYPOINT_MATCHER_HPP
#define KEYPOINT_MATCHER_HPP

#include <vector>
#include <string>
#include <thread>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>


class KeypointMatcher
{

    public:

        /** \brief Matching types between images.
         *  For circular methods, stereo images are required.
         */ 
        enum Type
        {
            RATIO_MATCHING               = 0,
            HOMOGRAPHY_MATCHING          = 1,
            CIRCULAR_RATIO_MATCHING      = 2,
            CIRCULAR_HOMOGRAPHY_MATCHING = 3
        };


        /** \brief Empty class constructor
         */
        KeypointMatcher() {};


        /** \brief Class constructor. 
         *  \param[in] matching_type Desired matching type name.
         */
        KeypointMatcher(const double ratio,
                        const std::string& matching_type);


        /** \brief Matches keypoints of the same stereo pair and removes outliers. To
         *  eliminate outliers, the Lowe's ratio is applied first, followed by an
         *  epipolar constraint with a maximum error of 1 pixel.
         *  \param[in] l_descs Left image descriptors.
         *  \param[in] r_descs Right image descriptors.
         *  \param[in] l_kpts Left image keypoints.
         *  \param[in] r_kpts Right image keypoints.
         *  \param[out] f_l_descs Filtered left image descriptors.
         *  \param[out] f_r_descs Filtered right image descriptors.
         *  \param[out] f_l_kpts Filtered left image keypoints.
         *  \param[out] f_r_kpts Filtered right image keypoints.
         */
        void matchStereoKeypoints(const cv::Mat& l_descs,
                                  const cv::Mat& r_descs,
                                  const std::vector<cv::KeyPoint>& l_kpts,
                                  const std::vector<cv::KeyPoint>& r_kpts,
                                  cv::Mat& f_l_descs,
                                  cv::Mat& f_r_descs,
                                  std::vector<cv::KeyPoint>& f_l_kpts,
                                  std::vector<cv::KeyPoint>& f_r_kpts);


        /** \brief Matches visual keypoints of two different stereo images. To remove
         *  outliers, after matching, the user can choose between these different
         *  techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *  "Plain" involves only left keypoints, "Circular" left and right keypoints.
         *  To select the desired outlier remover, the appropriate matcher (enum type:
         *  MatchingType) must be chosen.
         *  \param[in] l_descs_0 Left descriptors of the first stereo image. In consecutive 
         *  images: current left descriptors. In loop closing images: query left descriptors.
         *  \param[in] r_descs_0 Right descriptors of the first stereo image. In consecutive
         *  images: current right descriptors. In loop closing images: query right descriptors.
         *  \param[in] l_descs_1 Left descriptors of the second stereo image. In consecutive 
         *  images: previous left descriptors. In loop closing images: candidate left descriptors.
         *  \param[in] r_descs_1 Right descriptors of the second stereo image. In consecutive
         *  images: previous left descriptors. In loop closing images: candidate left descriptors.
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
         *  previous 3D points relative to left camera. In loop closing images: candidate
         *  3D points relative to world.
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
         *  current 3D points relative to left camera filtered out. In loop closing images: query 3D
         *  points relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second stereo image. In consecutive images: 
         *  previous 3D points relative to left camera filtered out. In loop closing images: candidate
         *  3D points relative to world filtered out.
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


        /** \brief Matches the visual keypoints of two different sets. These sets must come
         *  from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the
         *  sets are divided into clusters, the cluster_ids_0 and cluster_ids_1 variables
         *  must be filled appropriately. If the sets are not divided into clusters, the
         *  cluster_ids_0 and cluster_ids_1 variables may be empty. To remove outliers, after
         *  matching, the user can choose between these different techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *  "Plain" involves only left keypoints, "Circular" left and right keypoints. To 
         *  select the desired outlier remover, the appropriate matcher (enum type: MatchingType)
         *  must be chosen.
         *  \param[in] l_descs_0 Left descriptors of the first set. In consecutive images:
         *  current left descriptors. In loop closing images: query left descriptors.
         *  \param[in] r_descs_0 Right descriptors of the first set. In consecutive images:
         *  current right descriptors. In loop closing images: query right descriptors.
         *  \param[in] l_descs_1 Left descriptors of the second set. In consecutive images:
         *  previous left descriptors. In loop closing images: candidate left descriptors.
         *  \param[in] r_descs_1 Right descriptors of the second set. In consecutive images:
         *  previous left descriptors. In loop closing images: candidate left descriptors.
         *  \param[in] cluster_ids_0 Subset identifiers of the first set. In consecutive
         *  images: current identifiers. In loop closing images: query identifiers.
         *  \param[in] cluster_ids_1 Subset identifiers of the second set. In consecutive
         *  images: previous identifiers. In loop closing images: candidate identifiers.
         *  \param[in] l_kpts_0 Left keypoints of the first set. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param[in] r_kpts_0 Right keypoints of the first set. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints.
         *  \param[in] l_kpts_1 Left keypoints of the second set. In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param[in] r_kpts_1 Right keypoints of the second set. In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param[in] scene_pts_0 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera.
         *  \param[in] scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world.
         *  \param[out] f_cluster_ids_0 Filtered subset identifiers of the first set. In
         *  consecutive images: current identifiers filtered out. In loop closing images: 
         *  query identifiers filtered out.
         *  \param[out] f_cluster_ids_1 Filtered subset identifiers of the second set. In
         *  consecutive images: previous identifiers filtered out. In loop closing images:
         *  candidate identifiers filtered out.
         *  \param[out] f_l_kpts_0 Filtered left keypoints of the first set. In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_0 Filtered right keypoints of the first set. In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param[out] f_l_kpts_1 Filtered left keypoints of the second set. In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param[out] f_r_kpts_1 Filtered right keypoints of the second set. In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera filtered out. In loop closing images: query 3D points 
         *  relative to left camera filtered out.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera filtered out. In loop closing images: candidate 3D points 
         *  relative to world filtered out.
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

        // Attributes:

        KeypointMatcher::Type matching_type_ = RATIO_MATCHING; //!> Matching type.


    private:


        /** \brief Matches visual keypoints.
         *  \param[in] descs_0 Descriptors of the first set.
         *  \param[in] descs_1 Descriptors of the second set.
         *  @return The matches.
         */
        std::vector<std::vector<cv::DMatch>> matching(const cv::Mat& descs_0, 
                                                      const cv::Mat& descs_1);


        /** \brief Matches the visual keypoints of two different sets. These sets must come
         *  from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the
         *  sets are divided into clusters, the cluster_ids_0 and cluster_ids_1 variables
         *  must be filled appropriately. If the sets are not divided into clusters, the
         *  cluster_ids_0 and cluster_ids_1 variables may be empty. After matching, Lowe's
         *  ratio is applied to eliminate outliers.
         *  \param[in] descs_0 Descriptors of the first set.
         *  \param[in] descs_1 Descriptors of the second set.
         *  \param[in] cluster_ids_0 Subset identifier of the first set.
         *  \param[in] cluster_ids_1 Subset identifier of the second set.
         *  \param[in] kpts_0 Keypoints of the first set.
         *  \param[in] kpts_1 Keypoints of the second set.
         *  \param[in] scene_pts_0 3D points of the first set.
         *  \param[in] scene_pts_1 3D points of the second set.
         *  \param[out] f_descs_0 Filtered descriptors of the first set.
         *  \param[out] f_descs_1 Filtered descriptors of the second set.
         *  \param[out] f_cluster_ids_0 Filtered subset identifier of the first set.
         *  \param[out] f_cluster_ids_1 Filtered subset identifier of the second set.
         *  \param[out] f_kpts_0 Filtered keypoints of the first set.
         *  \param[out] f_kpts_1 Filtered keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second set.
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


        /** \brief Matches the visual keypoints of two different sets. These sets must come
         *  from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the
         *  sets are divided into clusters, the cluster_ids_0 and cluster_ids_1 variables
         *  must be filled appropriately. If the sets are not divided into clusters, the
         *  cluster_ids_0 and cluster_ids_1 variables may be empty. After matching, a 
         *  homography wrapped in a RANSAC scheme is applied to eliminate outliers.
         *  \param[in] descs_0 Descriptors of the first set.
         *  \param[in] descs_1 Descriptors of the second set.
         *  \param[in] cluster_ids_0 Subset identifier of the first set.
         *  \param[in] cluster_ids_1 Subset identifier of the second set.
         *  \param[in] kpts_0 Keypoints of the first set.
         *  \param[in] kpts_1 Keypoints of the second set.
         *  \param[in] scene_pts_0 3D points of the first set.
         *  \param[in] scene_pts_1 3D points of the second set.
         *  \param[out] f_descs_0 Filtered descriptors of the first set.
         *  \param[out] f_descs_1 Filtered descriptors of the second set.
         *  \param[out] f_cluster_ids_0 Filtered subset identifier of the first set.
         *  \param[out] f_cluster_ids_1 Filtered subset identifier of the second set.
         *  \param[out] f_kpts_0 Filtered keypoints of the first set.
         *  \param[out] f_kpts_1 Filtered keypoints of the second set.
         *  \param[out] f_scene_pts_0 Filtered 3D points of the first set.
         *  \param[out] f_scene_pts_1 Filtered 3D points of the second set.
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


        /** \brief Assemble four matchings to build circular matches.
         *  \param[in] stereo_kpts_0_lowest Keypoints of the first set matched in stereo matching.
         *  Must belong to the side (left/right) with the lowest number of keypoints after the
         *  temporal matching.
         *  \param[in] stereo_kpts_0_highest Keypoints of the first set matched in stereo matching.
         *  Must belong to the side (left/right) with the highest number of keypoints after the
         *  temporal matching.
         *  \param[in] stereo_kpts_1_lowest Keypoints of the second set matched in stereo matching.
         *  Must belong to the side (left/right) with the lowest number of keypoints after the
         *  temporal matching.
         *  \param[in] stereo_kpts_1_highest Keypoints of the second set matched in stereo matching.
         *  Must belong to the side (left/right) with the highest number of keypoints after the
         *  temporal matching.
         *  \param[in] temporal_kpts_0_lowest Keypoint vector of the first set with the lowest 
         *  number of keypoints after temporal matching.
         *  \param[in] temporal_kpts_0_highest Keypoint vector of the first set with the highest
         *  number of keypoints after temporal matching.
         *  \param[in] temporal_kpts_1_lowest Keypoint vector of the second set with the lowest 
         *  number of keypoints after temporal matching.
         *  \param[in] temporal_kpts_1_highest Keypoint vector of the second set with the highest
         *  number of keypoints after temporal matching.
         *  \param[in] temporal_scene_pts_0 3D points vector of the first set with the lowest number
         *  of 3D points after temporal matching.
         *  \param[in] temporal_scene_pts_1 3D points vector of the second set with the lowest number
         *  of 3D points after temporal matching.
         *  \param[in] temporal_cluster_ids_0 Cluster ids of the first set with the lowest number
         *  of ids after temporal matching.
         *  \param[in] temporal_cluster_ids_1 Cluster ids of the second set with the lowest number
         *  of ids after temporal matching.
         *  \param[out] f_kpts_0_lowest Filtered temporal_kpts_0_lowest. 
         *  \param[out] f_kpts_0_highest Filtered temporal_kpts_0_highest.
         *  \param[out] f_kpts_1_lowest Filtered temporal_kpts_1_lowest. 
         *  \param[out] f_kpts_1_highest Filtered temporal_kpts_1_highest. 
         *  \param[out] f_scene_pts_0 Filtered temporal_scene_pts_0.
         *  \param[out] f_scene_pts_1 Filtered temporal_scene_pts_1.
         *  \param[out] f_cluster_ids_0 Filtered temporal_cluster_ids_0.
         *  \param[out] f_cluster_ids_1 Filtered temporal_cluster_ids_1.
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


        // Attributes:

        double ratio_ = 0.8; //!> Lowe's ratio.
       
};


#endif // KEYPOINT_MATCHER_HPP