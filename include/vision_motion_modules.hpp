/**
 *  @file
 *  @brief The VisionMotionModules class provides all the tools needed to compute the motion between stereo images.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter-Spring 2024
 */

#ifndef VISION_MOTION_MODULES_HPP
#define VISION_MOTION_MODULES_HPP


#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class VisionMotionModules
{
    public:

        /** \brief Matching types between images.
         *  For circular methods, stereo images are required.
         */ 
        enum MatchingType
        {
            RATIO_MATCHING               = 0,
            ESSENTIAL_MATCHING           = 1,
            HOMOGRAPHY_MATCHING          = 2,
            CIRCULAR_RATIO_MATCHING      = 3,
            CIRCULAR_ESSENTIAL_MATCHING  = 4,
            CIRCULAR_HOMOGRAPHY_MATCHING = 5
        };
        

        /** \brief Class constructor. Projection matrices are not set.
         *  This constructor will be used only if is inheriting 
         *  VisionMotionModules class.
         *  \param ratio The ratio for Lowe's ratio test (input).
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name 
         *  (input).
         */
        VisionMotionModules(const double ratio,
                            std::string& matching_type,
                            std::string& motion_estimation_type);


        /** \brief Class constructor. Sets projection matrices.
         *  Assumes that left camera is the master.
         *  \param ratio The ratio for Lowe's ratio test (input).
         *  \param l_proj_mat Left camera projection matrix (input).
         *  \param r_proj_mat Right camera projection matrix (input).
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        VisionMotionModules(const double ratio,
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
         *  \param matching_type Desired matching type name (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        VisionMotionModules(const double b,
                            const double f_x,
                            const double f_y,
                            const double c_x,
                            const double c_y,
                            const double ratio,
                            std::string& matching_type,
                            std::string& motion_estimation_type);


        /** \brief Detects keypoints of an image.
         *  \param img The image (input).
         *  \param enable_enhacement If true, the image is enhanced using CLAHE (input).
         *  \param detector_name String containing the name of the desired detector for
         *  detecting image keypoints (input).
         *  \param descs The image keypoints descriptors (output).
         *  \param kpts The image keypoints (output).
         */
        void detectKeypoints(const cv::Mat& img,
                             const bool enable_enhacement,
                             const std::string& detector_name,
                             cv::Mat& descs,
                             std::vector<cv::KeyPoint>& kpts);


        /** \brief Parallelizes stereo image keypoint detection.
         *  \param l_img The left image (input).
         *  \param r_img The right image (input).
         *  \param enable_enhacement If true, the image is enhanced 
         *  using CLAHE (input).
         *  \param detector_name String containing the name of the 
         *  desired detector for detecting image keypoints (input).
         *  \param l_descs The left image keypoints descriptors (output).
         *  \param r_descs The right image keypoints descriptors (output).
         *  \param l_kpts The left image keypoints (output).
         *  \param r_kpts The right image keypoints (output).
         */
        void detectStereoKeypoints(const cv::Mat& l_img,
                                   const cv::Mat& r_img,
                                   const bool enable_enhacement,
                                   const std::string& detector_name,
                                   cv::Mat& l_descs,
                                   cv::Mat& r_descs,
                                   std::vector<cv::KeyPoint>& l_kpts,
                                   std::vector<cv::KeyPoint>& r_kpts); 


        /** \brief Matches keypoints of the same stereo pair and removes outliers. To
         *  eliminate outliers, the Lowe's ratio is applied first, followed by an
         *  epipolar constraint with a maximum error of 1 pixel.
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kpts Left image keypoints (input).
         *  \param r_kpts Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kpts Filtered left image keypoints (output).
         *  \param f_r_kpts Filtered right image keypoints (output).
         */
        void matchStereoKeypoints(const cv::Mat& l_descs,
                                  const cv::Mat& r_descs,
                                  const std::vector<cv::KeyPoint>& l_kpts,
                                  const std::vector<cv::KeyPoint>& r_kpts,
                                  cv::Mat& f_l_descs,
                                  cv::Mat& f_r_descs,
                                  std::vector<cv::KeyPoint>& f_l_kpts,
                                  std::vector<cv::KeyPoint>& f_r_kpts);


        /** \brief Computes 3D points using OpenCV functions. It then removes points
         *  categorised as "far". A 3D point is considered "far away" when its z is
         *  more than 40 times the stereo baseline. "Far" 3D points are removed because
         *  they do not provide accurate scale and translation information (Paz et al.
         *  2008 and Mur-Artal et al. 2017).
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kpts Left image keypoints (input).
         *  \param r_kpts Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kpts Filtered left image keypoints (output).
         *  \param f_r_kpts Filtered right image keypoints (output).
         *  \param f_cam_pts Filtered 3D points of the scene obtained from
         *  the stereo data and relative to camera (output).
         */
        void compute3DPoints(const cv::Mat& l_descs,
                             const cv::Mat& r_descs,
                             const std::vector<cv::KeyPoint>& l_kpts,
                             const std::vector<cv::KeyPoint>& r_kpts,
                             cv::Mat& f_l_descs,
                             cv::Mat& f_r_descs,
                             std::vector<cv::KeyPoint>& f_l_kpts,
                             std::vector<cv::KeyPoint>& f_r_kpts,
                             std::vector<cv::Point3f>& f_cam_pts);


        /** \brief Matches visual keypoints of two different stereo images. To remove
         *  outliers, after matching, the user can choose between these different
         *  techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Plain essential matrix + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *      - Circular essential matrix + RANSAC.
         *  "Plain" involves only left keypoints, "Circular" left and right keypoints.
         *  To select the desired outlier remover, the appropriate matcher (enum type:
         *  MatchingType) must be chosen.
         *  \param l_descs_0 Left descriptors of the first stereo image (input). In 
         *  consecutive images: current left descriptors. In loop closing images: query
         *  left descriptors.
         *  \param r_descs_0 Right descriptors of the first stereo image (input). In
         *  consecutive images: current right descriptors. In loop closing images: query
         *  right descriptors.
         *  \param l_descs_1 Left descriptors of the second stereo image (input). In
         *  consecutive images: previous left descriptors. In loop closing images: 
         *  candidate left descriptors.
         *  \param r_descs_1 Right descriptors of the second stereo image (input). In
         *  consecutive images: previous left descriptors. In loop closing images:
         *  candidate left descriptors.
         *  \param l_kpts_0 Left keypoints of the first stereo image (input). In
         *  consecutive images: current left keypoints. In loop closing images: 
         *  query left keypoints.
         *  \param r_kpts_0 Right keypoints of the first stereo image (input). In
         *  consecutive images: current right keypoints. In loop closing images: 
         *  query right keypoints.
         *  \param l_kpts_1 Left keypoints of the second stereo image (input). In
         *  consecutive images: previous left keypoints. In loop closing images:
         *  candidate left keypoints.
         *  \param r_kpts_1 Right keypoints of the second stereo image (input). In
         *  consecutive images: previous right keypoints. In loop closing images:
         *  candidate right keypoints.
         *  \param cam_pts_0 3D points relative to camera of the first stereo image (input).
         *  In consecutive images: current 3D points. In loop closing images: query 3D points.
         *  \param cam_pts_1 3D points relative to camera of the second stereo image (input).
         *  In consecutive images: previous 3D points. In loop closing images: candidate 3D points.
         *  \param f_l_kpts_0 Filtered left keypoints of the first stereo image (output).
         *  In consecutive images: current left keypoints filtered out. In loop closing images:
         *  query left keypoints filtered out.
         *  \param f_r_kpts_0 Filtered right keypoints of the first stereo image (output).
         *  In consecutive images: current right keypoints filtered out. In loop closing images:
         *  query right keypoints filtered out.
         *  \param f_l_kpts_1 Filtered left keypoints of the second stereo image (output). 
         *  In consecutive images: previous left keypoints filtered out. In loop closing images:
         *  candidate left keypoints filtered out.
         *  \param f_r_kpts_1 Filtered right keypoints of the second stereo image (output).
         *  In consecutive images: previous right keypoints filtered out. In loop closing images:
         *  candidate right keypoints filtered out.
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first stereo image (output).
         *  In consecutive images: current 3D points filtered out. In loop closing images: query
         *  3D points filtered out.
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second stereo image (output).
         *  In consecutive images: previous 3D points filtered out. In loop closing images: 
         *  candidate 3D points filtered out.
         */
        void matchNonStereoKeypoints(const cv::Mat& l_descs_0,
                                     const cv::Mat& r_descs_0,
                                     const cv::Mat& l_descs_1,
                                     const cv::Mat& r_descs_1,
                                     const std::vector<cv::KeyPoint>& l_kpts_0,
                                     const std::vector<cv::KeyPoint>& r_kpts_0,
                                     const std::vector<cv::KeyPoint>& l_kpts_1,
                                     const std::vector<cv::KeyPoint>& r_kpts_1,
                                     const std::vector<cv::Point3f>& cam_pts_0,
                                     const std::vector<cv::Point3f>& cam_pts_1,
                                     std::vector<cv::KeyPoint>& f_l_kpts_0,
                                     std::vector<cv::KeyPoint>& f_r_kpts_0,
                                     std::vector<cv::KeyPoint>& f_l_kpts_1,
                                     std::vector<cv::KeyPoint>& f_r_kpts_1,
                                     std::vector<cv::Point3f>& f_cam_pts_0,
                                     std::vector<cv::Point3f>& f_cam_pts_1);


        /** \brief Matches the visual keypoints of two different sets. These sets must come
         *  from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the
         *  sets are divided into clusters, the cluster_ids_0 and cluster_ids_1 variables
         *  must be filled appropriately. If the sets are not divided into clusters, the
         *  cluster_ids_0 and cluster_ids_1 variables may be empty. To remove outliers, after
         *  matching, the user can choose between these different techniques:
         *      - Plain Lowe's ratio.
         *      - Plain homography + RANSAC.
         *      - Plain essential matrix + RANSAC.
         *      - Circular Lowe's ratio.
         *      - Circular homography + RANSAC.
         *      - Circular essential matrix + RANSAC.
         *  "Plain" involves only left keypoints, "Circular" left and right keypoints. To 
         *  select the desired outlier remover, the appropriate matcher (enum type: MatchingType)
         *  must be chosen.
         *  \param l_descs_0 Left descriptors of the first set (input). In consecutive images:
         *  current left descriptors. In loop closing images: query left descriptors.
         *  \param r_descs_0 Right descriptors of the first set (input). In consecutive images:
         *  current right descriptors. In loop closing images: query right descriptors.
         *  \param l_descs_1 Left descriptors of the second set (input). In consecutive images:
         *  previous left descriptors. In loop closing images: candidate left descriptors.
         *  \param r_descs_1 Right descriptors of the second set (input). In consecutive images:
         *  previous left descriptors. In loop closing images: candidate left descriptors.
         *  \param cluster_ids_0 Subset identifiers of the first set (input). In consecutive
         *  images: current identifiers. In loop closing images: query identifiers.
         *  \param cluster_ids_1 Subset identifiers of the second set (input). In consecutive
         *  images: previous identifiers. In loop closing images: candidate identifiers.
         *  \param l_kpts_0 Left keypoints of the first set (input). In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param r_kpts_0 Right keypoints of the first set (input). In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints.
         *  \param l_kpts_1 Left keypoints of the second set (input). In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param r_kpts_1 Right keypoints of the second set (input). In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param cam_pts_0 3D points relative to camera of the first set (input). In 
         *  consecutive images: current 3D points. In loop closing images: query 3D points.
         *  \param cam_pts_1 3D points relative to camera of the second set (input). In
         *  consecutive images: previous 3D points. In loop closing images: candidate 3D points.
         *  \param f_cluster_ids_0 Filtered subset identifiers of the first set (output). In
         *  consecutive images: current identifiers filtered out. In loop closing images: 
         *  query identifiers filtered out.
         *  \param f_cluster_ids_1 Filtered subset identifiers of the second set (output). In
         *  consecutive images: previous identifiers filtered out. In loop closing images:
         *  candidate identifiers filtered out.
         *  \param f_l_kpts_0 Filtered left keypoints of the first set (output). In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param f_r_kpts_0 Filtered right keypoints of the first set (output). In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param f_l_kpts_1 Filtered left keypoints of the second set (output). In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param f_r_kpts_1 Filtered right keypoints of the second set (output). In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first set (output).
         *  In consecutive images: current 3D points filtered out. In loop closing images: query
         *  3D points filtered out.
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second set (output).
         *  In consecutive images: previous 3D points filtered out. In loop closing images: 
         *  candidate 3D points filtered out.
         */
        void matchKeypointClusters(const cv::Mat& l_descs_0,
                                   const cv::Mat& r_descs_0,
                                   const cv::Mat& l_descs_1,
                                   const cv::Mat& r_descs_1,
                                   const std::vector<int>& cluster_ids_0,
                                   const std::vector<int>& cluster_ids_1,
                                   const std::vector<cv::KeyPoint>& l_kpts_0,
                                   const std::vector<cv::KeyPoint>& r_kpts_0,
                                   const std::vector<cv::KeyPoint>& l_kpts_1,
                                   const std::vector<cv::KeyPoint>& r_kpts_1,
                                   const std::vector<cv::Point3f>& cam_pts_0,
                                   const std::vector<cv::Point3f>& cam_pts_1,
                                   std::vector<int>& f_cluster_ids_0,
                                   std::vector<int>& f_cluster_ids_1,
                                   std::vector<cv::KeyPoint>& f_l_kpts_0,
                                   std::vector<cv::KeyPoint>& f_r_kpts_0,
                                   std::vector<cv::KeyPoint>& f_l_kpts_1,
                                   std::vector<cv::KeyPoint>& f_r_kpts_1,
                                   std::vector<cv::Point3f>& f_cam_pts_0,
                                   std::vector<cv::Point3f>& f_cam_pts_1);


        /** \brief Estimates motion between two stereo images using keypoints and
         *  3D-to-2D method. The user can choose between the following methods to 
         *  estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *      - iterative.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param l_kpts_0 Left keypoints of the first stereo image (input). In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param r_kpts_0 Right keypoints of the first stereo image (input). In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints.
         *  \param l_kpts_1 Left keypoints of the second stereo image (input). In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param r_kpts_1 Right keypoints of the second stereo image (input). In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param cam_pts_0 3D points relative to camera of the first stereo image (input). In
         *  consecutive images: current 3D points. In loop closing images: query 3D points.
         *  \param cam_pts_1 3D points relative to camera of the second stereo image (input). In
         *  consecutive images: previous 3D points. In loop closing images: candidate 3D points.
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_l_kpts_0 Filtered left keypoints of the first stereo image (output). In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param f_r_kpts_0 Filtered right keypoints of the first stereo image (output). In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param f_l_kpts_1 Filtered left keypoints of the second stereo image (output). In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param f_r_kpts_1 Filtered right keypoints of the second stereo image (output). In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first stereo image (output).
         *  In consecutive images: current 3D points filtered out. In loop closing images: query
         *  3D points filtered out.
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second stereo image (output).
         *  In consecutive images: previous 3D points filtered out. In loop closing images: 
         *  candidate 3D points filtered out.
         */
        void estimateMotion(const std::vector<cv::KeyPoint>& l_kpts_0,
                            const std::vector<cv::KeyPoint>& r_kpts_0,
                            const std::vector<cv::KeyPoint>& l_kpts_1,
                            const std::vector<cv::KeyPoint>& r_kpts_1,
                            const std::vector<cv::Point3f>& cam_pts_0,
                            const std::vector<cv::Point3f>& cam_pts_1,
                            cv::Mat& r_vec,
                            cv::Mat& t_vec,
                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                            std::vector<cv::Point3f>& f_cam_pts_0,
                            std::vector<cv::Point3f>& f_cam_pts_1);


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
         *      - iterative.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param cluster_ids_0 Subset identifiers of the first set (input). In consecutive
         *  images: current identifiers. In loop closing images: query identifiers.
         *  \param cluster_ids_1 Subset identifiers of the second set (input). In consecutive
         *  images: previous identifiers. In loop closing images: candidate identifiers.
         *  \param l_kpts_0 Left keypoints of the first set (input). In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param r_kpts_0 Right keypoints of the first set (input). In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints.
         *  \param l_kpts_1 Left keypoints of the second set (input). In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param r_kpts_1 Right keypoints of the second set (input). In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param cam_pts_0 3D points relative to camera of the first set (input). In
         *  consecutive images: current 3D points. In loop closing images: query 3D points.
         *  \param cam_pts_1 3D points relative to camera of the second set (input). In
         *  consecutive images: previous 3D points. In loop closing images: candidate 3D points.
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_cluster_ids_0 Filtered subset identifiers of the first set (output). In
         *  consecutive images: current identifiers filtered out. In loop closing images: 
         *  query identifiers filtered out.
         *  \param f_cluster_ids_1 Filtered subset identifiers of the second set (output). In
         *  consecutive images: previous identifiers filtered out. In loop closing images:
         *  candidate identifiers filtered out.
         *  \param f_l_kpts_0 Filtered left keypoints of the first set (output). In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out.
         *  \param f_r_kpts_0 Filtered right keypoints of the first set (output). In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out.
         *  \param f_l_kpts_1 Filtered left keypoints of the second set (output). In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out.
         *  \param f_r_kpts_1 Filtered right keypoints of the second set (output). In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out.
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first set (output).
         *  In consecutive images: current 3D points filtered out. In loop closing images: query
         *  3D points filtered out.
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second set (output).
         *  In consecutive images: previous 3D points filtered out. In loop closing images: 
         *  candidate 3D points filtered out.
         */
        void estimateMotion(const std::vector<int>& cluster_ids_0,
                            const std::vector<int>& cluster_ids_1,
                            const std::vector<cv::KeyPoint>& l_kpts_0,
                            const std::vector<cv::KeyPoint>& r_kpts_0,
                            const std::vector<cv::KeyPoint>& l_kpts_1,
                            const std::vector<cv::KeyPoint>& r_kpts_1,
                            const std::vector<cv::Point3f>& cam_pts_0,
                            const std::vector<cv::Point3f>& cam_pts_1,
                            cv::Mat& r_vec,
                            cv::Mat& t_vec,
                            std::vector<int>& f_cluster_ids_0,
                            std::vector<int>& f_cluster_ids_1,
                            std::vector<cv::KeyPoint>& f_l_kpts_0,
                            std::vector<cv::KeyPoint>& f_r_kpts_0,
                            std::vector<cv::KeyPoint>& f_l_kpts_1,
                            std::vector<cv::KeyPoint>& f_r_kpts_1,
                            std::vector<cv::Point3f>& f_cam_pts_0,
                            std::vector<cv::Point3f>& f_cam_pts_1);


        /** \brief Enhances image using Contrast-Limited Adaptive Histogram Equalization (CLAHE),
         *  with adaptive thresholding to dynamically overcome variable lighting conditions (Sabry
         *  et al. 2022).
         *  \param img The image (input).
         *  @return The enhanced image. 
         */
        cv::Mat enhanceImage(const cv::Mat& img);


        /** \brief Homogenises the distribution of keypoints over the spatial domain of the image.
         *  \param n_rows_img Number of rows in the image (input).
         *  \param n_cols_img Number of columns in the image (input).
         *  \param n_kpts_bucket Number of keypoints per bucket (input).
         *  \param l_kpts_0 Left keypoints of the first set (input). In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints.
         *  \param r_kpts_0 Right keypoints of the first set (input). In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints.
         *  \param l_kpts_1 Left keypoints of the second set (input). In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints.
         *  \param r_kpts_1 Right keypoints of the second set (input). In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints.
         *  \param cam_pts_0 3D points relative to camera of the first set (input). In 
         *  consecutive images: current 3D points. In loop closing images: query 3D points.
         *  \param cam_pts_1 3D points relative to camera of the second set (input). In
         *  consecutive images: previous 3D points. In loop closing images: candidate 3D points.
         *  \param f_l_kpts_0 Bucketed left keypoints of the first set (output). In consecutive
         *  images: bucketed current left keypoints. In loop closing images: bucketed query left 
         *  keypoints.
         *  \param f_r_kpts_0 Bucketed right keypoints of the first set (output). In consecutive
         *  images: bucketed current right keypoints. In loop closing images: bucketed query right 
         *  keypoints.
         *  \param f_l_kpts_1 Bucketed left keypoints of the second set (output). In consecutive
         *  images: bucketed previous left keypoints. In loop closing images: bucketed candidate left 
         *  keypoints.
         *  \param f_r_kpts_1 Bucketed right keypoints of the second set (output). In consecutive
         *  images: bucketed previous right keypoints. In loop closing images: bucketed candidate right 
         *  keypoints.
         *  \param f_cam_pts_0 Bucketed 3D points relative to camera of the first set (output).
         *  In consecutive images: bucketed current 3D points. In loop closing images: bucketed query
         *  3D points filtered out.
         *  \param f_cam_pts_1 Bucketed 3D points relative to camera of the second set (output).
         *  In consecutive images: bucketed previous 3D points. In loop closing images: bucketed
         *  candidate 3D points filtered out.
         */
        std::vector<std::vector<std::pair<int, cv::KeyPoint>>> bucketKeypoints(const int n_rows_img,
                                                                               const int n_cols_img,
                                                                               const uint n_kpts_bucket,
                                                                               const std::vector<cv::KeyPoint>& l_kpts_0,
                                                                               const std::vector<cv::KeyPoint>& r_kpts_0,
                                                                               const std::vector<cv::KeyPoint>& l_kpts_1,
                                                                               const std::vector<cv::KeyPoint>& r_kpts_1,
                                                                               const std::vector<cv::Point3f>& cam_pts_0,
                                                                               const std::vector<cv::Point3f>& cam_pts_1,
                                                                               std::vector<cv::KeyPoint>& f_l_kpts_0,
                                                                               std::vector<cv::KeyPoint>& f_r_kpts_0,
                                                                               std::vector<cv::KeyPoint>& f_l_kpts_1,
                                                                               std::vector<cv::KeyPoint>& f_r_kpts_1,
                                                                               std::vector<cv::Point3f>& f_cam_pts_0,
                                                                               std::vector<cv::Point3f>& f_cam_pts_1);


    protected:

        
        struct KeypointPairResponseGreaterThanThreshold
        {
            KeypointPairResponseGreaterThanThreshold(float value): value_(value) {}  

            inline bool operator() (const std::pair<int, cv::KeyPoint>& pair) const
            {
                return pair.second.response > value_;
            }

            float value_;
        };


        struct KeypointPairResponseGreater
        {
            inline bool operator() (const std::pair<int, cv::KeyPoint>& pair_0, const std::pair<int, cv::KeyPoint>& pair_1) const
            {
                return pair_0.second.response > pair_1.second.response;
            }
        };



        /** \brief Transforms a string into the correct MatchingType value.
         *  \param matching_type Desired matching type name (input).
         *  @return a MatchingType value.
         */
        VisionMotionModules::MatchingType selectMatchingType(std::string& matching_type);


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


        /** \brief Matches visual keypoints.
         *  \param descs_0 Descriptors of the first set (input).
         *  \param descs_1 Descriptors of the second set (input).
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
         *  \param descs_0 Descriptors of the first set (input).
         *  \param descs_1 Descriptors of the second set (input).
         *  \param cluster_ids_0 Subset identifier of the first set (input).
         *  \param cluster_ids_1 Subset identifier of the second set (input).
         *  \param kpts_0 Keypoints of the first set (input).
         *  \param kpts_1 Keypoints of the second set (input).
         *  \param cam_pts_0 3D points relative to camera of the first set (input).
         *  \param cam_pts_1 3D points relative to camera of the second set (input).
         *  \param f_descs_0 Filtered descriptors of the first set (output).
         *  \param f_descs_1 Filtered descriptors of the second set (output).
         *  \param f_cluster_ids_0 Filtered subset identifier of the first set (output).
         *  \param f_cluster_ids_1 Filtered subset identifier of the second set (output).
         *  \param f_kpts_0 Filtered keypoints of the first set (output).
         *  \param f_kpts_1 Filtered keypoints of the second set (output).
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first set (output).
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second set (output).
         */
        void ratioMatching(const cv::Mat& descs_0, 
                           const cv::Mat& descs_1,
                           const std::vector<int>& cluster_ids_0,
                           const std::vector<int>& cluster_ids_1,
                           const std::vector<cv::KeyPoint>& kpts_0,
                           const std::vector<cv::KeyPoint>& kpts_1,
                           const std::vector<cv::Point3f>& cam_pts_0,
                           const std::vector<cv::Point3f>& cam_pts_1,
                           cv::Mat& f_descs_0,
                           cv::Mat& f_descs_1,
                           std::vector<int>& f_cluster_ids_0,
                           std::vector<int>& f_cluster_ids_1,
                           std::vector<cv::KeyPoint>& f_kpts_0,
                           std::vector<cv::KeyPoint>& f_kpts_1,
                           std::vector<cv::Point3f>& f_cam_pts_0,
                           std::vector<cv::Point3f>& f_cam_pts_1);


        /** \brief Matches the visual keypoints of two different sets. These sets must come
         *  from at least two different stereo pairs. These sets may consist of different
         *  keypoint clusters which in turn may belong to different stereo images. If the
         *  sets are divided into clusters, the cluster_ids_0 and cluster_ids_1 variables
         *  must be filled appropriately. If the sets are not divided into clusters, the
         *  cluster_ids_0 and cluster_ids_1 variables may be empty. After matching, a 
         *  geometrical restriction (essential matrix or homography) wrapped in a RANSAC 
         *  scheme is applied to remove outliers.
         *  \param coplanar_pts If true, the points are assumed to be coplanar
         *  (or can be approximated to be in the same plane) and a homography is
         *  calculated to eliminate outliers. Otherwise, an essential matrix is 
         *  computed to remove outliers.
         *  \param descs_0 Descriptors of the first set (input).
         *  \param descs_1 Descriptors of the second set (input).
         *  \param cluster_ids_0 Subset identifier of the first set (input).
         *  \param cluster_ids_1 Subset identifier of the second set (input).
         *  \param kpts_0 Keypoints of the first set (input).
         *  \param kpts_1 Keypoints of the second set (input).
         *  \param cam_pts_0 3D points relative to camera of the first set (input).
         *  \param cam_pts_1 3D points relative to camera of the second set (input).
         *  \param f_descs_0 Filtered descriptors of the first set (output).
         *  \param f_descs_1 Filtered descriptors of the second set (output).
         *  \param f_cluster_ids_0 Filtered subset identifier of the first set (output).
         *  \param f_cluster_ids_1 Filtered subset identifier of the second set (output).
         *  \param f_kpts_0 Filtered keypoints of the first set (output).
         *  \param f_kpts_1 Filtered keypoints of the second set (output).
         *  \param f_cam_pts_0 Filtered 3D points relative to camera of the first set (output).
         *  \param f_cam_pts_1 Filtered 3D points relative to camera of the second set (output).
         */
        void geometricalMatching(bool coplanar_pts,
                                 const cv::Mat& descs_0, 
                                 const cv::Mat& descs_1,
                                 const std::vector<int>& cluster_ids_0,
                                 const std::vector<int>& cluster_ids_1,
                                 const std::vector<cv::KeyPoint>& kpts_0,
                                 const std::vector<cv::KeyPoint>& kpts_1,
                                 const std::vector<cv::Point3f>& cam_pts_0,
                                 const std::vector<cv::Point3f>& cam_pts_1,
                                 cv::Mat& f_descs_0,
                                 cv::Mat& f_descs_1,
                                 std::vector<int>& f_cluster_ids_0,
                                 std::vector<int>& f_cluster_ids_1,
                                 std::vector<cv::KeyPoint>& f_kpts_0,
                                 std::vector<cv::KeyPoint>& f_kpts_1,
                                 std::vector<cv::Point3f>& f_cam_pts_0,
                                 std::vector<cv::Point3f>& f_cam_pts_1);


        /** \brief Assemble four matchings to build circular matches.
         *  \param stereo_kpts_0_lowest Keypoints of the first set matched in stereo matching.
         *  Must belong to the side (left/right) with the lowest number of keypoints after the
         *  successive matching (input).
         *  \param stereo_kpts_0_highest Keypoints of the first set matched in stereo matching.
         *  Must belong to the side (left/right) with the highest number of keypoints after the
         *  successive matching (input).
         *  \param stereo_kpts_1_lowest Keypoints of the second set matched in stereo matching.
         *  Must belong to the side (left/right) with the lowest number of keypoints after the
         *  successive matching (input).
         *  \param stereo_kpts_1_highest Keypoints of the second set matched in stereo matching.
         *  Must belong to the side (left/right) with the highest number of keypoints after the
         *  successive matching (input).
         *  \param successive_kpts_0_lowest Keypoint vector of the first set with the lowest 
         *  number of keypoints after successive matching (input).
         *  \param successive_kpts_0_highest Keypoint vector of the first set with the highest
         *  number of keypoints after successive matching (input).
         *  \param successive_kpts_1_lowest Keypoint vector of the second set with the lowest 
         *  number of keypoints after successive matching (input).
         *  \param successive_kpts_1_highest Keypoint vector of the second set with the highest
         *  number of keypoints after successive matching (input).
         *  \param successive_cam_pts_0 3D points vector relative to camera of the first set with
         *  the lowest number of 3D points after successive matching (input).
         *  \param successive_cam_pts_1 3D points vector relative to camera of the second set with
         *  the lowest number of 3D points after successive matching (input).
         *  \param successive_cluster_ids_0 Cluster ids of the first set with the lowest number
         *  of ids after successive matching (input).
         *  \param successive_cluster_ids_1 Cluster ids of the second set with the lowest number
         *  of ids after successive matching (input).
         *  \param f_kpts_0_lowest Filtered successive_kpts_0_lowest (output). 
         *  \param f_kpts_0_highest Filtered successive_kpts_0_highest (output).
         *  \param f_kpts_1_lowest Filtered successive_kpts_1_lowest (output). 
         *  \param f_kpts_1_highest Filtered successive_kpts_1_highest (output). 
         *  \param f_cam_pts_0 Filtered successive_cam_pts_0 (output).
         *  \param f_cam_pts_1 Filtered successive_cam_pts_1 (output).
         *  \param f_cluster_ids_0 Filtered successive_cluster_ids_0 (output).
         *  \param f_cluster_ids_1 Filtered successive_cluster_ids_1 (output).
         */
        void assembleCircularMatchings(const std::vector<cv::KeyPoint>& stereo_kpts_0_lowest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_0_highest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_1_lowest,
                                       const std::vector<cv::KeyPoint>& stereo_kpts_1_highest,
                                       const std::vector<cv::KeyPoint>& successive_kpts_0_lowest,
                                       const std::vector<cv::KeyPoint>& successive_kpts_0_highest,
                                       const std::vector<cv::KeyPoint>& successive_kpts_1_lowest,
                                       const std::vector<cv::KeyPoint>& successive_kpts_1_highest,
                                       const std::vector<cv::Point3f>& successive_cam_pts_0,
                                       const std::vector<cv::Point3f>& successive_cam_pts_1,
                                       const std::vector<int>& successive_cluster_ids_0,
                                       const std::vector<int>& successive_cluster_ids_1,
                                       std::vector<cv::KeyPoint>& f_kpts_0_lowest,
                                       std::vector<cv::KeyPoint>& f_kpts_0_highest,
                                       std::vector<cv::KeyPoint>& f_kpts_1_lowest,
                                       std::vector<cv::KeyPoint>& f_kpts_1_highest,
                                       std::vector<cv::Point3f>& f_cam_pts_0,
                                       std::vector<cv::Point3f>& f_cam_pts_1,
                                       std::vector<int>& f_cluster_ids_0,
                                       std::vector<int>& f_cluster_ids_1);


        /** \brief Gets the minimum, maximum and median value of a greyscale image.
         *  \param img The image (input).
         *  \param min The minimum intensity value (output).
         *  \param max The maximum intensity value (output).
         *  \param median The median intensity value (output).
         *  @return true if the image is in greyscale and therefore the desired 
         *  values could be obtained.
         */
        bool getImageMainStatisticalValues(const cv::Mat& img,
                                           int& min,
                                           int& max,
                                           int& median);


        /** \brief Retain the best keypoints. Modification of cv::KeyPointsFilter::retainBest.
         *  \param n_kpts Keypoints to be retained.
         *  \param kpts The keypoints.
         */
        void retainBest(int n_kpts, std::vector<std::pair<int, cv::KeyPoint>>& kpts);


        double ratio_; //!> Lowe's ratio.
        
        cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Left camera projection matrix.
        
        cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Right camera projection matrix.

        MatchingType matching_type_; //!> Matching type.

        cv::SolvePnPMethod motion_estimation_type_; //!> Motion estimation type.
};


#endif // VISION_MOTION_MODULES_HPP