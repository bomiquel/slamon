/**
 *  @file
 *  @brief The MotionEstimtor class provides tools to compute the motion between stereo images.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date 2024
 */

#ifndef MOTION_ESTIMATOR_HPP
#define MOTION_ESTIMATOR_HPP


#include <vector>
#include <string>
#include <iostream>

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
            IPPE = cv::SOLVEPNP_IPPE,
            DUAL = 10
        };


        /** \brief Empty class constructor.
         */
        MotionEstimator() {};


        /** \brief Class constructor. Projection matrices are not set. This
         *  constructor will be used only if is inheriting MotionEstimator class.
         *  \param motion_estimation_type Desired motion type name (input).
         */
        MotionEstimator(std::string& motion_estimation_type);


        /** \brief Class constructor. Sets projection matrices.
         *  Assumes that left camera is the master.
         *  \param l_proj_mat Left camera projection matrix (input).
         *  \param r_proj_mat Right camera projection matrix (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        MotionEstimator(const cv::Mat& l_proj_mat,
                        const cv::Mat& r_proj_mat,
                        std::string& motion_estimation_type);

        
        /** \brief Class constructor. Builds projection matrices.
         *  Assumes that left camera is the master.
         *  \param b Stereo rig baseline in meters (input).
         *  \param f_x Focal length x (input).
         *  \param f_y Focal length y (input).
         *  \param c_x Principal point x (input).
         *  \param c_y Principal point y (input).
         *  \param motion_estimation_type Desired motion type name (input).
         */
        MotionEstimator(const double b,
                        const double f_x,
                        const double f_y,
                        const double c_x,
                        const double c_y,
                        std::string& motion_estimation_type);


        /** \brief Get left projection matrix.
         *  @return cv::Mat with left projection matrix.
         */
        inline cv::Mat getLeftProjectionMatrix() const {return l_proj_mat_;}


        /** \brief Get right projection matrix.
         *  @return cv::Mat with right projection matrix.
         */
        inline cv::Mat getRightProjectionMatrix() const {return r_proj_mat_;}

        
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
         *  \param f_l_cam_pts Filtered 3D points of the scene obtained from
         *  the stereo data and relative to left camera (output).
         */
        void compute3DPoints(const cv::Mat& l_descs,
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
         *  \param max_kpts Maximum number of keypoints per bucket (input).
         *  \param img_width Number of columns in the image (input).
         *  \param img_height Number of rows in the image (input).
         *  \param c_l_kpts Current left keypoints (input).
         *  \param c_r_kpts Current right keypoints (input).
         *  \param p_l_kpts Previous left keypoints (input).
         *  \param p_r_kpts Previous right keypoints (input).
         *  \param c_l_cam_pts Current 3D points relative to left camera (input).
         *  \param p_l_cam_pts Previous 3D points relative to left camera (input).
         *  \param f_c_l_kpts Bucketed current left keypoints (output).
         *  \param f_c_r_kpts Bucketed current right keypoints (output). 
         *  \param f_p_l_kpts Bucketed previous left keypoints (output). 
         *  \param f_p_r_kpts Bucketed previous right keypoints (output). 
         *  \param f_c_l_cam_pts Bucketed current 3D points relative to left camera (output).
         *  \param f_p_l_cam_pts Bucketed previous 3D points relative to left camera (output).
         */
        std::vector<std::vector<std::pair<int, cv::KeyPoint>>> bucketKeypoints(const uint max_kpts,
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


        /** \brief Estimates motion between two stereo images using keypoints and
         *  3D-to-2D method. The user can choose between the following methods to 
         *  estimate the movement:
         *      - p3p.
         *      - ap3p.
         *      - epnp.
         *      - dual.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param l_kpts_0 Left keypoints of the first stereo image. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_kpts_0 Right keypoints of the first stereo image. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints (input). 
         *  \param l_kpts_1 Left keypoints of the second stereo image. In consecutive images:
         *  previous left keypoints. In loop closing images: candidate left keypoints (input).
         *  \param r_kpts_1 Right keypoints of the second stereo image. In consecutive images: 
         *  previous right keypoints. In loop closing images: candidate right keypoints (input).
         *  \param scene_pts_0 3D points of the first stereo image. In consecutive images: 
         *  current 3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera (input).
         *  \param scene_pts_1 3D points of the second stereo image. In consecutive images: 
         *  previous 3D points relative to left camera. In loop closing images: candidate 3D
         *  points relative to world (input).
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_l_kpts_0 Filtered left keypoints of the first stereo image. In consecutive
         *  images: current left keypoints filtered out. In loop closing images: query left
         *  keypoints filtered out (output).
         *  \param f_r_kpts_0 Filtered right keypoints of the first stereo image. In consecutive
         *  images: current right keypoints filtered out. In loop closing images: query right
         *  keypoints filtered out (output).
         *  \param f_l_kpts_1 Filtered left keypoints of the second stereo image. In consecutive
         *  images: previous left keypoints filtered out. In loop closing images: candidate left
         *  keypoints filtered out (output).
         *  \param f_r_kpts_1 Filtered right keypoints of the second stereo image. In consecutive
         *  images: previous right keypoints filtered out. In loop closing images: candidate right
         *  keypoints filtered out (output).
         *  \param f_scene_pts_0 Filtered 3D points of the first stereo image. In consecutive images:
         *  current 3D points relative to left camera filtered out. In loop closing images: query
         *  3D points relative to left camera filtered out (output).
         *  \param f_scene_pts_1 Filtered 3D points of the second stereo image. In consecutive images:
         *  previous 3D points relative to left camera filtered out. In loop closing images: candidate
         *  3D points relative to world filtered out (output).
         */
        void estimateMotion(const std::vector<cv::KeyPoint>& l_kpts_0,
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
         *      - dual.
         *  To be robust against outliers, the motion estimation is wrapped in a RANSAC scheme.
         *  \param cluster_ids_0 Subset identifiers of the first set. In consecutive images:
         *   current identifiers. In loop closing images: query identifiers (input).
         *  \param cluster_ids_1 Subset identifiers of the second set. In consecutive images: 
         *  previous identifiers. In loop closing images: candidate identifiers (input).
         *  \param l_kpts_0 Left keypoints of the first set In consecutive images: current
         *  left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_kpts_0 Right keypoints of the first set. In consecutive images: current 
         *  right keypoints. In loop closing images: query right keypoints (input).
         *  \param l_kpts_1 Left keypoints of the second set. In consecutive images: previous
         *  left keypoints. In loop closing images: candidate left keypoints (input).
         *  \param r_kpts_1 Right keypoints of the second set. In consecutive images: previous
         *  right keypoints. In loop closing images: candidate right keypoints (input).
         *  \param scene_pts_0 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera. In loop closing images: query 3D points
         *  relative to left camera (input).
         *  \param scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world (input).
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param f_cluster_ids_0 Filtered subset identifiers of the first set. In consecutive
         *  images: current identifiers filtered out. In loop closing images: query identifiers
         *  filtered out (output).
         *  \param f_cluster_ids_1 Filtered subset identifiers of the second set In consecutive
         *  images: previous identifiers filtered out. In loop closing images: candidate identifiers 
         *  filtered out (output).
         *  \param f_l_kpts_0 Filtered left keypoints of the first set. In consecutive images: 
         *  current left keypoints filtered out. In loop closing images: query left keypoints
         *  filtered out (output).
         *  \param f_r_kpts_0 Filtered right keypoints of the first set. In consecutive images: 
         *  current right keypoints filtered out. In loop closing images: query right keypoints
         *  filtered out (output).
         *  \param f_l_kpts_1 Filtered left keypoints of the second set. In consecutive images:
         *  previous left keypoints filtered out. In loop closing images: candidate left keypoints
         *  filtered out (output).
         *  \param f_r_kpts_1 Filtered right keypoints of the second set. In consecutive images:
         *  previous right keypoints filtered out. In loop closing images: candidate right keypoints
         *  filtered out (output).
         *  \param f_scene_pts_0 Filtered 3D points of the first set. In consecutive images: current
         *  3D points relative to left camera filtered out. In loop closing images: query 3D points 
         *  relative to left camera filtered out (output).
         *  \param f_scene_pts_1 Filtered 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera filtered out. In loop closing images: candidate 3D points 
         *  relative to world filtered out (output).
         */
        void estimateMotion(const std::vector<int>& cluster_ids_0,
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


    protected:

        cv::Mat l_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Left camera projection matrix.
        
        cv::Mat r_proj_mat_ = cv::Mat::eye(3, 4, CV_64FC1); //!> Right camera projection matrix.

    
    private:

        enum DualResult
        {
            FAILED    = 0,
            UPDATED   = 1,
            CONVERGED = 2
        };

        struct SortBucketsByMatchesSlopesAscending 
        {
            inline bool operator() (const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_0, 
                                    const std::pair<float, std::pair<int, cv::KeyPoint>>& pair_1) const
            {
                return pair_0.first < pair_1.first;
            }
        };


        /** \brief Transforms a string into the correct SolvePnPMethod value.
         *  \param motion_type Desired motion estimation type name (input).
         *  @return a MotionEstimator::Type value.
         */
        MotionEstimator::Type selectMotionEstimationType(std::string& motion_estimation_type);


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


        /** \brief By minimising the sum of the reprojection errors, obtain the transform
         *  (R | t) between the 3D points of the scene and the keypoints. 
         *  This code comes from/is a modification of LIBVISO2.
         *  \param ransac_iter RANSAC iterations (input).
         *  \param l_pts_0 Left keypoints of the first set. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_pts_0 Right keypoints of the first set. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints (input).
         *  \param scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world (input).
         *  \param r_vec Estimated rotation vector (output).
         *  \param t_vec Estimated translation matrix (output).
         *  \param inliers Vector that contains indices of inliers in l_pts_0, r_pts_0 and scene_pts_1
         *  (output).
         */
        void solveDualPnPRansac(const uint ransac_iter,
                                const std::vector<cv::Point2f>& l_pts_0,
                                const std::vector<cv::Point2f>& r_pts_0,
                                const std::vector<cv::Point3f>& scene_pts_1,
                                cv::Mat& r_vec,
                                cv::Mat& t_vec,
                                std::vector<int>& inliers);


        /** \brief Gets random and unique indices from a vector of size N.
         *  This code comes from/is a modification of LIBVISO2.
         *  \param size Number of values to choose from (N) (input).
         *  \param n_rand_idxs Number of random indices to be obtained (m) (input).
         *  @return A vector of m positions with random and uniques indices.
         */
        std::vector<int> getRandomIndices(uint size, uint n_rand_idxs);


        /** \brief Updates the transform values.
         *  This code comes from/is a modification of LIBVISO2.
         *  \param eps Value from which it is considered to be 0, to avoid accuracy
         *  errors (input).
         *  \param step_size 
         *  \param c_idxs Current indices, active indices (input).
         *  \param l_pts_0 Left keypoints of the first set. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_pts_0 Right keypoints of the first set. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints (input).
         *  \param scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world (input).
         *  \param J Jacobians of the 3D point projection on the left and right image plane.
         *  (output).
         *  \param tr Vector containing the translation and rotation (in Euler angles). The 
         *  first three values are the translation and the next three are the rotation (output).
         *  \param obs_pts Observed 2D points (output).
         *  \param pre_pts Predicted 2D points (output).
         *  \param res_pts Residuals (res_pts = obs_pts - pre_pts) (output).
         *  @return whether it has failed, been updated or converged.
         */
        MotionEstimator::DualResult updateDualParams(const double eps,
                                                     const double step_size,
                                                     const std::vector<int>& c_idxs,
                                                     const std::vector<cv::Point2f>& l_pts_0,
                                                     const std::vector<cv::Point2f>& r_pts_0,
                                                     const std::vector<cv::Point3f>& scene_pts_1,
                                                     std::vector<double>& J,
                                                     std::vector<double>& tr,
                                                     std::vector<double>& obs_pts,
                                                     std::vector<double>& pre_pts,
                                                     std::vector<double>& res_pts);


        /** \brief Computes observed, predicted and residuals points, and Jacobians.
         *  This code comes from/is a modification of LIBVISO2.
         *  \param c_idxs Current indices, active indices (input).
         *  \param l_pts_0 Left keypoints of the first set. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_pts_0 Right keypoints of the first set. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints (input).
         *  \param scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world (input).
         *  \param J Jacobians of the 3D point projection on the left and right image plane.
         *  (output).
         *  \param tr Vector containing the translation and rotation (in Euler angles). The 
         *  first three values are the translation and the next three are the rotation (output).
         *  \param obs_pts Observed 2D points (output).
         *  \param pre_pts Predicted 2D points (output).
         *  \param res_pts Residuals (res_pts = obs_pts - pre_pts) (output).
         */
        void computeVectorsAndJacobians(const std::vector<int>& c_idxs,
                                        const std::vector<cv::Point2f>& l_pts_0,
                                        const std::vector<cv::Point2f>& r_pts_0,
                                        const std::vector<cv::Point3f>& scene_pts_1,
                                        std::vector<double>& J,
                                        std::vector<double>& tr,
                                        std::vector<double>& obs_pts,
                                        std::vector<double>& pre_pts,
                                        std::vector<double>& res_pts);


        /** \brief Checks which predicted points are at a distance less than reproj_error²
         *  from the observed points.
         *  This code comes from/is a modification of LIBVISO2.
         *  \param reproj_error Inlier threshold value. The parameter value is the
         *  maximum allowed distance between the observed and predicted point 
         *  projections to consider it an inlier (input). 
         *  \param l_pts_0 Left keypoints of the first set. In consecutive images:
         *  current left keypoints. In loop closing images: query left keypoints (input).
         *  \param r_pts_0 Right keypoints of the first set. In consecutive images:
         *  current right keypoints. In loop closing images: query right keypoints (input).
         *  \param scene_pts_1 3D points of the second set. In consecutive images: previous
         *  3D points relative to left camera. In loop closing images: candidate 3D points
         *  relative to world (input).
         *  \param J Jacobians of the 3D point projection on the left and right image plane.
         *  (output).
         *  \param tr Vector containing the translation and rotation (in Euler angles). The 
         *  first three values are the translation and the next three are the rotation (output).
         *  \param obs_pts Observed 2D points (output).
         *  \param pre_pts Predicted 2D points (output).
         *  \param res_pts Residuals (res_pts = obs_pts - pre_pts) (output).
         */ 
        std::vector<int> getInliers(double reproj_error,
                                    const std::vector<cv::Point2f>& l_pts_0,
                                    const std::vector<cv::Point2f>& r_pts_0,
                                    const std::vector<cv::Point3f>& scene_pts_1,
                                    std::vector<double>& J,
                                    std::vector<double>& tr,
                                    std::vector<double>& obs_pts,
                                    std::vector<double>& pre_pts,
                                    std::vector<double>& res_pts);


        Type motion_estimation_type_ = AP3P; //!> Motion estimation type.

};


#endif // MOTION_ESTIMATOR_HPP