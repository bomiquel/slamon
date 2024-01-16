/**
 *  @file
 *  @brief The Stereo Motion Estimation Modules class provides all the modules (feature-baseds) needed to calculate the motion between two pairs of stereoscopic images.
 *  @author Bo Miquel Nordfeldt-Fiol
 *  @date Winter 2024
 */

#ifndef STEREO_MOTION_ESTIMATION_MODULES_HPP
#define STEREO_MOTION_ESTIMATION_MODULES_HPP


#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class StereoMotionEstimationModules
{
    public:

        enum MatchType
        {
            PLAIN_MATCHING      = 0,
            PLAIN_MATCHING_H    = 1,
            CIRCULAR_MATCHING   = 2,
            CIRCULAR_MATCHING_H = 3            
        };


        enum MatchResult
        {
            NOT_ENOUGH_KPS      = 0,
            NOT_ENOUGH_F_KPS    = 1,
            HOMOGRAPHY_PROBLEM  = 2,
            NOT_ENOUGH_H_F_KPS  = 3,
            SUCCESSFUL_MATCHING = 4
        };
        

        /** \brief Empty class constructor.
         */
        StereoMotionEstimationModules() {}


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
         *  To eliminate outliers, first Lowe's ratio is applied and then a
         *  Fundamental Matrix with 1 pixel of reprojection error is computed.
         *  \param ratio for eliminating outlier matches (input).
         *  \param l_descs Left image descriptors (input).
         *  \param r_descs Right image descriptors (input).
         *  \param l_kps Left image keypoints (input).
         *  \param r_kps Right image keypoints (input).
         *  \param f_l_descs Filtered left image descriptors (output).
         *  \param f_r_descs Filtered right image descriptors (output).
         *  \param f_l_kps Filtered left image keypoints (output).
         *  \param r_l_kps Filtered right image keypoints (output).
         *  @return whether the keypoint matching and the filtering could be performed.
         */
        bool matchStereoKeypoints(double ratio, 
                                  const cv::Mat& l_descs,
                                  const cv::Mat& r_descs,
                                  const std::vector<cv::KeyPoint>& l_kps,
                                  const std::vector<cv::KeyPoint>& r_kps,
                                  cv::Mat& f_l_descs,
                                  cv::Mat& f_r_descs,
                                  std::vector<cv::KeyPoint>& f_l_kps,
                                  std::vector<cv::KeyPoint>& f_r_kps);


        /** \brief Matches keypoints of differents stereo pairs and removes outliers.
         *  To eliminate outliers, Lowe's ratio is always used, but additionally an 
         *  homography can be applied. It is also possible to decide between a plain 
         *  matching (matching between left images of each stereo pair) or a circular
         *  matching (involving all 4 images).
         *  \param ratio for eliminating outlier matches (input).
         *  \param c_l_descs Current left image descriptors (input).
         *  \param c_r_descs Current right image descriptors (input).
         *  \param q_l_descs Query left image descriptors (input).
         *  \param q_r_descs Query right image descriptors (input).
         *  \param c_l_kps Current left image keypoints (input).
         *  \param c_r_kps Current right image keypoints (input).
         *  \param q_l_kps Query left image keypoints (input).
         *  \param q_r_kps Query right image keypoints (input).
         *  \param c_camera_points Current image 3D points (input).
         *  \param q_camera_points Query image 3D points (input).
         *  \param f_c_l_kps Current left image keypoints filtered out (output).
         *  \param f_c_r_kps Current right image keypoints filtered out (output).
         *  \param f_q_l_kps Query left image keypoints filtered out (output).
         *  \param f_q_r_kps Query right image keypoints filtered out (output).
         *  \param f_c_camera_points Current image 3D points filtered out (output).
         *  \param f_q_camera_points Query image 3D points filtered out (output).
         *  @return Whether keypoint matching and filtering has been performed correctly
         *  and if there are enough filtered points.
         */
        bool matchNonStereoKeypoints(double ratio,
                                     const cv::Mat& c_l_descs,
                                     const cv::Mat& c_r_descs,
                                     const cv::Mat& q_l_descs,
                                     const cv::Mat& q_r_descs,
                                     const std::vector<cv::KeyPoint>& c_l_kps,
                                     const std::vector<cv::KeyPoint>& c_r_kps,
                                     const std::vector<cv::KeyPoint>& q_l_kps,
                                     const std::vector<cv::KeyPoint>& q_r_kps,
                                     const std::vector<cv::Point3f>& c_camera_points,
                                     const std::vector<cv::Point3f>& q_camera_points,
                                     std::vector<cv::KeyPoint>& f_c_l_kps,
                                     std::vector<cv::KeyPoint>& f_c_r_kps,
                                     std::vector<cv::KeyPoint>& f_q_l_kps,
                                     std::vector<cv::KeyPoint>& f_q_r_kps,
                                     std::vector<cv::Point3f>& f_c_camera_points,
                                     std::vector<cv::Point3f>& f_q_camera_points);


    protected:

        /** \brief Matches visual keypoints and applies Lowe's ratio.
         *  \param ratio for eliminating outlier matches (input).
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
         *  @return Whether keypoint matching and filtering has been performed correctly
         *  and if there are enough filtered points.
         */
        MatchResult ratioMatching(double ratio,
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


        /** \brief Matches visual keypoints and applies Lowe's ratio.
         *  \param ratio for eliminating outlier matches (input).
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
         *  @return Whether keypoint matching and filtering has been performed correctly
         *  and if there are enough filtered points.
         */
        MatchResult ratioHomographyMatching(double ratio,
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


        bool parseMatchResult(bool enable_printing, 
                              const std::string& text,
                              const MatchResult& result);
        

        MatchType match_type_; //!> Desired type of matching between images that are not of the same stereo pair.

        bool enable_debugging_; //!> Enabling or disabling debugging messages.
};


#endif // STEREO_MOTION_ESTIMATION_MODULES_HPP