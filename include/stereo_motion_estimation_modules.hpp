#ifndef STEREO_MOTION_ESTIMATION_MODULES_HPP
#define STEREO_MOTION_ESTIMATION_MODULES_HPP


#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


class StereoMotionEstimationModules
{
    public:

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
         *  \param f_matches Filtered left-right matches (output).
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
                                  std::vector<cv::DMatch>& f_matches,
                                  std::vector<cv::KeyPoint>& f_l_kps,
                                  std::vector<cv::KeyPoint>& f_r_kps);


        // void matchNonStereoKeypoints();


    protected:

        /** \brief Matches visual keypoints and applies Lowe's ratio.
         *  \param ratio for eliminating outlier matches (input).
         *  \param descs_0 First image descriptors (input).
         *  \param descs_1 Second image descriptors (input).
         *  \param kps_0 First image keypoints (input).
         *  \param kps_1 Second image keypoints (input).
         *  \param points_3d_0 First image 3D points (input).
         *  \param points_3d_1 Second image 3D points (input).
         *  \param f_descs_0 First image descriptors filtered out (output).
         *  \param f_descs_1 Second image descriptors filtered out (output).
         *  \param f_matches Keypoints matches filtered out (output).
         *  \param f_kps_0 First image keypoints filtered out (output).
         *  \param f_kps_1 Second image keypoints filtered out (output).
         *  \param f_points_3d_0 First image 3D points filtered out (output).
         *  \param f_points_3d_1 Second image 3D points filtered out (output).
         *  @return Whether keypoint matching and filtering has been performed correctly
         *  and if there are enough filtered points.
         */
        bool ratioMatching(double ratio,
                           const cv::Mat& descs_0, 
                           const cv::Mat& descs_1,
                           const std::vector<cv::KeyPoint>& kps_0,
                           const std::vector<cv::KeyPoint>& kps_1,
                           const std::vector<cv::Point3f>& points_3d_0,
                           const std::vector<cv::Point3f>& points_3d_1,
                           cv::Mat& f_descs_0,
                           cv::Mat& f_descs_1,
                           std::vector<cv::DMatch>& f_matches,
                           std::vector<cv::KeyPoint>& f_kps_0,
                           std::vector<cv::KeyPoint>& f_kps_1,
                           std::vector<cv::Point3f>& f_points_3d_0,
                           std::vector<cv::Point3f>& f_points_3d_1);
};


#endif // STEREO_MOTION_ESTIMATION_MODULES_HPP