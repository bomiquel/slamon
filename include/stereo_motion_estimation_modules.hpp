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

    protected:

    private:
};


#endif // STEREO_MOTION_ESTIMATION_MODULES_HPP