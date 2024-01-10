#include "publisher.hpp"


void Publisher::publishStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const ros::Time& stamp,
                                       const std::vector<cv::KeyPoint>& l_kps,
                                       const std::vector<cv::KeyPoint>& r_kps,
                                       const image_transport::Publisher& publisher)
{
    // Draw the key points in their respective images.
    cv::Mat aux_l_img, aux_r_img;
    cv::Scalar color(0.0, 0.0, 255);
    cv::drawKeypoints(l_img, l_kps, aux_l_img, color);
    cv::drawKeypoints(r_img, r_kps, aux_r_img, color);

    // Write text
    std::stringstream s;
    cv::Mat stereo_txt_img, aux_txt_r_img;
    s << " Number of keypoints: " << l_kps.size();
    writeTxtInImage(aux_l_img, s, stereo_txt_img);
    s.str("");
    s << " Number of keypoints: " << r_kps.size();
    writeTxtInImage(aux_r_img, s, aux_txt_r_img);

    // Concatenates horizontally the painted images.
    cv::hconcat(stereo_txt_img, aux_txt_r_img, stereo_txt_img);

    // Publish image
    publishImage(stereo_txt_img, stamp, publisher);
}


void Publisher::publishStereoMatches(const cv::Mat& l_img,
                                     const cv::Mat& r_img,
                                     const ros::Time& stamp,
                                     const std::vector<cv::DMatch>& matches,
                                     const std::vector<cv::KeyPoint>& l_kps,
                                     const std::vector<cv::KeyPoint>& r_kps,
                                     const image_transport::Publisher& publisher)
{
    // Draw matches
    cv::Mat stereo_img;
    std::vector<char> mask;
    cv::Scalar color(0.0, 0.0, 255);
    cv::drawMatches(l_img, l_kps, r_img, r_kps, matches, stereo_img, color, color, mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Write text
    std::stringstream s;
    cv::Mat stereo_text_img;
    s << " Number of matches: " << matches.size();
    writeTxtInImage(stereo_img, s, stereo_text_img);

    // Publish image
    publishImage(stereo_text_img, stamp, publisher);
}


void Publisher::publishImage(const cv::Mat& img,
                             const ros::Time& stamp,
                             const image_transport::Publisher& publisher)
{
    cv_bridge::CvImage ros_image;
    ros_image.image = img;
    ros_image.header.stamp = stamp;
    ros_image.encoding = "bgr8";
    publisher.publish(ros_image.toImageMsg());
}


void Publisher::writeTxtInImage(const cv::Mat& img,
                                const std::stringstream& s,
                                cv::Mat& txt_img)
{
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1.5, 1, &baseline);
    txt_img = cv::Mat(img.rows + text_size.height + 20, img.cols, img.type());
    img.copyTo(txt_img.rowRange(0, img.rows).colRange(0, img.cols));
    txt_img.rowRange(img.rows, txt_img.rows).setTo(cv::Scalar(255, 255, 255));
    cv::putText(txt_img, s.str(), cv::Point(5, txt_img.rows - 10), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2, 8);
}