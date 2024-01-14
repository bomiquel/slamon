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


void Publisher::publishMatches(bool vertical, 
                               const cv::Mat& c_img,
                               const cv::Mat& q_img,
                               const ros::Time& stamp,
                               const std::vector<cv::KeyPoint>& f_c_kps,
                               const std::vector<cv::KeyPoint>& f_q_kps,
                               const image_transport::Publisher& publisher)
{
    // Draw matches
    cv::Mat out_img;
    cv::Scalar color(0.0, 0.0, 255);
    drawMatches(vertical, c_img, q_img, color, f_c_kps, f_q_kps, out_img);

    // Write text
    std::stringstream s;
    cv::Mat out_text_img;
    s << " Number of matches: " << f_c_kps.size();
    writeTxtInImage(out_img, s, out_text_img);

    // Publish image
    publishImage(out_text_img, stamp, publisher);
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


void Publisher::drawMatches(bool vertical,
                            const cv::Mat& c_img,
                            const cv::Mat& q_img,
                            const cv::Scalar& color,
                            const std::vector<cv::KeyPoint> f_c_kps,
                            const std::vector<cv::KeyPoint> f_q_kps,
                            cv::Mat& out_img)
{
    // Image concatenation and offset assignment
    int offset;
    if (vertical)
    {
        cv::vconcat(c_img, q_img, out_img);
        offset = c_img.rows;
    }
    else
    {
        cv::hconcat(c_img, q_img, out_img);
        offset = c_img.cols;
    }

    // Draw keypoints and matches
    cv::Point2f f_q_kp_offset;
    for (uint i = 0; i < f_c_kps.size(); i++)
    {
        f_q_kp_offset = f_q_kps[i].pt;
        if (vertical)
            f_q_kp_offset.y += offset;
        else
            f_q_kp_offset.x += offset;

        cv::circle(out_img, f_c_kps[i].pt, 4, color, 1, cv::LINE_AA, 0);
        cv::circle(out_img, f_q_kp_offset, 4, color, 1, cv::LINE_AA, 0);
        cv::line(out_img, f_c_kps[i].pt, f_q_kp_offset, color, 1, cv::LINE_AA, 0);
    }
}