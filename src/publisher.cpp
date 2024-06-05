#include "publisher.hpp"


void Publisher::writeKeypoints(const cv::Mat& img,
                               const cv::Scalar& color,
                               const std::string& o_path,
                               const std::vector<cv::Point2f> kpts)
{
    // Draws keypoints.
    cv::Mat o_img = drawPoints(img, color, kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const ros::Time& stamp,
                                       const ros::Publisher& publisher,
                                       const std::vector<cv::Point2f>& l_kpts,
                                       const std::vector<cv::Point2f>& r_kpts)
{
    // Concatenates stereo images and draws their keypoints.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawStereoKeypoints(l_img, r_img, color, l_kpts, r_kpts);

    // Publishes image.
    publishImage(o_img, stamp, publisher);
}


void Publisher::writeStereoKeypoints(const cv::Mat& l_img,
                                     const cv::Mat& r_img,
                                     const std::string& o_path,
                                     const std::vector<cv::Point2f>& l_kpts,
                                     const std::vector<cv::Point2f>& r_kpts)
{
    // Concatenates stereo images and draws their keypoints.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawStereoKeypoints(l_img, r_img, color, l_kpts, r_kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishMatches(const bool vertical, 
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const ros::Time& stamp,
                               const ros::Publisher& publisher,
                               const std::vector<cv::Point2f>& f_kpts_0,
                               const std::vector<cv::Point2f>& f_kpts_1)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat drawn_img = drawMatches(vertical, img_0, img_1, color, f_kpts_0, f_kpts_1);

    // Writes text.
    std::stringstream s;
    s << " Number of matches: " << f_kpts_0.size();
    cv::Mat o_img = writeTxtInImage(drawn_img, s);

    // Publishes image.
    publishImage(o_img, stamp, publisher);
}


void Publisher::writeMatches(const bool vertical, 
                             const cv::Mat& img_0,
                             const cv::Mat& img_1,
                             const std::string& o_path,
                             const std::vector<cv::Point2f>& f_kpts_0,
                             const std::vector<cv::Point2f>& f_kpts_1)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat drawn_img = drawMatches(vertical, img_0, img_1, color, f_kpts_0, f_kpts_1);

    // Writes text.
    std::stringstream s;
    s << " Number of matches: " << f_kpts_0.size();
    cv::Mat o_img = writeTxtInImage(drawn_img, s);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishCircularMatches(const cv::Mat& c_l_img,                                    
                                       const cv::Mat& c_r_img,
                                       const cv::Mat& p_l_img,
                                       const cv::Mat& p_r_img,
                                       const ros::Time& stamp,
                                       const ros::Publisher& publisher,
                                       const std::vector<cv::Point2f>& f_c_l_kpts,
                                       const std::vector<cv::Point2f>& f_c_r_kpts,
                                       const std::vector<cv::Point2f>& f_p_l_kpts,
                                       const std::vector<cv::Point2f>& f_p_r_kpts)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawCircularMatches(c_l_img, c_r_img, p_l_img, p_r_img, color,
                                        f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts);
    
    // Publishes image
    publishImage(o_img, stamp, publisher);
}


void Publisher::writeCircularMatches(const cv::Mat& c_l_img,
                                     const cv::Mat& c_r_img,
                                     const cv::Mat& p_l_img,
                                     const cv::Mat& p_r_img,
                                     const std::string& o_path,
                                     const std::vector<cv::Point2f>& f_c_l_kpts,
                                     const std::vector<cv::Point2f>& f_c_r_kpts,
                                     const std::vector<cv::Point2f>& f_p_l_kpts,
                                     const std::vector<cv::Point2f>& f_p_r_kpts)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawCircularMatches(c_l_img, c_r_img, p_l_img, p_r_img, color,
                                        f_c_l_kpts, f_c_r_kpts, f_p_l_kpts, f_p_r_kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishKeypointsClustering(const cv::Mat& l_img,
                                           const ros::Time& stamp,
                                           const ros::Publisher& publisher,
                                           const std::vector<cv::Point2f>& l_kpts,
                                           const std::vector<std::vector<int>>& clusters)
{
    // Draws keypoints clustering.
    cv::Mat o_img = drawKeypointsClustering(l_img, l_kpts, clusters);

    // Publishes image.
    publishImage(o_img, stamp, publisher);
}


void Publisher::writeKeypointsClustering(const cv::Mat& l_img,
                                         const std::string& o_path,
                                         const std::vector<cv::Point2f>& l_kpts,
                                         const std::vector<std::vector<int>>& clusters)
{
    // Draws keypoints clustering.
    cv::Mat o_img = drawKeypointsClustering(l_img, l_kpts, clusters);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishPose(const ros::Time& c_stamp,
                            const ros::Publisher& publisher,
                            const std::string& world_frame_id,
                            const std::string& robot_frame_id,
                            const tf::Transform& c_robot_pose)
{
    nav_msgs::Odometry msg = buildPoseMsg(c_stamp, world_frame_id, robot_frame_id, c_robot_pose);

    // Publish pose
    publisher.publish(msg);
}
                                            


void Publisher::publishPoseAndVelocity(const ros::Time& c_stamp,
                                       const ros::Time& p_stamp,
                                       const ros::Publisher& publisher,
                                       const std::string& world_frame_id,
                                       const std::string& robot_frame_id,
                                       const tf::Transform& c_robot_pose,
                                       const tf::Transform& c_robot_motion)
{
    nav_msgs::Odometry msg = buildPoseMsg(c_stamp, world_frame_id, robot_frame_id, c_robot_pose);

    // Set linear velocity
    double delta_t = (c_stamp - p_stamp).toSec();
    if (delta_t == 0.0)
    {
        msg.twist.twist.linear.x = 0.0;
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = 0.0;
    }
    else
    {
        msg.twist.twist.linear.x = c_robot_motion.getOrigin().getX() / delta_t;
        msg.twist.twist.linear.y = c_robot_motion.getOrigin().getY() / delta_t;
        msg.twist.twist.linear.z = c_robot_motion.getOrigin().getZ() / delta_t;
        tf::Quaternion delta_rot = c_robot_motion.getRotation();
        tfScalar angle = delta_rot.getAngle();
        tf::Vector3 axis = delta_rot.getAxis();
        tf::Vector3 angular_velocity = (axis * angle) / delta_t;
        msg.twist.twist.angular.x = angular_velocity.x();
        msg.twist.twist.angular.y = angular_velocity.y();
        msg.twist.twist.angular.z = angular_velocity.z();
    }

    // Publish pose and velocity
    publisher.publish(msg);
}


void Publisher::publishInfo(const ros::Time& stamp,
                            const bool valid_motion,
                            const VORunTimes& run_times,
                            const ros::Publisher& publisher)
{
    slamon::VOInfo msg;

    // Sets header
    msg.header.stamp = stamp;

    // Sets payload
    msg.valid_motion = valid_motion;
    msg.feature_extraction_run_time = run_times.feature_extraction;
    msg.stereo_matching_run_time = run_times.stereo_matching;
    msg.triangulation_run_time = run_times.triangulation;
    msg.successive_matching_run_time = run_times.successive_matching;
    msg.motion_estimation_run_time = run_times.motion_estimation;
    msg.total_run_time = run_times.total;

    // Publishes performance information
    publisher.publish(msg);
}   


void Publisher::publishFrame(const cv::Mat& l_descs,
                             const cv::Mat& r_descs,
                             const ros::Time& stamp,
                             const ros::Publisher& publisher,
                             const std::vector<cv::Point2f>& l_kpts,
                             const std::vector<cv::Point2f>& r_kpts,
                             const std::vector<cv::Point3f>& obj_pts)
{
    // Create and fill frame msg
    slamon::Frame frame_msg;
    geometry_msgs::Point obj_pt_msg;
    slamon::Keypoint l_kpt_msg, r_kpt_msg;
    frame_msg.header.stamp = stamp;
    for (uint i = 0; i < l_kpts.size(); i++)
    {
        // Fill base msgs
        obj_pt_msg.x = obj_pts[i].x;
        obj_pt_msg.y = obj_pts[i].y;
        obj_pt_msg.z = obj_pts[i].z;
        l_kpt_msg.pt.x = l_kpts[i].x;
        l_kpt_msg.pt.y = l_kpts[i].y;
        r_kpt_msg.pt.x = r_kpts[i].x;
        r_kpt_msg.pt.y = r_kpts[i].y;
        l_kpt_msg.desc = l_descs.row(i);
        r_kpt_msg.desc = r_descs.row(i);

        // Fill Frame msg
        frame_msg.l_kpts.push_back(l_kpt_msg);
        frame_msg.r_kpts.push_back(r_kpt_msg);
        frame_msg.obj_pts.push_back(obj_pt_msg);
    }

    // Publishes 
    publisher.publish(frame_msg);
}


void Publisher::writePoses(const std::string& o_path,
                           const std::vector<double>& stamps,
                           const std::vector<tf::Transform>& poses)
{
    std::string l_path = o_path + ".lock";

    // Wait until lock file has been released.
    while(std::filesystem::exists(l_path));

    // Create a locking element.
    std::fstream lock_file(l_path.c_str(), std::ios::out | std::ios::trunc);

    // Open to append.
    std::fstream poses_file(o_path.c_str(), std::ios::out | std::ios::trunc);

    // First line
    poses_file << std::fixed << std::setprecision(9) << "timestamp,x,y,z,qx,qy,qz,qw" << std::endl;

    // Write poses and timestamps
    tf::Transform pose;
    for (uint i = 0; i < poses.size(); i++)
    {
        pose = poses[i];
        poses_file << 
        stamps[i] << "," <<
        pose.getOrigin().getX() << "," <<
        pose.getOrigin().getY() << "," <<
        pose.getOrigin().getZ() << "," <<
        pose.getRotation().getX() << "," <<
        pose.getRotation().getY() << "," <<
        pose.getRotation().getZ() << "," <<
        pose.getRotation().getW() << std::endl;
    }

    poses_file.close();

    // Unlock
    poses_file.close();
    bool ok = std::filesystem::remove(l_path.c_str());
    if (!ok)
        ROS_ERROR("[Publisher->writePoses]: Error deleting the locking file!");
}


cv::Mat Publisher::drawPoints(const cv::Mat& img,
                              const cv::Scalar& color,
                              const std::vector<cv::Point2f>& pts)
{
    cv::Mat o_img;
    img.copyTo(o_img);

    // Draw points
    for (uint i = 0; i < pts.size(); i++)
        cv::circle(o_img, pts[i], 2, color, -1);

    return o_img;
}


cv::Mat Publisher::writeTxtInImage(const cv::Mat& img,
                                   const std::stringstream& s)
{
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1.5, 1, &baseline);
    cv::Mat o_img = cv::Mat(img.rows + text_size.height + 20, img.cols, img.type());
    img.copyTo(o_img.rowRange(0, img.rows).colRange(0, img.cols));
    o_img.rowRange(img.rows, o_img.rows).setTo(cv::Scalar(255, 255, 255));
    cv::putText(o_img, s.str(), cv::Point(5, o_img.rows - 10), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2, 8);

    return o_img;
}


cv::Mat Publisher::drawStereoKeypoints(const cv::Mat& l_img,
                                       const cv::Mat& r_img,
                                       const cv::Scalar& color,
                                       const std::vector<cv::Point2f>& l_kpts,
                                       const std::vector<cv::Point2f>& r_kpts)
{
    // Draw the key points in their respective images.
    cv::Mat drawn_l_img = drawPoints(l_img, color, l_kpts);
    cv::Mat drawn_r_img = drawPoints(r_img, color, r_kpts);

    // Write text
    std::stringstream s;
    s << " Number of keypoints: " << l_kpts.size();
    drawn_l_img = writeTxtInImage(drawn_l_img, s);
    s.str("");
    s << " Number of keypoints: " << r_kpts.size();
    drawn_r_img = writeTxtInImage(drawn_r_img, s);
    
    // Concatenates horizontally the painted images.
    cv::Mat o_img;
    cv::hconcat(drawn_l_img, drawn_r_img, o_img);

    return o_img;
}


cv::Mat Publisher::drawMatches(const bool vertical,
                               const cv::Mat& img_0,
                               const cv::Mat& img_1,
                               const cv::Scalar& color,
                               const std::vector<cv::Point2f>& f_kpts_0,
                               const std::vector<cv::Point2f>& f_kpts_1)
{
    // Image concatenation and offset assignment
    int offset;
    cv::Mat o_img;
    if (vertical)
    {
        cv::vconcat(img_0, img_1, o_img);
        offset = img_0.rows;
    }
    else
    {
        cv::hconcat(img_0, img_1, o_img);
        offset = img_0.cols;
    }

    // Draw keypoints and matches
    cv::Point2f f_q_kpt_offset;
    for (uint i = 0; i < f_kpts_0.size(); i++)
    {
        f_q_kpt_offset = f_kpts_1[i];
        if (vertical)
            f_q_kpt_offset.y += offset;
        else
            f_q_kpt_offset.x += offset;

        cv::circle(o_img, f_kpts_0[i], 4, color, 1, cv::LINE_AA, 0);
        cv::circle(o_img, f_q_kpt_offset, 4, color, 1, cv::LINE_AA, 0);
        cv::line(o_img, f_kpts_0[i], f_q_kpt_offset, color, 1, cv::LINE_AA, 0);
    }

    return o_img;
}


cv::Mat Publisher::drawCircularMatches(const cv::Mat& c_l_img,                                    
                                       const cv::Mat& c_r_img,
                                       const cv::Mat& p_l_img,
                                       const cv::Mat& p_r_img,
                                       const cv::Scalar& color,
                                       const std::vector<cv::Point2f>& f_c_l_kpts,
                                       const std::vector<cv::Point2f>& f_c_r_kpts,
                                       const std::vector<cv::Point2f>& f_p_l_kpts,
                                       const std::vector<cv::Point2f>& f_p_r_kpts)
{
    // Concatenates horizontal images and draws matches
    cv::Mat c_stereo_img = drawMatches(false, c_l_img, c_r_img, color, f_c_l_kpts, f_c_r_kpts);
    cv::Mat p_stereo_img = drawMatches(false, p_l_img, p_r_img, color, f_p_l_kpts, f_p_r_kpts);

    // Vertical concatenation
    cv::Mat drawn_stereo_img;
    cv::vconcat(c_stereo_img, p_stereo_img, drawn_stereo_img);

    // Draw vertical matches
    int x_offset = c_l_img.cols;
    int y_offset = c_l_img.rows;
    cv::Point2f f_c_r_pt_offset, f_p_l_pt_offset, f_p_r_pt_offset;
    for (uint i = 0; i < f_c_l_kpts.size(); i++)
    {
        // Add offset to filtered current right keypoints
        f_c_r_pt_offset = f_c_r_kpts[i];
        f_c_r_pt_offset.x += x_offset;
        // Add offset to filtered previous left keypoints
        f_p_l_pt_offset = f_p_l_kpts[i];
        f_p_l_pt_offset.y += y_offset;
        // Add offset to filtered previous right keypoints
        f_p_r_pt_offset = f_p_r_kpts[i];
        f_p_r_pt_offset.x += x_offset;
        f_p_r_pt_offset.y += y_offset;

        cv::line(drawn_stereo_img, f_c_l_kpts[i], f_p_l_pt_offset, color, 1, cv::LINE_AA, 0);
        cv::line(drawn_stereo_img, f_c_r_pt_offset, f_p_r_pt_offset, color, 1, cv::LINE_AA, 0);
    }

    // Writes text
    std::stringstream s;
    s << " Number of matches: " << f_c_l_kpts.size();
    cv::Mat o_img = writeTxtInImage(drawn_stereo_img, s);

    return o_img;
}


cv::Mat Publisher::drawKeypointsClustering(const cv::Mat& img,
                                           const std::vector<cv::Point2f>& kpts,
                                           const std::vector<std::vector<int>>& clusters)
{
    cv::Mat drawn_img;
    img.copyTo(drawn_img);

    if (clusters.size() == 0)
        return drawn_img;

    cv::RNG rng(12345);
    for (uint i = 0; i < clusters.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        for (uint j = 0; j < clusters[i].size(); j++)
            cv::circle(drawn_img, kpts[clusters[i][j]], 2, color, -1);
    }

    // Writes text
    std::stringstream s;
    s << " Number of clusters: " << clusters.size();
    cv::Mat o_img = writeTxtInImage(drawn_img, s);

    return o_img;
}


nav_msgs::Odometry Publisher::buildPoseMsg(const ros::Time& c_stamp,
                                           const std::string& world_frame_id,
                                           const std::string& robot_frame_id,
                                           const tf::Transform& c_robot_pose)
{
    nav_msgs::Odometry msg;

    // Set header
    msg.header.stamp = c_stamp;
    msg.header.frame_id = world_frame_id;
    msg.child_frame_id = robot_frame_id;

    // Set pose
    tf::poseTFToMsg(c_robot_pose, msg.pose.pose);

    return msg;
}


void Publisher::publishImage(const cv::Mat& img,
                             const ros::Time& stamp,
                             const ros::Publisher& publisher)
{
    cv_bridge::CvImage ros_image;
    ros_image.image = img;
    ros_image.header.stamp = stamp;
    ros_image.encoding = "bgr8";
    publisher.publish(ros_image.toImageMsg());
}