#include "publisher.hpp"


void Publisher::writeImgKeypoints(const cv::Mat& img,
                                  const cv::Scalar& color,
                                  const std::string& o_path,
                                  const std::vector<cv::KeyPoint>& kpts)
{
    // Draws keypoints.
    cv::Mat o_img = drawImgKeypoints(img, color, kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishImgStereoKeypoints(const cv::Mat& l_img,
                                          const cv::Mat& r_img,
                                          const ros::Time& stamp,
                                          const ros::Publisher& publisher,
                                          const std::vector<cv::KeyPoint>& l_kpts,
                                          const std::vector<cv::KeyPoint>& r_kpts)
{
    // Concatenates stereo images and draws their keypoints.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawImgStereoKeypoints(l_img, r_img, color, l_kpts, r_kpts);

    // Publishes image.
    publishImg(o_img, stamp, publisher);
}


void Publisher::writeImgStereoKeypoints(const cv::Mat& l_img,
                                        const cv::Mat& r_img,
                                        const std::string& o_path,
                                        const std::vector<cv::KeyPoint>& l_kpts,
                                        const std::vector<cv::KeyPoint>& r_kpts)
{
    // Concatenates stereo images and draws their keypoints.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawImgStereoKeypoints(l_img, r_img, color, l_kpts, r_kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishImgMatches(const bool vertical, 
                                  const cv::Mat& img_0,
                                  const cv::Mat& img_1,
                                  const ros::Time& stamp,
                                  const ros::Publisher& publisher,
                                  const std::vector<cv::KeyPoint>& m_kpts_0,
                                  const std::vector<cv::KeyPoint>& m_kpts_1)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat drawn_img = drawImgMatches(vertical, img_0, img_1, color, m_kpts_0, m_kpts_1);

    // Writes text.
    std::stringstream caption;
    caption << " Number of matches: " << m_kpts_0.size();
    cv::Mat o_img = drawTxtInImg(true, drawn_img, caption, 20);

    // Publishes image.
    publishImg(o_img, stamp, publisher);
}


void Publisher::writeImgMatches(const bool vertical, 
                                const cv::Mat& img_0,
                                const cv::Mat& img_1,
                                const std::string& o_path,
                                const std::vector<cv::KeyPoint>& m_kpts_0,
                                const std::vector<cv::KeyPoint>& m_kpts_1)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat drawn_img = drawImgMatches(vertical, img_0, img_1, color, m_kpts_0, m_kpts_1);

    // Writes text.
    std::stringstream caption;
    caption << " Number of matches: " << m_kpts_0.size();
    cv::Mat o_img = drawTxtInImg(true, drawn_img, caption, 20);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishImgCircularMatches(const cv::Mat& c_l_img,                                    
                                          const cv::Mat& c_r_img,
                                          const cv::Mat& p_l_img,
                                          const cv::Mat& p_r_img,
                                          const ros::Time& stamp,
                                          const ros::Publisher& publisher,
                                          const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                          const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                          const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                          const std::vector<cv::KeyPoint>& m_p_r_kpts)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawImgCircularMatches(c_l_img, c_r_img, p_l_img, p_r_img, color,
                                           m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts);
    
    // Publishes image
    publishImg(o_img, stamp, publisher);
}


void Publisher::writeImgCircularMatches(const cv::Mat& c_l_img,
                                        const cv::Mat& c_r_img,
                                        const cv::Mat& p_l_img,
                                        const cv::Mat& p_r_img,
                                        const std::string& o_path,
                                        const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                        const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                        const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                        const std::vector<cv::KeyPoint>& m_p_r_kpts)
{
    // Draws matches.
    cv::Scalar color(0.0, 0.0, 255);
    cv::Mat o_img = drawImgCircularMatches(c_l_img, c_r_img, p_l_img, p_r_img, color,
                                           m_c_l_kpts, m_c_r_kpts, m_p_l_kpts, m_p_r_kpts);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishImgKeypointsClustering(const cv::Mat& l_img,
                                              const ros::Time& stamp,
                                              const ros::Publisher& publisher,
                                              const std::vector<cv::KeyPoint>& l_kpts,
                                              const std::vector<std::vector<int>>& clusters)
{
    // Draws keypoints clustering.
    cv::Mat o_img = drawImgKeypointsClustering(l_img, l_kpts, clusters);

    // Publishes image.
    publishImg(o_img, stamp, publisher);
}


void Publisher::writeImgKeypointsClustering(const cv::Mat& l_img,
                                            const std::string& o_path,
                                            const std::vector<cv::KeyPoint>& l_kpts,
                                            const std::vector<std::vector<int>>& clusters)
{
    // Draws keypoints clustering.
    cv::Mat o_img = drawImgKeypointsClustering(l_img, l_kpts, clusters);

    // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishImgLoopClosure(const ros::Time& stamp,
                                      const ros::Publisher& publisher,
                                      const int q_keyframe_id,
                                      const std::string& i_path,
                                      const std::vector<int>& q_cluster_ids,
                                      const std::vector<int>& cand_cluster_ids,
                                      const std::vector<int>& cand_keyframe_ids,
                                      const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                      const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                      const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                      const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                      const std::vector<int>& definitive_inliers_per_pair,
                                      const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                      const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation)
{
    // Draws loop closures.
    cv::Mat o_img = drawImgLoopClosure(q_keyframe_id,
                                       i_path,
                                       q_cluster_ids,
                                       cand_cluster_ids,
                                       cand_keyframe_ids,
                                       f_q_l_kpts,
                                       f_q_r_kpts,
                                       f_cand_l_kpts,
                                       f_cand_r_kpts,
                                       definitive_inliers_per_pair,
                                       definitive_cluster_pairs,
                                       cand_cluster_keyframe_relation);

    // Publish image.
    publishImg(o_img, stamp, publisher);
}


void Publisher::writeImgLoopClosure(const std::string& o_path,
                                    const int q_keyframe_id,
                                    const std::string& i_path,
                                    const std::vector<int>& q_cluster_ids,
                                    const std::vector<int>& cand_cluster_ids,
                                    const std::vector<int>& cand_keyframe_ids,
                                    const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                    const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                    const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                    const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                    const std::vector<int>& definitive_inliers_per_pair,
                                    const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                    const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation)
{
    // Draws loop closures.
    cv::Mat o_img = drawImgLoopClosure(q_keyframe_id,
                                       i_path,
                                       q_cluster_ids,
                                       cand_cluster_ids,
                                       cand_keyframe_ids,
                                       f_q_l_kpts,
                                       f_q_r_kpts,
                                       f_cand_l_kpts,
                                       f_cand_r_kpts,
                                       definitive_inliers_per_pair,
                                       definitive_cluster_pairs,
                                       cand_cluster_keyframe_relation);

     // Writes image.
    cv::imwrite(o_path + ".png", o_img);
}


void Publisher::publishPose(const ros::Time& c_stamp,
                            const tf::Transform& c_pose,
                            const ros::Publisher& publisher,
                            const std::string& reference_frame_id)
{
    nav_msgs::Odometry msg = buildPoseMsg(c_stamp, c_pose, reference_frame_id);

    // Publish pose
    publisher.publish(msg);
}
                                            

void Publisher::publishPoseAndVelocity(const ros::Time& c_stamp,
                                       const ros::Time& p_stamp,
                                       const tf::Transform& c_pose,
                                       const tf::Transform& c_motion,
                                       const ros::Publisher& publisher,
                                       const std::string& reference_frame_id)
{
    nav_msgs::Odometry msg = buildPoseMsg(c_stamp, c_pose, reference_frame_id);

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
        msg.twist.twist.linear.x = c_motion.getOrigin().getX() / delta_t;
        msg.twist.twist.linear.y = c_motion.getOrigin().getY() / delta_t;
        msg.twist.twist.linear.z = c_motion.getOrigin().getZ() / delta_t;
        tf::Quaternion delta_rot = c_motion.getRotation();
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


void Publisher::publishPerformance(const ros::Time& stamp,
                                   const int successes_num,
                                   const RunTimes& run_times,
                                   const ros::Publisher& publisher)
{
    // Creates custom message.
    slamon::Performance msg;

    // Sets header.
    msg.header.stamp = stamp;

    // Sets payload.
    msg.successes_num = successes_num;
    msg.run_time_0 = run_times.run_time_0; // Time needed for VO: Feature extraction, MM: From Frame msg to Frame object or LC: Process current cluster.
    msg.run_time_1 = run_times.run_time_1; // Time needed for VO: Stereo matching, MM: Need new keyframe? or LC: Search for candidates to close loops by proximity.
    msg.run_time_2 = run_times.run_time_2; // Time needed for VO: Triangulation, MM: Add vertex (cluster) to graph or LC: Search for candidates to close loops by hash.
    msg.run_time_3 = run_times.run_time_3; // Time needed for VO: temporal matching, MM: Add edges to graph or LC: Read cluster candidates.
    msg.run_time_4 = run_times.run_time_4; // Time needed for VO: Bucketing, MM: Write graph to disk and publish updated robot pose or LC: Loop detection.
    msg.run_time_5 = run_times.run_time_5; // Time needed for VO: Motion estimation, MM: Nothing or LC: Close loop.
    msg.total_run_time = run_times.total_run_time; // Time needed for each thread to perform one iteration.

    publisher.publish(msg);
}


void Publisher::publishCloud(const ros::Publisher& publisher,
                             const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    publisher.publish(cloud_msg);
}


void Publisher::publishStereoImageAttributes(const cv::Mat& l_descs,
                                             const cv::Mat& r_descs,
                                             const ros::Time& stamp,
                                             const ros::Publisher& publisher,
                                             const std::vector<cv::KeyPoint>& l_kpts,
                                             const std::vector<cv::KeyPoint>& r_kpts,
                                             const std::vector<cv::Point3f>& l_cam_pts)
{
    // Create and fill StereoImageAttributes msg.
    slamon::StereoImageAttributes msg;
    geometry_msgs::Point cam_pt_msg;
    slamon::Keypoint l_kpt_msg, r_kpt_msg;
    msg.header.stamp = stamp;
    for (unsigned int i = 0; i < l_kpts.size(); i++)
    {
        // Fill base msgs
        cam_pt_msg.x = l_cam_pts[i].x;
        cam_pt_msg.y = l_cam_pts[i].y;
        cam_pt_msg.z = l_cam_pts[i].z;
        l_kpt_msg.pt.x = l_kpts[i].pt.x;
        l_kpt_msg.pt.y = l_kpts[i].pt.y;
        r_kpt_msg.pt.x = r_kpts[i].pt.x;
        r_kpt_msg.pt.y = r_kpts[i].pt.y;
        l_kpt_msg.desc = l_descs.row(i);
        r_kpt_msg.desc = r_descs.row(i);

        // Fill StereoImageAttributes msg.
        msg.l_kpts.push_back(l_kpt_msg);
        msg.r_kpts.push_back(r_kpt_msg);
        msg.l_cam_pts.push_back(cam_pt_msg);
    }

    // Publishes.
    publisher.publish(msg);
}


void Publisher::publishCounter(const int counter,
                               const ros::Publisher& publisher)
{
    std_msgs::Int32 msg;
    msg.data = counter;
    publisher.publish(msg);
}


void Publisher::writePoses(const std::string& o_path,
                           const std::vector<double>& stamps,
                           const std::vector<tf::Transform>& poses)
{
    std::string l_path = o_path + ".lock";

    // Wait until lock file has been released.
    while (std::filesystem::exists(l_path));

    // Create a locking element.
    std::fstream lock_file(l_path.c_str(), std::ios::out | std::ios::trunc);

    // Open to write.
    std::fstream poses_file(o_path.c_str(), std::ios::out | std::ios::trunc);

    // First line
    poses_file << std::fixed << std::setprecision(9) << "timestamp,id,x,y,z,qx,qy,qz,qw" << std::endl;

    // Write poses and timestamps
    for (unsigned int i = 0; i < poses.size(); i++)
    {
        poses_file << 
        stamps[i] << "," <<
        i << "," <<
        poses[i].getOrigin().getX() << "," <<
        poses[i].getOrigin().getY() << "," <<
        poses[i].getOrigin().getZ() << "," <<
        poses[i].getRotation().getX() << "," <<
        poses[i].getRotation().getY() << "," <<
        poses[i].getRotation().getZ() << "," <<
        poses[i].getRotation().getW() << std::endl;
    }
    poses_file.close();

    // Unlock
    bool ok = std::filesystem::remove(l_path.c_str());
    if (!ok)
        ROS_ERROR("[Publisher->writePoses]: Error deleting the locking file!");
}


void Publisher::writeLoopClosureEdges(const std::string& o_path,
                                      const std::vector<std::pair<int, int>> edges_ids,
                                      const std::vector<std::pair<tf::Transform, tf::Transform>>& edges_poses)
{
    std::string l_path = o_path + ".lock";

     // Wait until lock file has been released.
    while(std::filesystem::exists(l_path));

    // Create a locking element.
    std::fstream lock_file(l_path.c_str(), std::ios::out | std::ios::trunc);

    // Open to write.
    std::fstream edges_file(o_path.c_str(), std::ios::out | std::ios::trunc);

    // First line
    edges_file << std::fixed << std::setprecision(9) << "frame a,frame b,ax,ay,az,aqx,aqy,aqz,aqw,bx,by,bz,bqx,bqy,bqz,bqw" << std::endl;

    // Write 
    for (unsigned int i = 0; i < edges_ids.size(); i++)
    {
        edges_file <<
        edges_ids[i].first << "," <<
        edges_ids[i].second << "," <<
        edges_poses[i].first.getOrigin().getX() << "," <<
        edges_poses[i].first.getOrigin().getY() << "," <<
        edges_poses[i].first.getOrigin().getZ() << "," <<
        edges_poses[i].first.getRotation().getX() << "," <<
        edges_poses[i].first.getRotation().getY() << "," <<
        edges_poses[i].first.getRotation().getZ() << "," <<
        edges_poses[i].first.getRotation().getW() << "," <<
        edges_poses[i].second.getOrigin().getX() << "," <<
        edges_poses[i].second.getOrigin().getY() << "," <<
        edges_poses[i].second.getOrigin().getZ() << "," <<
        edges_poses[i].second.getRotation().getX() << "," <<
        edges_poses[i].second.getRotation().getY() << "," <<
        edges_poses[i].second.getRotation().getZ() << "," <<
        edges_poses[i].second.getRotation().getW() << std::endl;
    }
    edges_file.close();

    // Unlock
    bool ok = std::filesystem::remove(l_path.c_str());
    if (!ok)
        ROS_ERROR("[Publisher->writeLoopClosureEdges]: Error deleting the locking file!");
}


std::string Publisher::convertTo5Digits(int in)
{
    unsigned int val = (unsigned int)in;
    std::string output;
    int digits = 5;
    while (digits-- > 0) 
    {
        output += ('0' + val % 10);
        val /= 10;
    }
    std::reverse(output.begin(), output.end());
    return output;
}


cv::Mat Publisher::drawImgKeypoints(const cv::Mat& img,
                                    const cv::Scalar& color,
                                    const std::vector<cv::KeyPoint>& kpts)
{
    cv::Mat o_img;
    img.copyTo(o_img);

    // Draw points
    for (unsigned int i = 0; i < kpts.size(); i++)
        cv::circle(o_img, kpts[i].pt, 2, color, -1);

    return o_img;
}


cv::Mat Publisher::drawTxtInImg(const bool caption,
                                const cv::Mat& img,
                                const std::stringstream& text,
                                const int extra_vertical_space)
{
    
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_PLAIN, 1.5, 1, &baseline);
    cv::Mat o_img = cv::Mat(img.rows + text_size.height + extra_vertical_space, img.cols, img.type());
    if (caption)
    {   
        img.copyTo(o_img.rowRange(0, img.rows).colRange(0, img.cols));
        o_img.rowRange(img.rows, o_img.rows).setTo(cv::Scalar(255, 255, 255));
        cv::putText(o_img, text.str(), cv::Point(5, o_img.rows - 10), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2, 8);
    }
    else
    {
        img.copyTo(o_img.rowRange(text_size.height + extra_vertical_space, img.rows + text_size.height + extra_vertical_space).colRange(0, img.cols));
        o_img.rowRange(0, text_size.height + extra_vertical_space).setTo(cv::Scalar(255, 255, 255));
        cv::putText(o_img, text.str(), cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2, 8);
    }
    return o_img;
}


cv::Mat Publisher::drawImgStereoKeypoints(const cv::Mat& l_img,
                                          const cv::Mat& r_img,
                                          const cv::Scalar& color,
                                          const std::vector<cv::KeyPoint>& l_kpts,
                                          const std::vector<cv::KeyPoint>& r_kpts)
{
    // Draw the key points in their respective images.
    cv::Mat drawn_l_img = drawImgKeypoints(l_img, color, l_kpts);
    cv::Mat drawn_r_img = drawImgKeypoints(r_img, color, r_kpts);

    // Write text
    std::stringstream caption;
    caption << " Number of keypoints: " << l_kpts.size();
    drawn_l_img = drawTxtInImg(true, drawn_l_img, caption, 20);
    caption.str("");
    caption << " Number of keypoints: " << r_kpts.size();
    drawn_r_img = drawTxtInImg(true, drawn_r_img, caption, 20);
    
    // Concatenates horizontally the painted images.
    cv::Mat o_img;
    cv::hconcat(drawn_l_img, drawn_r_img, o_img);

    return o_img;
}


cv::Mat Publisher::drawImgMatches(const bool vertical,
                                  const cv::Mat& img_0,
                                  const cv::Mat& img_1,
                                  const cv::Scalar& color,
                                  const std::vector<cv::KeyPoint>& m_kpts_0,
                                  const std::vector<cv::KeyPoint>& m_kpts_1)
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
    cv::Point2f m_q_kpt_offset;
    for (unsigned int i = 0; i < m_kpts_0.size(); i++)
    {
        m_q_kpt_offset = m_kpts_1[i].pt;
        if (vertical)
            m_q_kpt_offset.y += offset;
        else
            m_q_kpt_offset.x += offset;

        cv::circle(o_img, m_kpts_0[i].pt, 4, color, 1, cv::LINE_AA, 0);
        cv::circle(o_img, m_q_kpt_offset, 4, color, 1, cv::LINE_AA, 0);
        cv::line(o_img, m_kpts_0[i].pt, m_q_kpt_offset, color, 1, cv::LINE_AA, 0);
    }

    return o_img;
}


cv::Mat Publisher::drawImgCircularMatches(const cv::Mat& c_l_img,                                    
                                          const cv::Mat& c_r_img,
                                          const cv::Mat& p_l_img,
                                          const cv::Mat& p_r_img,
                                          const cv::Scalar& color,
                                          const std::vector<cv::KeyPoint>& m_c_l_kpts,
                                          const std::vector<cv::KeyPoint>& m_c_r_kpts,
                                          const std::vector<cv::KeyPoint>& m_p_l_kpts,
                                          const std::vector<cv::KeyPoint>& m_p_r_kpts)
{
    // Concatenates horizontal images and draws matches
    cv::Mat c_stereo_img = drawImgMatches(false, c_l_img, c_r_img, color, m_c_l_kpts, m_c_r_kpts);
    cv::Mat p_stereo_img = drawImgMatches(false, p_l_img, p_r_img, color, m_p_l_kpts, m_p_r_kpts);

    // Vertical concatenation
    cv::Mat drawn_stereo_img;
    cv::vconcat(c_stereo_img, p_stereo_img, drawn_stereo_img);

    // Draw vertical matches
    int x_offset = c_l_img.cols;
    int y_offset = c_l_img.rows;
    cv::Point2f m_c_r_pt_offset, m_p_l_pt_offset, m_p_r_pt_offset;
    for (unsigned int i = 0; i < m_c_l_kpts.size(); i++)
    {
        // Add offset to filtered current right keypoints
        m_c_r_pt_offset = m_c_r_kpts[i].pt;
        m_c_r_pt_offset.x += x_offset;
        // Add offset to filtered previous left keypoints
        m_p_l_pt_offset = m_p_l_kpts[i].pt;
        m_p_l_pt_offset.y += y_offset;
        // Add offset to filtered previous right keypoints
        m_p_r_pt_offset = m_p_r_kpts[i].pt;
        m_p_r_pt_offset.x += x_offset;
        m_p_r_pt_offset.y += y_offset;

        cv::line(drawn_stereo_img, m_c_l_kpts[i].pt, m_p_l_pt_offset, color, 1, cv::LINE_AA, 0);
        cv::line(drawn_stereo_img, m_c_r_pt_offset, m_p_r_pt_offset, color, 1, cv::LINE_AA, 0);
    }

    // Writes text
    std::stringstream caption;
    caption << " Number of matches: " << m_c_l_kpts.size();
    cv::Mat o_img = drawTxtInImg(true, drawn_stereo_img, caption, 20);

    return o_img;
}


cv::Mat Publisher::drawImgKeypointsClustering(const cv::Mat& img,
                                              const std::vector<cv::KeyPoint>& kpts,
                                              const std::vector<std::vector<int>>& clusters)
{
    cv::Mat drawn_img;
    img.copyTo(drawn_img);

    if (clusters.size() == 0)
        return drawn_img;

    cv::RNG rng(12345);
    for (unsigned int i = 0; i < clusters.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        for (unsigned int j = 0; j < clusters[i].size(); j++)
            cv::circle(drawn_img, kpts[clusters[i][j]].pt, 2, color, -1);
    }

    // Writes text
    std::stringstream caption;
    caption << " Number of clusters: " << clusters.size();
    cv::Mat o_img = drawTxtInImg(true, drawn_img, caption, 20);

    return o_img;
}


cv::Mat Publisher::drawImgLoopClosure(const int q_keyframe_id,
                                      const std::string& i_path,
                                      const std::vector<int>& q_cluster_ids,
                                      const std::vector<int>& cand_cluster_ids,
                                      const std::vector<int>& cand_keyframe_ids,
                                      const std::vector<cv::KeyPoint>& f_q_l_kpts,
                                      const std::vector<cv::KeyPoint>& f_q_r_kpts,
                                      const std::vector<cv::KeyPoint>& f_cand_l_kpts,
                                      const std::vector<cv::KeyPoint>& f_cand_r_kpts,
                                      const std::vector<int>& definitive_inliers_per_pair,
                                      const std::vector<std::pair<int, int>>& definitive_cluster_pairs,
                                      const std::vector<std::pair<int, int>>& cand_cluster_keyframe_relation)
{
    // Local variables.
    std::stringstream text;
    cv::Mat img, o_img, written_img;
 
    // Builds candidate keyframe vector.
    cv::Mat cand_keyframe_vector_img;
    for (unsigned int i = 0; i < cand_keyframe_ids.size(); i++)
    {
        // Reads candidate keyframes.
        img = cv::imread(i_path + convertTo5Digits(cand_keyframe_ids[i]) + "_left.png", cv::IMREAD_COLOR);

        // Adds caption to candidate keyframe.
        text.str("");
        text << "Keyframe: " << cand_keyframe_ids[i];
        written_img = drawTxtInImg(true, img, text, 20);

        // Horizontal concatenation of candidate keyframes.
        if (cand_keyframe_vector_img.cols == 0)
            written_img.copyTo(cand_keyframe_vector_img);
        else
            cv::hconcat(cand_keyframe_vector_img, written_img, cand_keyframe_vector_img);
    }

    // Reads query keyframe.
    img = cv::imread(i_path + convertTo5Digits(q_keyframe_id) + "_left.png", cv::IMREAD_COLOR);

    // Count definitive inliers.
    int definitive_inliers_counter = 0;
    for (unsigned int i = 0; i < definitive_inliers_per_pair.size(); i++)
        definitive_inliers_counter += definitive_inliers_per_pair[i];

    // Adds title to query keyframe.
    text.str("");
    text << "Keyframe " << q_keyframe_id;
    written_img = drawTxtInImg(false, img, text, 20);

    // Gets query offset to compose query and candidates keyframes correctly and to draw the keypoints in the correct location.
    int q_y_offset = written_img.rows - img.rows;
    int q_x_offset = std::round((cand_keyframe_vector_img.cols / 2) - (written_img.cols / 2));
    
    // Composes query keyframe with candidate keyframes.
    cv::Mat q_written_spaced_img(written_img.rows, cand_keyframe_vector_img.cols, cand_keyframe_vector_img.type(), cv::Scalar(0, 0, 0));
    written_img.copyTo(q_written_spaced_img(cv::Rect(q_x_offset, 0, written_img.cols, written_img.rows)));
    cv::vconcat(q_written_spaced_img, cand_keyframe_vector_img, o_img);

    // Builds the vector of colors.
    cv::RNG rng(12345);
    std::vector<cv::Scalar> colors;
    for (unsigned int i = 0; i < definitive_inliers_per_pair.size(); i++)
        colors.push_back(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));

    // Draws the matchings.
    cv::Point2f q_l_kpt, cand_l_kpt;
    int color_idx, keyframe_position;
    int cand_cluster_id, cand_keyframe_id;
    for (unsigned int i = 0; i < f_q_l_kpts.size(); i++)
    {
        // Gets the ith keypoints.
        q_l_kpt = f_q_l_kpts[i].pt;
        cand_l_kpt = f_cand_l_kpts[i].pt;

        // Gets the ith candidate cluster ids.
        cand_cluster_id = cand_cluster_ids[i];

        // Gets the candidate keyframe id due to the relation between clusters and keyframes.
        cand_keyframe_id = -1;
        for (unsigned int j = 0; j < cand_cluster_keyframe_relation.size(); j++)
        {
            if (cand_cluster_keyframe_relation[j].first == cand_cluster_id)
                cand_keyframe_id = cand_cluster_keyframe_relation[j].second;
        }

        // Gets the index of the candidate keyframe within the variable cand_keyframe_vector_img.
        keyframe_position = -1;
        for (unsigned int j = 0; j < cand_keyframe_ids.size(); j++)
        {
            if (cand_keyframe_id == cand_keyframe_ids[j])
            {
                keyframe_position = j;
                break;
            }
        }

        // Draws the ith matching.
        if (keyframe_position >= 0)
        {
            // Decides the color. Weak edges (less than 5 inliers) are not inserted into the graph
            // and are therefore not drawn nor assigned a colour.
            color_idx = -1;
            for (unsigned int j = 0; j < definitive_inliers_per_pair.size(); j++)
            {
                if (definitive_cluster_pairs[j].second == cand_cluster_id)
                {
                    color_idx = j;
                    break;
                }
            }

            // Now it really draws.
            if (color_idx >= 0)
            {
                // Adds x offset to query left point.
                q_l_kpt.x += q_x_offset;
                q_l_kpt.y += q_y_offset;

                // Adds x and y offsets to candidate left point.
                cand_l_kpt.y += q_written_spaced_img.rows;
                cand_l_kpt.x += keyframe_position * written_img.cols;

                cv::circle(o_img, q_l_kpt, 2, colors[color_idx], -1);
                cv::circle(o_img, cand_l_kpt, 2, colors[color_idx], -1);
                cv::line(o_img, q_l_kpt, cand_l_kpt, colors[color_idx], 1, cv::LINE_AA, 0);
            }
        }
        else
        {
            ROS_ERROR_STREAM("[drawImgLoopClosure:] Negative keyframe position!");
        }
    }

    return o_img;
}


nav_msgs::Odometry Publisher::buildPoseMsg(const ros::Time& c_stamp,
                                           const tf::Transform& c_pose,
                                           const std::string& reference_frame_id)
{
    nav_msgs::Odometry msg;

    // Set header
    msg.header.stamp = c_stamp;
    msg.header.frame_id = reference_frame_id;

    // Set pose
    tf::poseTFToMsg(c_pose, msg.pose.pose);

    return msg;
}


void Publisher::publishImg(const cv::Mat& img,
                           const ros::Time& stamp,
                           const ros::Publisher& publisher)
{
    cv_bridge::CvImage ros_image;
    ros_image.image = img;
    ros_image.header.stamp = stamp;
    ros_image.encoding = "bgr8";
    publisher.publish(ros_image.toImageMsg());
}