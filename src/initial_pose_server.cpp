#include <ros/ros.h>

#include "slamon/GetTransform.h"


double tx_, ty_, tz_, qx_, qy_, qz_, qw_;

bool getPoseCallback(slamon::GetTransform::Request  &req,
                     slamon::GetTransform::Response &res)
{
    ROS_WARN_STREAM("Hello there");
    res.tf.position.x = tx_;
    res.tf.position.y = ty_;
    res.tf.position.z = tz_;
    res.tf.orientation.x = qx_;
    res.tf.orientation.y = qy_;
    res.tf.orientation.z = qz_;
    res.tf.orientation.w = qw_;
    
    return true;
}


void readParams()
{
    // Getting parameters from parameter server.
    ros::NodeHandle nhp("~");
    nhp.param("tx", tx_, 0.0);
    nhp.param("ty", ty_, 0.0);
    nhp.param("tz", tz_, 0.0);
    nhp.param("qx", qx_, 0.0);
    nhp.param("qy", qy_, 0.0);
    nhp.param("qz", qz_, 0.0);
    nhp.param("qw", qw_, 1.0);

    ROS_INFO_STREAM("[initial_pose_server:]" << std::endl <<
                    "tx = " << tx_ << std::endl << 
                    "ty = " << ty_ << std::endl << 
                    "tz = " << tz_ << std::endl << 
                    "qx = " << qx_ << std::endl << 
                    "qy = " << qy_ << std::endl << 
                    "qz = " << qz_ << std::endl << 
                    "qw = " << qw_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_server");
    ros::start();

    ros::NodeHandle nh;
    ros::ServiceServer initial_pose_server_ = nh.advertiseService("get_pose", getPoseCallback);

    readParams();

    ROS_INFO_STREAM("Ready to send punctual pose");
    ros::spin();

    return 0;
}