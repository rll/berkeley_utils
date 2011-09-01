#include <Eigen/Core>

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PointStamped.h"

#include "opencv2/core/core.hpp"

inline cv::Mat eigen_to_cv(const Eigen::Vector2d& eigen){

  cv::Mat mat(1,2,CV_64F);
  double* cv_ptr = mat.ptr<double>(0);

  cv_ptr[0] = eigen[0];
  cv_ptr[1] = eigen[1];

  return mat;

}

inline cv::Mat eigen_to_cv(const Eigen::Vector3d& eigen){

  cv::Mat mat(1,3,CV_64F);
  double* cv_ptr = mat.ptr<double>(0);

  cv_ptr[0] = eigen[0];
  cv_ptr[1] = eigen[1];
  cv_ptr[2] = eigen[2];

  return mat;

}

inline cv::Point3d eigen_to_cv_pt(const Eigen::Vector3d& eigen){
  return cv::Point3d(eigen[0], eigen[1], eigen[2]);
}

inline tf::Point eigen_to_tf(const Eigen::Vector3d& eigen){
  return tf::Point(eigen[0], eigen[1], eigen[2]);
}

inline tf::Point cv_pt3_to_tf(const cv::Point3d& cv){
  return tf::Point(cv.x, cv.y, cv.z);
}

inline Eigen::Vector3d tf_to_eigen(const tf::Point& tfpt){
  return Eigen::Vector3d(tfpt.x(), tfpt.y(), tfpt.z());
}

inline cv::Point3d tf_to_cv_pt(const tf::Point& tfpt){
  return cv::Point3d(tfpt.x(), tfpt.y(), tfpt.z());
}

inline tf::Point pt_msg_to_tf(const geometry_msgs::PointStamped& msg){
    return tf::Point(msg.point.x, msg.point.y, msg.point.z);
}

inline tf::Point pt_msg_to_tf(const geometry_msgs::Point& msg){
    return tf::Point(msg.x, msg.y, msg.z);
}

inline void set_ps_from_tf(geometry_msgs::PointStamped& msg, const tf::Point& tfpt)
{

    ros::Time now = ros::Time::now();

    msg.header.stamp = now;
    msg.header.stamp = now;

    msg.point.x = tfpt.x();
    msg.point.y = tfpt.y();
    msg.point.z = tfpt.z();
}
