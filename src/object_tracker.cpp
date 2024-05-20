#include "load_carriers_tracker/object_tracker.h"

ObjectTracker::ObjectTracker(const geometry_msgs::Pose &pose)
{
  pose_ = pose;
  x_ = pose.position.x;
  y_ = pose.position.y;
  theta_ = tf::getYaw(pose.orientation);
  filter_x_ = Filter::KalmanFilter(0.2, 10.0, 100.0, x_);
  filter_y_ = Filter::KalmanFilter(0.2, 10.0, 100.0, y_);
  filter_theta_ = Filter::KalmanFilter(0.2, 10.0, 100.0, theta_);
}

void ObjectTracker::update(const geometry_msgs::Pose &pose)
{
  // If the difference in theta is greater than 90 degrees, skip the update
  if (fabs(tf::getYaw(pose.orientation) - theta_) > M_PI / 2)
  {
    return;
  }

  double x = pose.position.x;
  double y = pose.position.y;
  double theta = tf::getYaw(pose.orientation);
  filter_x_.update(x);
  filter_y_.update(y);
  filter_theta_.update(theta);
  x_ = filter_x_.getEstimation();
  y_ = filter_y_.getEstimation();
  theta_ = filter_theta_.getEstimation();
  x_var_ = filter_x_.getVariance();
  y_var_ = filter_y_.getVariance();
  theta_var_ = filter_theta_.getVariance();
  // Update pose
  pose_.position.x = x_;
  pose_.position.y = y_;
  pose_.orientation = tf::createQuaternionMsgFromYaw(theta_);
}
