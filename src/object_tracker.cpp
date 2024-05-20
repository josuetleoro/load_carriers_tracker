#include "load_carriers_tracker/object_tracker.h"

ObjectTracker::ObjectTracker(const geometry_msgs::Pose &pose, double pos_init_estim_var, double pos_proc_var, double pos_meas_noise,
                             double ori_init_estim_var, double ori_proc_var, double ori_meas_noise)
{
  // Generate UUID
  id_ = generateUUID();
  pose_.pose = pose;
  pose_.covariance[0] = pos_init_estim_var;
  pose_.covariance[7] = pos_init_estim_var;
  pose_.covariance[35] = ori_init_estim_var;
  x_ = pose.position.x;
  y_ = pose.position.y;
  theta_ = tf::getYaw(pose.orientation);
  filter_x_ = Filter::KalmanFilter(pos_init_estim_var, pos_proc_var, pos_meas_noise, x_);
  filter_y_ = Filter::KalmanFilter(ori_init_estim_var, ori_proc_var, pos_meas_noise, y_);
  filter_theta_ = Filter::KalmanFilter(ori_init_estim_var, ori_proc_var, ori_meas_noise, theta_);
}

void ObjectTracker::update(const geometry_msgs::Pose &pose)
{
  // If the difference in theta is greater than 90 degrees, skip the update
  double theta = tf::getYaw(pose.orientation);
  if (fabs(angles::shortest_angular_distance(theta_, theta)) > M_PI / 2)
  {
    return;
  }
  double x = pose.position.x;
  double y = pose.position.y;

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
  pose_.pose.position.x = x_;
  pose_.pose.position.y = y_;
  pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
  pose_.covariance[0] = x_var_;
  pose_.covariance[7] = y_var_;
  pose_.covariance[35] = theta_var_;
}

std::string ObjectTracker::generateUUID()
{
  auto uuid = unique_id::fromRandom();
  id_ = unique_id::toHexString(uuid);
  return id_;
}
