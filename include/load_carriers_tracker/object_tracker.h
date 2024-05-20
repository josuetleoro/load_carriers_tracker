#ifndef _OBJECT_TRACKER_
#define _OBJECT_TRACKER_

#include <unique_id/unique_id.h>
#include <boost/uuid/uuid.hpp>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <angles/angles.h>

#include "load_carriers_tracker/kalman_filter.hpp"

class ObjectTracker
{
public:
  ObjectTracker(const geometry_msgs::Pose &pose, double pos_init_estim_var, double pos_proc_var, double pos_meas_noise,
                double ori_init_estim_var, double ori_proc_var, double ori_meas_noise);
  void update(const geometry_msgs::Pose &pose);
  std::string getId() { return id_; }
  const double getX() { return x_; }
  const double getY() { return y_; }
  const double getTheta() { return theta_; }
  const geometry_msgs::PoseWithCovariance getPoseWithCov() { return pose_; }

private:
  std::string generateUUID();

  std::string id_;
  double x_, y_, theta_;
  double x_var_, y_var_, theta_var_;
  geometry_msgs::PoseWithCovariance pose_;
  Filter::KalmanFilter filter_x_, filter_y_, filter_theta_;
};

#endif