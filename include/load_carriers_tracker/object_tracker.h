#ifndef _OBJECT_TRACKER_
#define _OBJECT_TRACKER_

#include <tf/transform_datatypes.h>
#include "geometry_msgs/Pose.h"

#include "load_carriers_tracker/kalman_filter.hpp"

class ObjectTracker
{
public:
  ObjectTracker(const geometry_msgs::Pose &pose);
  void update(const geometry_msgs::Pose &pose);
  double getId() { return id_; }
  const double getX() { return x_; }
  const double getY() { return y_; }
  const double getTheta() { return theta_; }
  const geometry_msgs::Pose getPose() { return pose_; }

private:
  int id_;
  double x_, y_, theta_;
  double x_var_, y_var_, theta_var_;
  geometry_msgs::Pose pose_;
  Filter::KalmanFilter filter_x_, filter_y_, filter_theta_;
};

#endif