#ifndef _CARRIERS_TRACKER_
#define _CARRIERS_TRACKER_

#include <vector>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseArray.h>

#include "load_carriers_tracker/object_tracker.h"

class CarriersTracker
{
public:
  CarriersTracker();
  void detectionCb(const geometry_msgs::PoseArray::ConstPtr &msg);

private:
  int matchCarrier(const geometry_msgs::Pose &detection);

  std::vector<ObjectTracker> carriers_;
  std::vector<int> cycles_without_detection_;
  int max_cycles_without_detection_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber detection_sub_;
  ros::Publisher filtered_detection_pub_;

  double match_th_;
  double kalman_p_, kalman_q_, kalman_r_;
};

#endif