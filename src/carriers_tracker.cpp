#include "load_carriers_tracker/carriers_tracker.h"

CarriersTracker::CarriersTracker() : nh_priv_("~")
{
  nh_priv_.param<double>("match_th", match_th_, 0.2);
  nh_priv_.param<double>("estimation_variance", kalman_p_, 5.0);
  nh_priv_.param<double>("process_nosie", kalman_q_, 2.0);
  nh_priv_.param<double>("meas_noise", kalman_r_, 5.0);

  detection_sub_ = nh_priv_.subscribe<geometry_msgs::PoseArray>("/transformed_detections", 1, &CarriersTracker::detectionCb, this);
  filtered_detection_pub_ = nh_priv_.advertise<geometry_msgs::PoseArray>("/filtered_detection", 1);
}

void CarriersTracker::detectionCb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  for (const auto &pose : msg->poses)
  {
    // Find a match
    int match = matchCarrier(pose);
    if (match != -1)
    {
      carriers_[match].update(pose);
      continue;
    }
    else
    {
      carriers_.push_back(ObjectTracker(pose));
    }
  }

  // Copy objects pose to PoseArray
  geometry_msgs::PoseArray filtered_detection;
  filtered_detection.header.stamp = ros::Time::now();
  filtered_detection.header.frame_id = "map";
  for (auto carrier : carriers_)
  {
    filtered_detection.poses.push_back(carrier.getPose());
  }

  filtered_detection_pub_.publish(filtered_detection);
}

int CarriersTracker::matchCarrier(const geometry_msgs::Pose &detection)
{
  for (size_t i = 0; i < carriers_.size(); i++)
  {
    double distance = sqrt(pow(carriers_[i].getX() - detection.position.x, 2) + pow(carriers_[i].getY() - detection.position.y, 2));
    if (distance < match_th_)
    {
      return i;
    }
  }
  // No match found
  return -1;
}