#include "load_carriers_tracker/carriers_tracker.h"

CarriersTracker::CarriersTracker() : nh_priv_("~")
{
  nh_priv_.param<double>("match_th", match_th_, 0.2);
  nh_priv_.param<double>("pos_estimation_variance", pos_kalman_p_, 0.2);
  nh_priv_.param<double>("pos_process_noise_variance", pos_kalman_q_, 0.5);
  nh_priv_.param<double>("pos_meas_noise", pos_kalman_r_, 20.0);
  nh_priv_.param<double>("ori_estimation_variance", ori_kalman_p_, 1.0);
  nh_priv_.param<double>("ori_process_noise_variance", ori_kalman_q_, 20.0);
  nh_priv_.param<double>("ori_meas_noise", ori_kalman_r_, 50.0);
  nh_priv_.param<int>("max_cycles_without_detection", max_cycles_without_detection_, 125); // 5 seconds assuming 25Hz rate of detections

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
      cycles_without_detection_[match] = 0;
      continue;
    }
    else
    {
      carriers_.push_back(ObjectTracker(pose, pos_kalman_p_, pos_kalman_q_, pos_kalman_r_, ori_kalman_p_, ori_kalman_q_, ori_kalman_r_));
      cycles_without_detection_.push_back(0);
    }
  }

  // Remove carriers that had not been detected for a while
  for (size_t i = 0; i < carriers_.size(); i++)
  {
    if (cycles_without_detection_[i] > max_cycles_without_detection_)
    {
      carriers_.erase(carriers_.begin() + i);
      cycles_without_detection_.erase(cycles_without_detection_.begin() + i);
    }
    else
    {
      cycles_without_detection_[i]++;
    }
  }

  // Copy objects pose to PoseArray for visualization
  geometry_msgs::PoseArray filtered_detection;
  filtered_detection.header.stamp = ros::Time::now();
  filtered_detection.header.frame_id = "map";
  for (auto carrier : carriers_)
  {
    filtered_detection.poses.push_back(carrier.getPoseWithCov().pose);
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