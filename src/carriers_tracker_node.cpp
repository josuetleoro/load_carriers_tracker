#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include "load_carriers_tracker/carriers_tracker.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "carrier_tracker_node");
  CarriersTracker carrier_tracker;

  ros::spin();

  return 0;
}
