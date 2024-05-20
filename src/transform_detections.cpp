#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using nlohmann::json;
using namespace std;

typedef geometry_msgs::Pose2D Pose2D;
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::PoseStamped PoseStamped;

ros::Publisher detection_pub;
ros::Publisher robot_pose_pub;
tf2::Transform robot_pose_tf;

void detection_cb(const std_msgs::String::ConstPtr &msg)
{
  const json df = json::parse(msg->data);

  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";
  std::vector<Pose2D> poses;
  for (const auto &detection : df["detections"]["poses"])
  {
    Pose pose;
    pose.position.x = detection["x"].get<float>();
    pose.position.y = detection["y"].get<float>();
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(detection["theta"].get<float>());

    // Convert pose to robot frame using robot_pose_tf
    tf2::Transform pose_tf;
    tf2::fromMsg(pose, pose_tf);
    pose_tf = robot_pose_tf * pose_tf;
    geometry_msgs::Pose pose_msg;
    tf2::toMsg(pose_tf, pose_msg);
    pose_array.poses.push_back(pose_msg);
  }
  detection_pub.publish(pose_array);
}

void robot_pose_cb(const std_msgs::String::ConstPtr &msg)
{
  const json df = json::parse(msg->data)["pose"];

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = "map";
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.pose.position.x = df.at("x").get<float>();
  robot_pose.pose.position.y = df.at("y").get<float>();
  robot_pose.pose.position.z = 0;
  robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(df.at("theta").get<float>());

  // Update robot pose transform
  tf2::Quaternion q;
  tf2::fromMsg(robot_pose.pose.orientation, q);
  robot_pose_tf.setOrigin(tf2::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, 0));
  robot_pose_tf.setRotation(q);

  robot_pose_pub.publish(robot_pose);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "transform_detections");
  ros::NodeHandle nh_;

  ros::Subscriber detection_sub = nh_.subscribe<std_msgs::String>("/detection", 5, detection_cb);
  ros::Subscriber robot_pose_sub = nh_.subscribe<std_msgs::String>("/robotPose", 5, robot_pose_cb);

  detection_pub = nh_.advertise<geometry_msgs::PoseArray>("/transformed_detections", 5);
  robot_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose_disp", 5);

  ros::spin();

  return 0;
}
