#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "dynamic_estimator.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "dynamic_estimator");
  ros::NodeHandle nh("~");

  DynamicEstimator estimator;
  estimator.onInit(nh);

  std::string topic_imu;
  nh.param<std::string>("topic_imu", topic_imu, "/uav_1/mavros/imu/data");
  std::string topic_pos;
  nh.param<std::string>("topic_pos", topic_pos, "/abs_position");
  std::string topic_gps_pos;
  nh.param<std::string>("topic_gps_pos", topic_gps_pos, "/uav_1/ground_truth/state");
  std::string topic_signal;
  nh.param<std::string>("topic_signal", topic_signal, "/disable_gps");

  std::string topic_pub_pose;
  nh.param<std::string>("topic_pub_pose", topic_pub_pose, "/poseimu");
  std::string topic_pub_path;
  nh.param<std::string>("topic_pub_path", topic_pub_path, "/pathimu");

  estimator.pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pub_pose, 10);
  estimator.pub_pathimu = nh.advertise<nav_msgs::Path>(topic_pub_path, 10);
  // estimator.outFile_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

  estimator.sub_imu_  = nh.subscribe(topic_imu,  10,&DynamicEstimator::input_callback,  &estimator);
  estimator.sub_pos_ = nh.subscribe(topic_pos, 10,&DynamicEstimator::r_callback, &estimator);
  estimator.sub_gps_pos_ = nh.subscribe(topic_gps_pos, 10,&DynamicEstimator::gps_r_callback, &estimator);
  estimator.sub_signal_ = nh.subscribe(topic_signal, 10,&DynamicEstimator::signal_callback, &estimator);

  ros::spin();

  return 0;
}
