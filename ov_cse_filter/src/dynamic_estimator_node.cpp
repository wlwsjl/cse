#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "dynamic_estimator.h"
#include <random>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "dynamic_estimator");
  ros::NodeHandle nh("~");

  DynamicEstimator estimator;
  estimator.onInit(nh);
  estimator.pub_pathimu = nh.advertise<nav_msgs::Path>("/pathimu", 10);
  estimator.outFile_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

  estimator.sub_imu_  = nh.subscribe("/uav_1/mavros/imu/data",  10,&DynamicEstimator::input_callback,  &estimator);
  estimator.sub_pos_ = nh.subscribe("/abs_position", 10,&DynamicEstimator::r_callback, &estimator);

  while (ros::ok())
  {
    ros::spinOnce();
    MeasureGroup Measures;
    if(estimator.sync_packages(Measures)) 
    {
      estimator.process_packages(Measures);
    }
  }

  ros::spin();

  return 0;
}
