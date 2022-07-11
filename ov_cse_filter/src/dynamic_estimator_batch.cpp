#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "dynamic_estimator.h"
#include "tic_toc.h"
#include <random>

DynamicEstimator estimator;
void single_run(double position_sigma);

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "dynamic_estimator");
  ros::NodeHandle nh("~");
  // estimator.onInit(nh);

  estimator.useMethod_ = 1;
  double position_sigma = 0.1;
  single_run(position_sigma);

  cout << "end!" << endl;
  ros::spin();

  return 0;
}

void single_run(double position_sigma)
{
  estimator.initialized_ = false;
  estimator.Init();
  estimator.outFile_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

  std::random_device device_random_;
  std::default_random_engine generator_(device_random_());
  std::normal_distribution<> distribution_rx_(0.0, position_sigma);
  std::normal_distribution<> distribution_ry_(0.0, position_sigma);
  std::normal_distribution<> distribution_rz_(0.0, position_sigma);

  // Load rosbag here, and find messages we can play
  std::string bag_folder = "/home/junlin/CSE/";
  // std::string path_to_bag = bag_folder + "data.bag";
  std::string path_to_bag = bag_folder + "data_optitrack.bag";
  // std::string path_to_bag = bag_folder + "data_euroc.bag";
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start = 0;
  double bag_durr = -1;
  ROS_INFO("bag start: %.1f", bag_start);
  ROS_INFO("bag duration: %.1f", bag_durr);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  ROS_INFO("time start = %.6f", time_init.toSec());
  ROS_INFO("time end   = %.6f", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return;
  }

  std::string pose_topic_name = "/vio_pose";
  std::string relative_position_topic_name = "/relative_position";
  std::string imu_topic_name = "/imu";
  // Open our iterators
  auto view_pose = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(pose_topic_name), time_init, time_finish);
  auto view_pose_iter = view_pose->begin();
  auto view_relative_pos = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(relative_position_topic_name), time_init, time_finish);
  auto view_relative_pos_iter = view_relative_pos->begin();
  auto view_imu = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(imu_topic_name), time_init, time_finish);
  auto view_imu_iter = view_imu->begin();

  // Record the current measurement timestamps
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_pose_current;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_pose_next;
  geometry_msgs::PointStamped::ConstPtr msg_relative_pos_current;
  geometry_msgs::PointStamped::ConstPtr msg_relative_pos_next;
  sensor_msgs::Imu::ConstPtr msg_imu_current;
  sensor_msgs::Imu::ConstPtr msg_imu_next;

  msg_pose_current = view_pose_iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
  view_pose_iter++;
  msg_pose_next = view_pose_iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
  msg_relative_pos_current = view_relative_pos_iter->instantiate<geometry_msgs::PointStamped>();
  view_relative_pos_iter++;
  msg_relative_pos_next = view_relative_pos_iter->instantiate<geometry_msgs::PointStamped>();
  msg_imu_current = view_imu_iter->instantiate<sensor_msgs::Imu>();
  view_imu_iter++;
  msg_imu_next = view_imu_iter->instantiate<sensor_msgs::Imu>();

  double last_t = -1;
  double t = -1;
  double update_time = 0.0;
  int count_meas = 0;
  while (ros::ok()) {
    if (t > 0)
    {
      last_t = t;
    }

    bool should_process_pose = false;
    bool should_process_imu = false;
    double time_pose = msg_pose_current->header.stamp.toSec();
    double time_imu = msg_imu_current->header.stamp.toSec();

    if (time_pose <= time_imu) {
      should_process_pose = true;
    }
    else {
      should_process_imu = true;
    }

    if (should_process_pose) {
      static int cnt = -1;
      cnt++;
      if (cnt % 4 != 0)
      {
        // Move forward in time
        msg_pose_current = msg_pose_next;
        view_pose_iter++;
        if (view_pose_iter != view_pose->end()) {
          msg_pose_next = view_pose_iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        }
        else
        {
          break;
        }
        msg_relative_pos_current = msg_relative_pos_next;
        view_relative_pos_iter++;
        if (view_relative_pos_iter != view_relative_pos->end()) {
          msg_relative_pos_next = view_relative_pos_iter->instantiate<geometry_msgs::PointStamped>();
        }
        else
        {
          break;
        }
        continue;
      }

      bool have_found_pair = false;
      while (!have_found_pair && view_pose_iter != view_pose->end() &&
             view_relative_pos_iter != view_relative_pos->end()) {

        // timestamps
        double time0 = msg_pose_current->header.stamp.toSec();
        double time1 = msg_relative_pos_current->header.stamp.toSec();
        double time0_next = msg_pose_next->header.stamp.toSec();
        double time1_next = msg_relative_pos_next->header.stamp.toSec();

        if (std::abs(time1 - time0) < std::abs(time1_next - time0) && std::abs(time0 - time1) < std::abs(time0_next - time1)) {
          have_found_pair = true;
        } else if (std::abs(time1 - time0) >= std::abs(time1_next - time0)) {
          msg_relative_pos_current = msg_relative_pos_next;
          view_relative_pos_iter++;
          if (view_relative_pos_iter != view_relative_pos->end()) {
            msg_relative_pos_next = view_relative_pos_iter->instantiate<geometry_msgs::PointStamped>();
          }
        } else {
          msg_pose_current = msg_pose_next;
          view_pose_iter++;
          if (view_pose_iter != view_pose->end()) {
            msg_pose_next = view_pose_iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
          }
        }
      }

      // Break out if we have ended
      if (view_pose_iter == view_pose->end() || view_relative_pos_iter == view_relative_pos->end()) {
        break;
      }

      // printf("t1 %f t2 %f\n", msg_pose_current->header.stamp.toSec(), msg_relative_pos_current->header.stamp.toSec());
      // getchar();

      Eigen::Matrix<double, 4, 1> q_I1_W;
      Eigen::Matrix<double, 3, 1> t_I1_W;
      q_I1_W(0, 0) = msg_pose_current->pose.pose.orientation.x; // quat
      q_I1_W(1, 0) = msg_pose_current->pose.pose.orientation.y;
      q_I1_W(2, 0) = msg_pose_current->pose.pose.orientation.z;
      q_I1_W(3, 0) = msg_pose_current->pose.pose.orientation.w;
      t_I1_W(0, 0) = msg_pose_current->pose.pose.position.x; // pos
      t_I1_W(1, 0) = msg_pose_current->pose.pose.position.y;
      t_I1_W(2, 0) = msg_pose_current->pose.pose.position.z;
      Eigen::Matrix<double, 3, 3> R_I1_W = quat_2_Rot(q_I1_W);

      Eigen::Matrix<double, 3, 1> t_I2_I1;
      // t_I2_I1(0, 0) = msg_relative_pos_current->point.x;
      // t_I2_I1(1, 0) = msg_relative_pos_current->point.y;
      // t_I2_I1(2, 0) = msg_relative_pos_current->point.z;
      t_I2_I1(0, 0) = msg_relative_pos_current->point.x + distribution_rx_(generator_);
      t_I2_I1(1, 0) = msg_relative_pos_current->point.y + distribution_ry_(generator_);
      t_I2_I1(2, 0) = msg_relative_pos_current->point.z + distribution_rz_(generator_);

      Eigen::Matrix<double, 3, 1> t_I2_W = t_I1_W + R_I1_W.transpose() * t_I2_I1;

      Eigen::Matrix<double, 6, 6> covariance_vio_pose = Eigen::Matrix<double, 6, 6>::Zero();
      for (int r = 0; r < 6; r++) {
          for (int c = 0; c < 6; c++) {
              covariance_vio_pose(r, c) = msg_pose_current->pose.covariance[6 * r + c];
          }
      }

      Eigen::Matrix<double, 3, 6> Jacob = Eigen::Matrix<double, 3, 6>::Zero();
      Jacob.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
      Jacob.block(0, 3, 3, 3) = -R_I1_W.transpose() * skewSymmetric_d(t_I2_I1);

      Eigen::Matrix<double, 3, 3> covariance_position = Eigen::Matrix<double, 3, 3>::Zero();
      // covariance_position = Jacob * covariance_vio_pose * Jacob.transpose();
      covariance_position = Jacob * covariance_vio_pose * Jacob.transpose() + (position_sigma * position_sigma) * Eigen::Matrix<double, 3, 3>::Identity();

      MeasPose cur_pos;
      cur_pos.r = t_I2_W.cast<flt>();
      cur_pos.cov = covariance_position.cast<flt>();
      TicToc t_solve;
      estimator.r_callback(cur_pos);
      update_time += t_solve.toc();
      count_meas++;

      // Move forward in time
      msg_pose_current = msg_pose_next;
      view_pose_iter++;
      if (view_pose_iter != view_pose->end()) {
        msg_pose_next = view_pose_iter->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      }
      else
      {
        break;
      }
      msg_relative_pos_current = msg_relative_pos_next;
      view_relative_pos_iter++;
      if (view_relative_pos_iter != view_relative_pos->end()) {
        msg_relative_pos_next = view_relative_pos_iter->instantiate<geometry_msgs::PointStamped>();
      }
      else
      {
        break;
      }
    }

    if (should_process_imu) {
      t = msg_imu_current->header.stamp.toSec();
      printf("imu t %f\n", t);
      Input cur_input;
      cur_input.t = msg_imu_current->header.stamp;
      cur_input.a(0) = msg_imu_current->linear_acceleration.x;
      cur_input.a(1) = msg_imu_current->linear_acceleration.y;
      cur_input.a(2) = msg_imu_current->linear_acceleration.z;
      cur_input.w(0) = msg_imu_current->angular_velocity.x;
      cur_input.w(1) = msg_imu_current->angular_velocity.y;
      cur_input.w(2) = msg_imu_current->angular_velocity.z;
      
      estimator.input_callback(cur_input);
      // move forward in time
      msg_imu_current = msg_imu_next;
      view_imu_iter++;
      if (view_imu_iter != view_imu->end()) {
         msg_imu_next = view_imu_iter->instantiate<sensor_msgs::Imu>();
      }
      else
      {
        break;
      }
    }

    if (last_t > 0)
    {
      // printf("dt %f\n", t - last_t);
      if ((t - last_t) < 0) {
        ROS_ERROR("dt < 0.  Exiting.");
        ros::shutdown();
        return;
      }
    }

  }

  update_time = update_time/count_meas;

  return;
}
