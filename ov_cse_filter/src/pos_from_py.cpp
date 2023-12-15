#include <ros/ros.h>
#include <ov_cse_filter/PointId.h>
#include <ov_cse_filter/PointIdArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <fstream>

using namespace std;

geometry_msgs::PoseStamped gt_msg_uav_b;
bool gt_uav_b_received = false;
bool gt_uav_a_received = false;

Eigen::Matrix3d R_CtoI;
Eigen::Vector3d p_CinI;
Eigen::Matrix<double, 3, 3> R_A_W;
Eigen::Matrix<double, 3, 1> t_A_W;

std::ofstream outFile_gt_pose;
std::ofstream outFile_cam_pose;

std::vector<geometry_msgs::PoseStamped> poses_b;
ros::Publisher pub_path;

void relative_pos_callback(ov_cse_filter::PointIdArray tag_msg)
{
  if (gt_uav_a_received && gt_uav_b_received)
  {
    double t = tag_msg.header.stamp.toSec();
    for(unsigned int i = 0; i < tag_msg.points.size(); i++)
    {
      if (tag_msg.points.at(i).id == 0)
      {
        Eigen::Matrix<double, 3, 1> t_rel;
        t_rel << tag_msg.points.at(i).position.x, tag_msg.points.at(i).position.y, tag_msg.points.at(i).position.z;
        Eigen::Matrix<double, 3, 1> t_B_A = p_CinI + R_CtoI * t_rel;
        Eigen::Matrix<double, 3, 1> t_B_W = R_A_W * t_B_A + t_A_W;
        Eigen::Matrix<double, 3, 1> t_B_W_gt;
        t_B_W_gt << gt_msg_uav_b.pose.position.x, gt_msg_uav_b.pose.position.y, gt_msg_uav_b.pose.position.z;
        outFile_cam_pose << std::fixed << std::setprecision(6) << t << " " << t_B_W(0) << " " << t_B_W(1) << " " << t_B_W(2) << " " << std::endl;
        outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << t_B_W_gt(0) << " " << t_B_W_gt(1) << " " << t_B_W_gt(2) << " " << std::endl;

        // Append to our pose vector
        geometry_msgs::PoseStamped posetemp;
        posetemp.header.stamp = ros::Time().fromSec(t);
        posetemp.header.frame_id = "map";
        posetemp.pose.position.x = t_B_W(0);
        posetemp.pose.position.y = t_B_W(1);
        posetemp.pose.position.z = t_B_W(2);

        poses_b.push_back(posetemp);

        // Create our path
        // NOTE: We downsample the number of poses as needed to prevent rviz crashes
        // NOTE: https://github.com/ros-visualization/rviz/issues/1107
        nav_msgs::Path arr_b;
        arr_b.header.stamp = posetemp.header.stamp;
        arr_b.header.frame_id = "map";
        for (size_t i = 0; i < poses_b.size(); i += std::floor((double)poses_b.size() / 16384.0) + 1) {
            arr_b.poses.push_back(poses_b.at(i));
        }
        pub_path.publish(arr_b);
      }
    }
  }
}

void gt_uav_b_callback(geometry_msgs::PoseStamped gt_msg)
{
  gt_uav_b_received = true;
  gt_msg_uav_b = gt_msg;
}

void gt_uav_a_callback(geometry_msgs::PoseStamped gt_msg)
{
  gt_uav_a_received = true;
  Eigen::Quaterniond q_A_W;        
  q_A_W.x() = gt_msg.pose.orientation.x; // quat
  q_A_W.y() = gt_msg.pose.orientation.y;
  q_A_W.z() = gt_msg.pose.orientation.z;
  q_A_W.w() = gt_msg.pose.orientation.w;
  R_A_W = q_A_W.toRotationMatrix();

  t_A_W(0, 0) = gt_msg.pose.position.x; // pos
  t_A_W(1, 0) = gt_msg.pose.position.y;
  t_A_W(2, 0) = gt_msg.pose.position.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_from_py");
  ros::NodeHandle nh("~");

  Eigen::Matrix3d R_CtoI_test;
  R_CtoI_test << 0.003, -0.411,  0.912,
                 -1   ,   0.01,  0.008,
                -0.013, -0.912, -0.411;
  R_CtoI = R_CtoI_test;
  Eigen::Vector3d p_CinI_test; 
  p_CinI_test << 0.139, -0.031, -0.234;
  p_CinI = p_CinI_test;

  outFile_gt_pose.open("/home/junlin/GNSS/eval/gt.txt");
  outFile_cam_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

  ros::Subscriber relative_pos_sub = nh.subscribe("/vision_stack/drones", 1, relative_pos_callback);
  ros::Subscriber gt_uav_a_sub = nh.subscribe("/vrpn_client_node/uav_a/pose", 1, gt_uav_a_callback);
  ros::Subscriber gt_uav_b_sub = nh.subscribe("/vrpn_client_node/uav_b/pose", 1, gt_uav_b_callback);

  pub_path = nh.advertise<nav_msgs::Path>("/path", 1);

  ros::spin();
}