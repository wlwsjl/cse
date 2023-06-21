#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <fstream>
#include <queue>

std::ofstream outFile_gt_pose;
std::ofstream outFile_odo_pose;

void gt_callback(nav_msgs::Odometry gt_msg)
{
    double t = gt_msg.header.stamp.toSec();
    double x = gt_msg.pose.pose.position.x;
    double y = gt_msg.pose.pose.position.y;
    double z = gt_msg.pose.pose.position.z;

    outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;
}

void odometry_callback(geometry_msgs::PoseStamped odo_msg)
{
    double t = odo_msg.header.stamp.toSec();
    double x = odo_msg.pose.position.x;
    double y = odo_msg.pose.position.y;
    double z = odo_msg.pose.position.z;

    outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;
}

void position_callback(geometry_msgs::PointStamped pos_msg)
{
    double t = pos_msg.header.stamp.toSec();
    double x = pos_msg.point.x;
    double y = pos_msg.point.y;
    double z = pos_msg.point.z;

    outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "sub_pose");
    ros::NodeHandle n;

    outFile_gt_pose.open("/home/junlin/GNSS/eval/gt.txt");
    outFile_odo_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

    ros::Subscriber gt_subscriber = n.subscribe("/uav_1/ground_truth/state", 1000, gt_callback);
    ros::Subscriber odometry_subscriber = n.subscribe("/poseimu", 1000, odometry_callback);
    // ros::Subscriber odometry_subscriber = n.subscribe("/detect_track2D_and_locate/detection_point_stamped", 1000, position_callback);

    ros::spin();
  
    return 0;
}