#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <fstream>
#include <queue>

std::ofstream outFile_odo_pose;

void odometry_callback(geometry_msgs::PoseStamped odo_msg)
{
    double t = odo_msg.header.stamp.toSec();
    double x = odo_msg.pose.position.x;
    double y = odo_msg.pose.position.y;
    double z = odo_msg.pose.position.z;

    outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "sub_pose");
    ros::NodeHandle n;

    outFile_odo_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

    ros::Subscriber odometry_subscriber = n.subscribe("/poseimu", 1000, odometry_callback);

    ros::spin();
  
    return 0;
}