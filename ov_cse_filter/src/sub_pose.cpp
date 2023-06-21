#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <fstream>
#include <queue>

std::deque<nav_msgs::Odometry> gt_buffer;
std::deque<nav_msgs::Odometry> gt2_buffer;
std::deque<geometry_msgs::PoseStamped> odo_buffer;
std::deque<geometry_msgs::PointStamped> pos_buffer;

std::ofstream outFile_gt_pose;
std::ofstream outFile_odo_pose;
std::ofstream outFile_pos_pose;

void gt_callback(nav_msgs::Odometry gt_msg)
{
    double t = gt_msg.header.stamp.toSec();
    double x = gt_msg.pose.pose.position.x;
    double y = gt_msg.pose.pose.position.y;
    double z = gt_msg.pose.pose.position.z;

    outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

    gt_buffer.push_back(gt_msg);
    gt2_buffer.push_back(gt_msg);
}

void odometry_callback(geometry_msgs::PoseStamped odo_msg)
{
    double t = odo_msg.header.stamp.toSec();
    double x = odo_msg.pose.position.x;
    double y = odo_msg.pose.position.y;
    double z = odo_msg.pose.position.z;

    outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

    odo_buffer.push_back(odo_msg);
}

void position_callback(geometry_msgs::PointStamped pos_msg)
{
    double t = pos_msg.header.stamp.toSec();
    double x = pos_msg.point.x;
    double y = pos_msg.point.y;
    double z = pos_msg.point.z;

    outFile_pos_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

    pos_buffer.push_back(pos_msg);
}

void Get_Odo_Err();
void Get_Pos_Err();

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "sub_pose");
    ros::NodeHandle n;

    outFile_gt_pose.open("/home/junlin/GNSS/eval/gt.txt");
    outFile_odo_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");
    outFile_pos_pose.open("/home/junlin/GNSS/eval/stamped_pos_estimate.txt");

    ros::Subscriber gt_subscriber = n.subscribe("/uav_1/ground_truth/state", 1000, gt_callback);
    ros::Subscriber odometry_subscriber = n.subscribe("/poseimu", 1000, odometry_callback);
    ros::Subscriber pos_subscriber = n.subscribe("/detect_track2D_and_locate/detection_point_stamped", 1000, position_callback);

    while (ros::ok())
    {
        ros::spinOnce();
        Get_Odo_Err();
        Get_Pos_Err();
    }

    ros::spin();
  
    return 0;
}

void Get_Odo_Err()
{
    if (!gt_buffer.empty() && !odo_buffer.empty())
    {
        double time1 = gt_buffer.front().header.stamp.toSec();
        double time2 = odo_buffer.front().header.stamp.toSec();
        // printf("%f %f\n", time1, time2);

        if(time1 < time2 - 0.03)
        {
            gt_buffer.pop_front();
            // printf("throw gt\n");
        }
        else if(time1 > time2 + 0.03)
        {
            odo_buffer.pop_front();
            // printf("throw odo\n");
        }
        else
        {
            double time = time1;
            
            auto gt_msg = gt_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I1_W;
            t_I1_W(0) = gt_msg.pose.pose.position.x;
            t_I1_W(1) = gt_msg.pose.pose.position.y;
            t_I1_W(2) = gt_msg.pose.pose.position.z;

            auto odo_msg = odo_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I2_W;
            t_I2_W(0) = odo_msg.pose.position.x;
            t_I2_W(1) = odo_msg.pose.position.y;
            t_I2_W(2) = odo_msg.pose.position.z;

            Eigen::Matrix<double, 3, 1> t_I1_I2 = t_I1_W - t_I2_W;
            printf("time %f odo dis err %f\n", time, t_I1_I2.norm());

            gt_buffer.pop_front();
            odo_buffer.pop_front();
        }
    }
}

void Get_Pos_Err()
{
    if (!gt2_buffer.empty() && !pos_buffer.empty())
    {
        double time1 = gt2_buffer.front().header.stamp.toSec();
        double time2 = pos_buffer.front().header.stamp.toSec();
        // printf("%f %f\n", time1, time2);

        if(time1 < time2 - 0.03)
        {
            gt2_buffer.pop_front();
            // printf("throw gt\n");
        }
        else if(time1 > time2 + 0.03)
        {
            pos_buffer.pop_front();
            // printf("throw pos\n");
        }
        else
        {
            double time = time1;
            
            auto gt_msg = gt2_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I1_W;
            t_I1_W(0) = gt_msg.pose.pose.position.x;
            t_I1_W(1) = gt_msg.pose.pose.position.y;
            t_I1_W(2) = gt_msg.pose.pose.position.z;

            auto pos_msg = pos_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I2_W;
            t_I2_W(0) = pos_msg.point.x;
            t_I2_W(1) = pos_msg.point.y;
            t_I2_W(2) = pos_msg.point.z;

            Eigen::Matrix<double, 3, 1> t_I1_I2 = t_I1_W - t_I2_W;
            printf("\033[31m" "time %f pos dis err %f\n" "\033[0m", time, t_I1_I2.norm());

            gt2_buffer.pop_front();
            pos_buffer.pop_front();
        }
    }
}