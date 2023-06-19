#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <Eigen/Eigen>
#include <fstream>
#include <queue>

std::deque<nav_msgs::Odometry> gt1_buffer;
std::deque<nav_msgs::Odometry> gt2_buffer;

std::deque<geometry_msgs::PointStamped> rel_p_buffer;
std::deque<geometry_msgs::PoseStamped> odo2_p_buffer;
std::deque<geometry_msgs::PointStamped> abs_position_buffer;

std::ofstream outFile_odo_pose;
std::ofstream outFile_gt_pose;

void odometry1_callback(geometry_msgs::PoseStamped odo_msg)
{
    double t = odo_msg.header.stamp.toSec();
    double x = odo_msg.pose.position.x;
    double y = odo_msg.pose.position.y;
    double z = odo_msg.pose.position.z;

    // outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;
}

// void odometry2_callback(geometry_msgs::PoseStamped odo_msg)
// {
//     double t = odo_msg.header.stamp.toSec();
//     double x = odo_msg.pose.position.x;
//     double y = odo_msg.pose.position.y;
//     double z = odo_msg.pose.position.z;

//     // outFile_odo_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

//     odo2_p_buffer.push_back(odo_msg);
// }

void odometry2_callback(nav_msgs::Odometry gt_msg)
{
    geometry_msgs::PoseStamped odo_msg;
    odo_msg.header = gt_msg.header;
    odo_msg.pose.position.x = gt_msg.pose.pose.position.x;
    odo_msg.pose.position.y = gt_msg.pose.pose.position.y;
    odo_msg.pose.position.z = gt_msg.pose.pose.position.z;
    odo_msg.pose.orientation.x = gt_msg.pose.pose.orientation.x;
    odo_msg.pose.orientation.y = gt_msg.pose.pose.orientation.y;
    odo_msg.pose.orientation.z = gt_msg.pose.pose.orientation.z;
    odo_msg.pose.orientation.w = gt_msg.pose.pose.orientation.w;

    odo2_p_buffer.push_back(odo_msg);
}


void gt1_callback(nav_msgs::Odometry gt_msg)
{
    double t = gt_msg.header.stamp.toSec();
    double x = gt_msg.pose.pose.position.x;
    double y = gt_msg.pose.pose.position.y;
    double z = gt_msg.pose.pose.position.z;

    // outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

    gt1_buffer.push_back(gt_msg);
}

void gt2_callback(nav_msgs::Odometry gt_msg)
{
    double t = gt_msg.header.stamp.toSec();
    double x = gt_msg.pose.pose.position.x;
    double y = gt_msg.pose.pose.position.y;
    double z = gt_msg.pose.pose.position.z;

    // outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " " << std::endl;

    gt2_buffer.push_back(gt_msg);
}

void Get_Rel_Pos();
ros::Publisher abs_pos_pub;
void Get_Abs_Pos();
ros::Publisher signal_pub;

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "message_handler");
    ros::NodeHandle n;

    // outFile_odo_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");
    // outFile_gt_pose.open("/home/junlin/GNSS/eval/gt.txt");

    // ros::Subscriber odometry1_subscriber = n.subscribe("/uav_1/ual/pose", 1000, odometry1_callback);
    // ros::Subscriber odometry2_subscriber = n.subscribe("/uav_2/ual/pose", 1000, odometry2_callback);
    ros::Subscriber odometry2_subscriber = n.subscribe("/uav_2/ground_truth/state", 1000, odometry2_callback);
    ros::Subscriber gt1_subscriber = n.subscribe("/uav_1/ground_truth/state", 1000, gt1_callback);
    ros::Subscriber gt2_subscriber = n.subscribe("/uav_2/ground_truth/state", 1000, gt2_callback);

    abs_pos_pub = n.advertise<geometry_msgs::PointStamped>("/abs_position", 1000);
    signal_pub = n.advertise<std_msgs::Bool>("/disable_gps", 1000);

    while (ros::ok())
    {
        ros::spinOnce();
        Get_Rel_Pos();
        Get_Abs_Pos();
    }

    ros::spin();
  
    return 0;
}

void Get_Rel_Pos()
{
    if (!gt1_buffer.empty() && !gt2_buffer.empty())
    {
        double time1 = gt1_buffer.front().header.stamp.toSec();
        double time2 = gt2_buffer.front().header.stamp.toSec();
        // printf("%f %f\n", time1, time2);

        if(time1 < time2 - 0.03)
        {
            gt1_buffer.pop_front();
            // printf("throw gt1\n");
        }
        else if(time1 > time2 + 0.03)
        {
            gt2_buffer.pop_front();
            // printf("throw gt2\n");
        }
        else
        {
            double time = time1;
            // printf("rel pos %f\n", time);
            auto gt1_msg = gt1_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I1_W;
            Eigen::Quaterniond q_I1_W; 
            t_I1_W(0) = gt1_msg.pose.pose.position.x;
            t_I1_W(1) = gt1_msg.pose.pose.position.y;
            t_I1_W(2) = gt1_msg.pose.pose.position.z;

            auto gt2_msg = gt2_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I2_W;
            Eigen::Quaterniond q_I2_W; 
            t_I2_W(0) = gt2_msg.pose.pose.position.x;
            t_I2_W(1) = gt2_msg.pose.pose.position.y;
            t_I2_W(2) = gt2_msg.pose.pose.position.z;
            q_I2_W.x() = gt2_msg.pose.pose.orientation.x;
            q_I2_W.y() = gt2_msg.pose.pose.orientation.y;
            q_I2_W.z() = gt2_msg.pose.pose.orientation.z;
            q_I2_W.w() = gt2_msg.pose.pose.orientation.w;
            Eigen::Matrix<double, 3, 3> R_I2_W = q_I2_W.toRotationMatrix();
            Eigen::Matrix<double, 3, 1> t_I1_I2 = R_I2_W * (t_I1_W - t_I2_W);

            // Append to our relative_position vector
            geometry_msgs::PointStamped rel_pos_temp;
            rel_pos_temp.header.stamp = ros::Time().fromSec(time);
            rel_pos_temp.point.x = t_I1_I2(0);
            rel_pos_temp.point.y = t_I1_I2(1);
            rel_pos_temp.point.z = t_I1_I2(2);
            rel_p_buffer.push_back(rel_pos_temp);

            // printf("find rel pos\n");
            gt1_buffer.pop_front();
            gt2_buffer.pop_front();
        }
    }
}

void Get_Abs_Pos()
{
    if (!rel_p_buffer.empty() && !odo2_p_buffer.empty())
    {
        double time1 = rel_p_buffer.front().header.stamp.toSec();
        double time2 = odo2_p_buffer.front().header.stamp.toSec();
        // printf("%f %f\n", time1, time2);

        if(time1 < time2 - 0.03)
        {
            rel_p_buffer.pop_front();
            // printf("throw rel_p\n");
        }
        else if(time1 > time2 + 0.03)
        {
            odo2_p_buffer.pop_front();
            // printf("throw odo2_p\n");
        }
        else
        {
            double time = time1;
            // printf("composed pos %f\n", time);

            auto odo2_msg = odo2_p_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I2_W;
            Eigen::Quaterniond q_I2_W; 
            t_I2_W(0) = odo2_msg.pose.position.x;
            t_I2_W(1) = odo2_msg.pose.position.y;
            t_I2_W(2) = odo2_msg.pose.position.z;
            q_I2_W.x() = odo2_msg.pose.orientation.x;
            q_I2_W.y() = odo2_msg.pose.orientation.y;
            q_I2_W.z() = odo2_msg.pose.orientation.z;
            q_I2_W.w() = odo2_msg.pose.orientation.w;
            Eigen::Matrix<double, 3, 3> R_I2_W = q_I2_W.toRotationMatrix();

            auto rel_pos_temp = rel_p_buffer.front();
            Eigen::Matrix<double, 3, 1> t_I1_I2;
            t_I1_I2(0) = rel_pos_temp.point.x;
            t_I1_I2(1) = rel_pos_temp.point.y;
            t_I1_I2(2) = rel_pos_temp.point.z;

            Eigen::Matrix<double, 3, 1> t_I1_W = t_I2_W + R_I2_W.transpose() * t_I1_I2;
            geometry_msgs::PointStamped abs_pos_temp;
            abs_pos_temp.header.stamp = ros::Time().fromSec(time);
            abs_pos_temp.point.x = t_I1_W(0);
            abs_pos_temp.point.y = t_I1_W(1);
            abs_pos_temp.point.z = t_I1_W(2);
            abs_pos_pub.publish(abs_pos_temp);

            // outFile_odo_pose << std::fixed << std::setprecision(6) << time << " " << t_I1_W(0) << " " << t_I1_W(1) << " " << t_I1_W(2) << " " << std::endl;
            
            // printf("find composed pos\n");
            rel_p_buffer.pop_front();
            odo2_p_buffer.pop_front();

            static double init_time = -1;
            if (init_time < 0)
            {
                init_time = time;
            }

            std_msgs::Bool signal_temp;
            if (time < init_time + 30)
            {
                signal_temp.data = false;
            }
            else if (time < init_time + 60)
            {
                signal_temp.data = true;
            }
            else
            {
                signal_temp.data = false;
            }
            signal_pub.publish(signal_temp);
        }
    }
}