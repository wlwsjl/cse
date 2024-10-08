#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Eigen>
#include <fstream>

std::ofstream outFile_gps;

void position_callback(sensor_msgs::NavSatFix pos_msg)
{
    double t = pos_msg.header.stamp.toSec();
    double x = pos_msg.latitude;
    double y = pos_msg.longitude;
    double z = pos_msg.altitude;

    double cx = pos_msg.position_covariance[0];
    double cy = pos_msg.position_covariance[4];
    double cz = pos_msg.position_covariance[8];

    std::cout << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << std::endl;

    outFile_gps << std::fixed << std::setprecision(6) << t << " " << x << " " << y << " " << z << " "
                << cx << " " << 0 << " " << 0 << " "
                << 0 << " " << cy << " " << 0 << " "
                << 0 << " " << 0 << " " << cz << std::endl;
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "sub_gps");
    ros::NodeHandle n;

    outFile_gps.open("/home/junlin/GNSS/eval/gps.txt");
    ros::Subscriber pos_subscriber = n.subscribe("/fla/gps/fix", 1000, position_callback);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    ros::spin();
  
    return 0;
}