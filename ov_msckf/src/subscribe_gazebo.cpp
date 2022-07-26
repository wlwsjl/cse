#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>

rosbag::Bag bag;

// Messages for getting model and link poses
// geometry_msgs::Pose ball_model_pose, ball_link_pose;
geometry_msgs::PoseStamped gt1_pose, gt2_pose;

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

// void model_states_callback(gazebo_msgs::ModelStates model_states)
// {
//     int ball_model_index = getIndex(model_states.name, "ball");
//     ball_model_pose = model_states.pose[ball_model_index];
// }

// void link_states_callback(gazebo_msgs::LinkStates link_states)
// {
//     int ball_link_index = getIndex(link_states.name, "ball::body");
//     ball_link_pose = link_states.pose[ball_link_index];
// }

void link_states_callback(gazebo_msgs::LinkStates link_states)
{
    ros::Time gt_time = ros::Time::now();
    int gt1_index = getIndex(link_states.name, "iris_0::base_link");
    gt1_pose.header.stamp = gt_time;
    gt1_pose.header.frame_id = 'map';
    gt1_pose.pose = link_states.pose[gt1_index];

    int gt2_index = getIndex(link_states.name, "iris_1::realsense_camera::link");
    gt2_pose.header.stamp = gt_time;
    gt2_pose.header.frame_id = 'map';
    gt2_pose.pose = link_states.pose[gt2_index];

    bag.write("/gt1_pose", gt_time, gt1_pose);
    bag.write("/gt2_pose", gt_time, gt2_pose);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ros::Time img_time = ros::Time::now();
    // sensor_msgs::Image img = *img_msg;
    // img.header.stamp = img_time;
    // bag.write("/img2_left", img_time, img);

    bag.write("/img2_left", img_msg->header.stamp, img_msg);
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "model_states_handler");
    ros::NodeHandle n;

    bag.open("/home/junlin/dataset_sim.bag", rosbag::bagmode::Write);
  
    // Create suvsvribers for Gazebo model and link states
    // ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 100, model_states_callback);
    // ros::Subscriber link_states_subscriber = n.subscribe("/gazebo/link_states", 100, link_states_callback);

    ros::Subscriber link_states_subscriber = n.subscribe("/gazebo/link_states", 100, link_states_callback);
    ros::Subscriber img_subscriber = n.subscribe("/iris_1/stereo_camera/left/image_raw", 100, img_callback);

    ros::spin();
  
    return 0;
}