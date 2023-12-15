#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>
#include <fstream>

using namespace cv;
using namespace std;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
cv::Mat camMatrix, distCoeffs;
bool haveCamInfo = false;
double markerLength = 0.12;

geometry_msgs::PoseStamped gt_msg_uav_b;
bool gt_uav_b_received = false;
bool gt_uav_a_received = false;

Eigen::Matrix3d R_ItoC;
Eigen::Vector3d p_IinC;
Eigen::Matrix3d R_CtoI;
Eigen::Vector3d p_CinI;
Eigen::Matrix<double, 3, 3> R_A_W;
Eigen::Matrix<double, 3, 1> t_A_W;

std::ofstream outFile_gt_pose;
std::ofstream outFile_cam_pose;

std::vector<geometry_msgs::PoseStamped> poses_b;
ros::Publisher pub_path;

// https://github.com/BYUMarsRover/aruco_detect/blob/master/src/aruco_detect.cpp
void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                camMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distCoeffs.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  if (!haveCamInfo)
  {
    return;
  }

  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat

    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(image, dictionary, corners, ids);
    if(ids.size() > 0)
    {
      aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

      if (gt_uav_a_received && gt_uav_b_received)
      {
        double t = msg->header.stamp.toSec();
        for(unsigned int i = 0; i < ids.size(); i++)
        {
          if (ids[i] == 0)
          {
            Eigen::Matrix<double, 3, 1> t_rel;
            t_rel << tvecs[i](0), tvecs[i](1), tvecs[i](2);
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

      // draw results
      aruco::drawDetectedMarkers(image, corners, ids);
      for(unsigned int i = 0; i < ids.size(); i++)
                aruco::drawAxis(image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
    }

    cv::imshow("out", image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
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
  ros::init(argc, argv, "detector_global");
  ros::NodeHandle nh("~");

  std::vector<double> R_temp;
  nh.param("R_ItoC", R_temp, std::vector<double>(9,1));
  R_ItoC = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&R_temp[0]);
  std::vector<double> p_temp;
  nh.param("p_IinC", p_temp, std::vector<double>(3,1));
  p_IinC = Eigen::Map<Eigen::Vector3d>(&p_temp[0]);

  R_CtoI = R_ItoC.transpose();
  p_CinI = -R_CtoI * p_IinC;

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

  std::string topic_camera_info;
  nh.param<std::string>("topic_camera_info", topic_camera_info, "/d455/color/camera_info");
  std::string topic_image;
  nh.param<std::string>("topic_image", topic_image, "/d455/color/image_raw/compressed");
  std::string topic_gt_uav_a;
  nh.param<std::string>("topic_gt_uav_a", topic_gt_uav_a, "/vrpn_client_node/uav_a/pose");
  std::string topic_gt_uav_b;
  nh.param<std::string>("topic_gt_uav_b", topic_gt_uav_b, "/vrpn_client_node/uav_b/pose");
  std::string topic_pub_path;
  nh.param<std::string>("topic_pub_path", topic_pub_path, "/path");

  camMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
  ros::Subscriber caminfo_sub = nh.subscribe(topic_camera_info, 1, camInfoCallback);

  ros::Subscriber image_sub = nh.subscribe(topic_image, 1, imageCallback);
  ros::Subscriber gt_uav_a_sub = nh.subscribe(topic_gt_uav_a, 1, gt_uav_a_callback);
  ros::Subscriber gt_uav_b_sub = nh.subscribe(topic_gt_uav_b, 1, gt_uav_b_callback);

  pub_path = nh.advertise<nav_msgs::Path>(topic_pub_path, 1);

  ros::spin();
}