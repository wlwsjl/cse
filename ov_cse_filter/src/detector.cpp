#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
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
Eigen::Matrix<double, 3, 1> t_B_A;
bool t_B_A_received = false;

std::ofstream outFile_gt_pose;
std::ofstream outFile_cam_pose;

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

      if (t_B_A_received)
      {
        double t = msg->header.stamp.toSec();
        for(unsigned int i = 0; i < ids.size(); i++)
        {
          if (ids[i] == 0)
          {
            outFile_cam_pose << std::fixed << std::setprecision(6) << t << " " << tvecs[i](0) << " " << tvecs[i](1) << " " << tvecs[i](2) << " " << std::endl;
            outFile_gt_pose << std::fixed << std::setprecision(6) << t << " " << t_B_A(0) << " " << t_B_A(1) << " " << t_B_A(2) << " " << std::endl;
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
  Eigen::Quaterniond q_A_W;        
  q_A_W.x() = gt_msg.pose.orientation.x; // quat
  q_A_W.y() = gt_msg.pose.orientation.y;
  q_A_W.z() = gt_msg.pose.orientation.z;
  q_A_W.w() = gt_msg.pose.orientation.w;
  Eigen::Matrix<double, 3, 3> R_A_W = q_A_W.toRotationMatrix();

  Eigen::Matrix<double, 3, 1> t_A_W;
  t_A_W(0, 0) = gt_msg.pose.position.x; // pos
  t_A_W(1, 0) = gt_msg.pose.position.y;
  t_A_W(2, 0) = gt_msg.pose.position.z;

  if (gt_uav_b_received)
  {
    Eigen::Matrix<double, 3, 1> t_B_W;
    t_B_W(0, 0) = gt_msg_uav_b.pose.position.x; // pos
    t_B_W(1, 0) = gt_msg_uav_b.pose.position.y;
    t_B_W(2, 0) = gt_msg_uav_b.pose.position.z;
    t_B_A = R_A_W.transpose() * (t_B_W - t_A_W);
    t_B_A_received = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detector");
  ros::NodeHandle nh;

  // // Camera intrinsics
  // double fx = 416.85223429743274;
  // double fy = 414.92069080087543;
  // double cx = 421.02459311003213;
  // double cy = 237.76180565241077;
  // camMatrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  // // distortion coefficients
  // double k1 = -0.045761895748285604;
  // double k2 = 0.03423951132164367;
  // double p1 = -0.00040139057556727315;
  // double p2 = 0.000431371425853453;
  // distCoeffs = (Mat1d(1, 4) << k1, k2, p1, p2);

  outFile_gt_pose.open("/home/junlin/GNSS/eval/gt.txt");
  outFile_cam_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

  camMatrix = cv::Mat::zeros(3, 3, CV_64F);
  distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
  ros::Subscriber caminfo_sub = nh.subscribe("/d455/color/camera_info", 1, camInfoCallback);

  ros::Subscriber image_sub = nh.subscribe("/d455/color/image_raw/compressed", 1, imageCallback);
  ros::Subscriber gt_uav_a_sub = nh.subscribe("/vrpn_client_node/uav_a/pose", 1, gt_uav_a_callback);
  ros::Subscriber gt_uav_b_sub = nh.subscribe("/vrpn_client_node/uav_b/pose", 1, gt_uav_b_callback);
  ros::spin();
}