#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/LinkStates.h>
#include <opencv2/opencv.hpp>

#include "utils/dataset_reader.h"
#include "utils/quat_ops.h"

vector<cv::Point2f> distortPoints(
    const vector<cv::Point2f>& pts_in,
    const cv::Vec4d& intrinsics,
    const string& distortion_model,
    const cv::Vec4d& distortion_coeffs);

void get_pose(const geometry_msgs::PoseStamped& pose0, const geometry_msgs::PoseStamped& pose1,
                const double& img_timestamp, Eigen::Matrix<double, 3, 3>& R_interp, Eigen::Matrix<double, 3, 1>& p_interp);

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "projection_test");
    auto nh = std::make_shared<ros::NodeHandle>("~");
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // ros::Publisher pub_path;
    // pub_path = nh->advertise<nav_msgs::Path>("/ov_msckf/pathimu", 1000);
    // ros::Rate loop_rate(1000);

    // rosbag::Bag bag_gt_traj;
    // bag_gt_traj.open("/home/junlin/dataset_sim.bag");  // BagMode is Read by default
    // std::string gt_topic_name = "/gt2_pose";
    // std::vector<std::string> gt_topics;
    // gt_topics.push_back(gt_topic_name);
    // rosbag::View view_bag_gt(bag_gt_traj, rosbag::TopicQuery(gt_topics));
    // std::vector<geometry_msgs::PoseStamped> poses_gt;

    // for(rosbag::View::iterator it = view_bag_gt.begin();it != view_bag_gt.end() && ros::ok();it++)
    // {
    //     if(it->getTopic() == gt_topic_name)
    //     {
    //         geometry_msgs::PoseStamped::ConstPtr pose_Msg = it->instantiate<geometry_msgs::PoseStamped>();
    //         if (pose_Msg != nullptr)
    //         {
    //             // Append to our pose vector
    //             geometry_msgs::PoseStamped posetemp;
    //             posetemp.header = pose_Msg->header;
    //             posetemp.pose = pose_Msg->pose;
    //             poses_gt.push_back(posetemp);

    //             // Create our path (imu)
    //             // NOTE: We downsample the number of poses as needed to prevent rviz crashes
    //             // NOTE: https://github.com/ros-visualization/rviz/issues/1107
    //             nav_msgs::Path arr;
    //             arr.header.stamp = pose_Msg->header.stamp;
    //             arr.header.frame_id = "global";
    //             for (size_t i = 0; i < poses_gt.size(); i += std::floor((double)poses_gt.size() / 16384.0) + 1) {
    //                 arr.poses.push_back(poses_gt.at(i));
    //             }
    //             pub_path.publish(arr);

    //             ros::spinOnce();
    //             loop_rate.sleep();
    //         }
    //     }        
    // } 
    // bag_gt_traj.close();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Load groundtruth if we have it
    std::string path_to_bag_gt;
    nh->param<std::string>("path_bag", path_to_bag_gt, "");
    PRINT_DEBUG("ros bag_gt path is: %s\n", path_to_bag_gt.c_str());

    rosbag::Bag bag_gt;
    bag_gt.open(path_to_bag_gt);  // BagMode is Read by default
    std::string gt1_topic_name = "/gt2_pose";
    std::string gt2_topic_name = "/gt1_pose";
    std::vector<std::string> topics_gt;
    topics_gt.push_back(gt1_topic_name);
    topics_gt.push_back(gt2_topic_name);
    rosbag::View view_bag_gt(bag_gt, rosbag::TopicQuery(topics_gt));
    std::vector<geometry_msgs::PoseStamped> gt1_states;
    std::vector<geometry_msgs::PoseStamped> gt2_states;
    for(rosbag::View::iterator it = view_bag_gt.begin();it != view_bag_gt.end() && ros::ok();it++)
    {
        if(it->getTopic() == gt1_topic_name)
        {
            geometry_msgs::PoseStamped::ConstPtr MoCap_Msg = it->instantiate<geometry_msgs::PoseStamped>();
            if (MoCap_Msg != NULL)
            {
              geometry_msgs::PoseStamped message = *MoCap_Msg;
              gt1_states.push_back(message);
            }
        }

        if(it->getTopic() == gt2_topic_name)
        {
            geometry_msgs::PoseStamped::ConstPtr MoCap_Msg = it->instantiate<geometry_msgs::PoseStamped>();
            if (MoCap_Msg != NULL)
            {
              geometry_msgs::PoseStamped message = *MoCap_Msg;
              gt2_states.push_back(message);
            }
        }
    }
    bag_gt.close();

    printf("dataset1 %d %f %f\n", gt1_states.size(), gt1_states.begin()->header.stamp.toSec(), gt1_states.rbegin()->header.stamp.toSec());
    printf("dataset2 %d %f %f\n", gt2_states.size(), gt2_states.begin()->header.stamp.toSec(), gt2_states.rbegin()->header.stamp.toSec());

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    string cam_distortion_model = "radtan";
    cv::Vec4d cam_intrinsics;
    cv::Vec4d cam_distortion_coeffs;

    cam_intrinsics[0] = 376.0;
    cam_intrinsics[1] = 376.0;
    cam_intrinsics[2] = 376.0;
    cam_intrinsics[3] = 240.0;

    cam_distortion_coeffs[0] = -0.1;
    cam_distortion_coeffs[1] = 0.01;
    cam_distortion_coeffs[2] = 5.0e-5;
    cam_distortion_coeffs[3] = -1.0e-4;

    // B C
    Eigen::Matrix3d R_BtoC;
    R_BtoC.setIdentity();
    double roll = 0.0;
    double pitch = -1.8;
    double yaw = 0.0;
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    R_BtoC = q.matrix();
    // cout << R_BtoC << endl;
    // getchar();

    Eigen::Vector3d p_BinC;
    p_BinC << 0.06, 0.0, 0.0;

    // Location of the ROS bag we want to read in
    std::string path_to_bag1;
    nh->param<std::string>("path_bag", path_to_bag1, "");
    PRINT_DEBUG("ros bag1 path is: %s\n", path_to_bag1.c_str());
    
    rosbag::Bag bag1;
    bag1.open(path_to_bag1);  // BagMode is Read by default
    std::string cam_topic_name = "/img2_left";
    std::vector<std::string> img_topics;
    img_topics.push_back(cam_topic_name);
    rosbag::View view_bag1(bag1, rosbag::TopicQuery(img_topics));
    std::vector<double> cam_msg_buffer;
    cv::VideoWriter video("/home/junlin/auto_label_sim.mp4",CV_FOURCC('D','I','V','X'),30, cv::Size(752,480));
    for(rosbag::View::iterator it = view_bag1.begin();it != view_bag1.end() && ros::ok();it++)
    {
        if(it->getTopic() == cam_topic_name)
        {
            auto cam_Msg = it->instantiate<sensor_msgs::Image>();
            if (cam_Msg != NULL)
            {
                // Get the image
                cv_bridge::CvImageConstPtr cv_ptr;
                try {
                    // cv_ptr = cv_bridge::toCvShare(cam_Msg, sensor_msgs::image_encodings::MONO8);
                    cv_ptr = cv_bridge::toCvShare(cam_Msg, sensor_msgs::image_encodings::BGR8);
                } catch (cv_bridge::Exception &e) {
                    PRINT_ERROR("cv_bridge exception: %s", e.what());
                    getchar();
                }

                // Create the measurement
                double img_timestamp = cv_ptr->header.stamp.toSec();
                cam_msg_buffer.push_back(img_timestamp);
                cv::Mat img = cv_ptr->image.clone();

                if (img_timestamp < gt1_states.begin()->header.stamp.toSec() || img_timestamp < gt2_states.begin()->header.stamp.toSec())
                {
                    continue;
                }

                int used_gt1_msg_cntr = 0;
                for (auto const& x : gt1_states)
                {
                    double t = x.header.stamp.toSec();
                    if (t > img_timestamp)
                    {
                        break;
                    }
                    ++used_gt1_msg_cntr;
                }
                // printf("gt1 %f img %f gt1 %f\n", gt1_states[used_gt1_msg_cntr-1].header.stamp.toSec(), img_timestamp, gt1_states[used_gt1_msg_cntr].header.stamp.toSec());

                int used_gt2_msg_cntr = 0;
                for (auto const& x : gt2_states)
                {
                    double t = x.header.stamp.toSec();
                    if (t > img_timestamp)
                    {
                        break;
                    }
                    ++used_gt2_msg_cntr;
                }
                // printf("gt2 %f img %f gt2 %f\n", gt2_states[used_gt2_msg_cntr-1].header.stamp.toSec(), img_timestamp, gt2_states[used_gt2_msg_cntr].header.stamp.toSec());

                Eigen::Matrix<double, 3, 3> R_B1_W;
                Eigen::Matrix<double, 3, 1> t_B1_W;
                get_pose(gt1_states[used_gt1_msg_cntr-1], gt1_states[used_gt1_msg_cntr], img_timestamp, R_B1_W,t_B1_W);
                Eigen::Matrix<double, 3, 3> R_B2_W;
                Eigen::Matrix<double, 3, 1> t_B2_W;
                get_pose(gt2_states[used_gt2_msg_cntr-1], gt2_states[used_gt2_msg_cntr], img_timestamp, R_B2_W,t_B2_W);

                Eigen::Matrix<double, 3, 1> t_B2_B1 = R_B1_W * (t_B2_W - t_B1_W);

                t_B2_B1 = R_BtoC * t_B2_B1;
                Eigen::Matrix<double, 3, 1> t_B2_B1_new;
                t_B2_B1_new(0) = -t_B2_B1(1);
                t_B2_B1_new(1) = -t_B2_B1(2);
                t_B2_B1_new(2) = t_B2_B1(0);

                Eigen::Matrix<double, 3, 1> t_B2_c = p_BinC + t_B2_B1_new;
                cout << t_B2_c.transpose() << endl;

                cv::Point2f norm_pt;
                norm_pt.x = t_B2_c(0)/t_B2_c(2);
                norm_pt.y = t_B2_c(1)/t_B2_c(2);
                cout << "norm_pt " << norm_pt.x << " " << norm_pt.y << endl;
                vector<cv::Point2f> cam_points_undistorted;
                cam_points_undistorted.push_back(norm_pt);

                vector<cv::Point2f> cam_points = distortPoints(cam_points_undistorted, cam_intrinsics, cam_distortion_model, cam_distortion_coeffs);

                cv::Point2f pt = cam_points.at(0);
                cout << "pt " << pt.x << " " << pt.y << endl;
                cv::Point2f pt_top = cv::Point2f(pt.x - 10, pt.y - 10);
                cv::Point2f pt_bot = cv::Point2f(pt.x + 10, pt.y + 10);
    
                cv::rectangle(img, pt_top, pt_bot, cv::Scalar(0, 0, 255), 1);

                video.write(img);
                cv::imshow("img", img);
                cv::waitKey(5);

                if (used_gt1_msg_cntr > 2)
                {
                    gt1_states.erase(gt1_states.begin(), gt1_states.begin()+used_gt1_msg_cntr-2);
                }

                if (used_gt2_msg_cntr > 2)
                {
                    gt2_states.erase(gt2_states.begin(), gt2_states.begin()+used_gt2_msg_cntr-2);
                }
            }
        }        
    }

    bag1.close();
    printf("finish!\n");

    ros::spin();
    return 0;
}

vector<cv::Point2f> distortPoints(
    const vector<cv::Point2f>& pts_in,
    const cv::Vec4d& intrinsics,
    const string& distortion_model,
    const cv::Vec4d& distortion_coeffs) {

  const cv::Matx33d K(intrinsics[0], 0.0, intrinsics[2],
                      0.0, intrinsics[1], intrinsics[3],
                      0.0, 0.0, 1.0);

  vector<cv::Point2f> pts_out;
  if (distortion_model == "radtan") {
    vector<cv::Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  } else if (distortion_model == "equidistant") {
    cv::fisheye::distortPoints(pts_in, pts_out, K, distortion_coeffs);
  } else {
    ROS_WARN_ONCE("The model %s is unrecognized, using radtan instead...",
                  distortion_model.c_str());
    vector<cv::Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  }

  return pts_out;
}

void get_pose(const geometry_msgs::PoseStamped& pose0, const geometry_msgs::PoseStamped& pose1,
                const double& img_timestamp, Eigen::Matrix<double, 3, 3>& R_interp, Eigen::Matrix<double, 3, 1>& p_interp)
{
    double lambda = (img_timestamp - pose0.header.stamp.toSec()) / (pose1.header.stamp.toSec() - pose0.header.stamp.toSec());

    // Bounding SO(3) orientations
    Eigen::Matrix<double, 4, 1> q0;        
    q0(0, 0) = pose0.pose.orientation.x; // quat
    q0(1, 0) = pose0.pose.orientation.y;
    q0(2, 0) = pose0.pose.orientation.z;
    q0(3, 0) = pose0.pose.orientation.w;
    Eigen::Matrix<double, 4, 1> q1;        
    q1(0, 0) = pose1.pose.orientation.x; // quat
    q1(1, 0) = pose1.pose.orientation.y;
    q1(2, 0) = pose1.pose.orientation.z;
    q1(3, 0) = pose1.pose.orientation.w;
    Eigen::Matrix<double, 3, 3> R_Gto0 = ov_core::quat_2_Rot(q0);
    Eigen::Matrix<double, 3, 3> R_Gto1 = ov_core::quat_2_Rot(q1);

    // Eigen::Quaterniond q0;        
    // q0.x() = pose0.pose.orientation.x; // quat
    // q0.y() = pose0.pose.orientation.y;
    // q0.z() = pose0.pose.orientation.z;
    // q0.w() = pose0.pose.orientation.w;
    // Eigen::Quaterniond q1;        
    // q1.x() = pose1.pose.orientation.x; // quat
    // q1.y() = pose1.pose.orientation.y;
    // q1.z() = pose1.pose.orientation.z;
    // q1.w() = pose1.pose.orientation.w;
    // Eigen::Matrix<double, 3, 3> R_Gto0 = q0.toRotationMatrix();
    // Eigen::Matrix<double, 3, 3> R_Gto1 = q1.toRotationMatrix();

    // Now perform the interpolation
    Eigen::Matrix<double, 3, 3> R_0to1 = R_Gto1 * R_Gto0.transpose();
    Eigen::Matrix<double, 3, 3> R_0toi = ov_core::exp_so3(lambda * ov_core::log_so3(R_0to1));
    R_interp = R_0toi * R_Gto0;

    Eigen::Matrix<double, 3, 1> p0;
    p0(0, 0) = pose0.pose.position.x; // pos
    p0(1, 0) = pose0.pose.position.y;
    p0(2, 0) = pose0.pose.position.z;
    Eigen::Matrix<double, 3, 1> p1;
    p1(0, 0) = pose1.pose.position.x; // pos
    p1(1, 0) = pose1.pose.position.y;
    p1(2, 0) = pose1.pose.position.z;
    p_interp = (1 - lambda) * p0 + lambda * p1;
}