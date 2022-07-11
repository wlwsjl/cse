#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
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

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "create_bag");
    auto nh = std::make_shared<ros::NodeHandle>("~");
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // ros::Publisher pub_pathimu;
    // pub_pathimu = nh->advertise<nav_msgs::Path>("/ov_msckf/pathimu", 1000);
    // ros::Rate loop_rate(1000);

    // rosbag::Bag bag_vio_traj;
    // bag_vio_traj.open("/home/junlin/CSE/vio_result/bag_vio_traj_aerolab.bag");  // BagMode is Read by default
    // std::string vio_topic_name = "/vio_odo";
    // std::vector<std::string> vio_topics;
    // vio_topics.push_back(vio_topic_name);
    // rosbag::View view_bag_vio(bag_vio_traj, rosbag::TopicQuery(vio_topics));
    // std::vector<geometry_msgs::PoseStamped> poses_imu;
    // std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_cov_buffer;

    // for(rosbag::View::iterator it = view_bag_vio.begin();it != view_bag_vio.end() && ros::ok();it++)
    // {
    //     if(it->getTopic() == vio_topic_name)
    //     {
    //         geometry_msgs::PoseWithCovarianceStamped::ConstPtr vio_pose_Msg = it->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    //         if (vio_pose_Msg != nullptr)
    //         {
    //             // printf("%d %f\n", vio_pose_Msg->header.seq, vio_pose_Msg->header.stamp.toSec());
    //             // getchar();

    //             poses_cov_buffer.push_back(*vio_pose_Msg);

    //             // Append to our pose vector
    //             geometry_msgs::PoseStamped posetemp;
    //             posetemp.header = vio_pose_Msg->header;
    //             posetemp.pose = vio_pose_Msg->pose.pose;
    //             poses_imu.push_back(posetemp);

    //             // Create our path (imu)
    //             // NOTE: We downsample the number of poses as needed to prevent rviz crashes
    //             // NOTE: https://github.com/ros-visualization/rviz/issues/1107
    //             nav_msgs::Path arrIMU;
    //             arrIMU.header.stamp = vio_pose_Msg->header.stamp;
    //             arrIMU.header.seq = vio_pose_Msg->header.seq;
    //             arrIMU.header.frame_id = "global";
    //             for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
    //                 arrIMU.poses.push_back(poses_imu.at(i));
    //             }
    //             pub_pathimu.publish(arrIMU);

    //             ros::spinOnce();
    //             loop_rate.sleep();
    //         }
    //     }        
    // } 
    // bag_vio_traj.close();

    // for (int i = 1; i < poses_imu.size(); i++)
    // {
    //     double dt = poses_imu.at(i).header.stamp.toSec() - poses_imu.at(i-1).header.stamp.toSec();
    //     if (dt < 0 || dt > 0.08)
    //     {
    //         printf("poses_imu dt err %f\n", dt);
    //         getchar();
    //     }
    // }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Load groundtruth if we have it
    std::string path_to_bag_gt;
    nh->param<std::string>("path_bag", path_to_bag_gt, "");
    PRINT_DEBUG("ros bag_gt path is: %s\n", path_to_bag_gt.c_str());

    rosbag::Bag bag_gt;
    bag_gt.open(path_to_bag_gt);  // BagMode is Read by default
    std::string gt1_topic_name = "/vrpn_client_node/quail/pose";
    std::string gt2_topic_name = "/uav_1/mavros/vision_pose/pose";
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

    // // construct relative_position_buffer
    // // double shift_time = gt1_states.begin()->header.stamp.toSec() - gt2_states.begin()->header.stamp.toSec();
    // double shift_time = 0.0;
    // std::vector<geometry_msgs::PoseStamped> gt2_states_cut;
    // for (auto const& x : gt2_states)
    // {
    //     double t = x.header.stamp.toSec();
    //     if (t > gt1_states.rbegin()->header.stamp.toSec() - shift_time)
    //     {
    //         break;
    //     }
    //     gt2_states_cut.push_back(x);
    // }
    // printf("gt2_states_cut %d %f %f\n", gt2_states_cut.size(), gt2_states_cut.begin()->header.stamp.toSec(), gt2_states_cut.rbegin()->header.stamp.toSec());

    // std::vector<geometry_msgs::PointStamped> relative_position_buffer;
    // double max_dis = -1;
    // for (auto const& x : gt1_states)
    // {
    //     double t1 = x.header.stamp.toSec() - shift_time;
    //     Eigen::Matrix<double, 4, 1> q_B1_W;        
    //     q_B1_W(0, 0) = x.pose.orientation.x; // quat
    //     q_B1_W(1, 0) = x.pose.orientation.y;
    //     q_B1_W(2, 0) = x.pose.orientation.z;
    //     q_B1_W(3, 0) = x.pose.orientation.w;
    //     Eigen::Matrix<double, 3, 3> R_B1_W = ov_core::quat_2_Rot(q_B1_W);

    //     // Eigen::Quaterniond q_B1_W;        
    //     // q_B1_W.x() = x.pose.orientation.x; // quat
    //     // q_B1_W.y() = x.pose.orientation.y;
    //     // q_B1_W.z() = x.pose.orientation.z;
    //     // q_B1_W.w() = x.pose.orientation.w;
    //     // Eigen::Matrix<double, 3, 3> R_B1_W = q_B1_W.toRotationMatrix();
        
    //     Eigen::Matrix<double, 3, 1> t_B1_W;
    //     t_B1_W(0, 0) = x.pose.position.x; // pos
    //     t_B1_W(1, 0) = x.pose.position.y;
    //     t_B1_W(2, 0) = x.pose.position.z;

    //     for (auto const& y : gt2_states_cut)
    //     {
    //         double t2 = y.header.stamp.toSec();
    //         if (fabs(t1 - t2) < 0.005)
    //         {
    //             Eigen::Matrix<double, 3, 1> t_B2_W;
    //             t_B2_W(0, 0) = y.pose.position.x; // pos
    //             t_B2_W(1, 0) = y.pose.position.y;
    //             t_B2_W(2, 0) = y.pose.position.z;
    //             Eigen::Matrix<double, 3, 1> t_B2_B1 = R_B1_W * (t_B2_W - t_B1_W);
    //             if (t_B2_B1.norm() > max_dis)
    //             {
    //                 max_dis = t_B2_B1.norm();
    //             }

    //             // Append to our relative_position vector
    //             geometry_msgs::PointStamped posetemp;
    //             posetemp.header.stamp = ros::Time().fromSec(t2);
    //             posetemp.point.x = t_B2_B1(0);
    //             posetemp.point.y = t_B2_B1(1);
    //             posetemp.point.z = t_B2_B1(2);
    //             relative_position_buffer.push_back(posetemp);
                
    //             break;
    //         }
    //     }
    // }
    // if (relative_position_buffer.size() == 0)
    // {
    //     printf("relative_position_buffer.size() == 0\n");
    // }
    // printf("relative_position_buffer %d %f %f\n", relative_position_buffer.size(), relative_position_buffer.begin()->header.stamp.toSec(), relative_position_buffer.rbegin()->header.stamp.toSec());
    // printf("max_dis %f\n", max_dis);

    // for (int i = 1; i < relative_position_buffer.size(); i++)
    // {
    //     double dt = relative_position_buffer.at(i).header.stamp.toSec() - relative_position_buffer.at(i-1).header.stamp.toSec();
    //     if (dt < 0 || dt > 0.05)
    //     {
    //         printf("relative_position_buffer dt err %f\n", dt);
    //         getchar();
    //     }
    // }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    string cam_distortion_model = "radtan";
    cv::Vec4d cam_intrinsics;
    cv::Vec4d cam_distortion_coeffs;

    cam_intrinsics[0] = 390.878;
    cam_intrinsics[1] = 392.270;
    cam_intrinsics[2] = 319.832;
    cam_intrinsics[3] = 242.767;

    cam_distortion_coeffs[0] = 0.015;
    cam_distortion_coeffs[1] = -0.008;
    cam_distortion_coeffs[2] = 0.003;
    cam_distortion_coeffs[3] = -0.006;

    // B C
    Eigen::Matrix3d R_BtoC;
    R_BtoC << -0.056876,  -0.997753, -0.0354129,
            -0.0595952,  0.0387999,  -0.997468,
            0.996601, -0.0546216, -0.0616681;
    Eigen::Vector3d p_BinC;
    p_BinC << 0.00513002, -0.0474967,   -0.15212;

    double roll = 0.0;
    double yaw = 0.05;
    double pitch = -0.05;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    // Location of the ROS bag we want to read in
    std::string path_to_bag1;
    nh->param<std::string>("path_bag", path_to_bag1, "");
    PRINT_DEBUG("ros bag1 path is: %s\n", path_to_bag1.c_str());
    
    rosbag::Bag bag1;
    bag1.open(path_to_bag1);  // BagMode is Read by default
    std::string cam_topic_name = "/camera/infra1/image_rect_raw";
    std::vector<std::string> img_topics;
    img_topics.push_back(cam_topic_name);
    rosbag::View view_bag1(bag1, rosbag::TopicQuery(img_topics));
    std::vector<double> cam_msg_buffer;
    int valid_img_cnt = 0;

    // for(rosbag::View::iterator it = view_bag1.begin();it != view_bag1.end() && ros::ok();it++)
    // {
    //     if(it->getTopic() == cam_topic_name)
    //     {
    //         auto cam_Msg = it->instantiate<sensor_msgs::Image>();
    //         if (cam_Msg != NULL)
    //         {
    //             // Get the image
    //             cv_bridge::CvImageConstPtr cv_ptr;
    //             try {
    //                 cv_ptr = cv_bridge::toCvShare(cam_Msg, sensor_msgs::image_encodings::MONO8);
    //             } catch (cv_bridge::Exception &e) {
    //                 PRINT_ERROR("cv_bridge exception: %s", e.what());
    //                 getchar();
    //             }

    //             // Create the measurement
    //             double img_timestamp = cv_ptr->header.stamp.toSec();
    //             cam_msg_buffer.push_back(img_timestamp);
    //             cv::Mat img = cv_ptr->image.clone();

    //             for (auto const& y : relative_position_buffer)
    //             {
    //                 double t2 = y.header.stamp.toSec();
    //                 if (fabs(img_timestamp - t2) < 0.005)
    //                 {
    //                     valid_img_cnt++;
    //                     Eigen::Matrix<double, 3, 1> t_B2_B1;
    //                     t_B2_B1(0) = y.point.x;
    //                     t_B2_B1(1) = y.point.y;
    //                     t_B2_B1(2) = y.point.z;
    //                     Eigen::Matrix<double, 3, 1> t_B2_c = p_BinC + R_BtoC * t_B2_B1;
    //                     // Eigen::Matrix<double, 3, 1> t_B2_c = rotationMatrix * (p_BinC + R_BtoC * t_B2_B1);
    //                     cout << t_B2_c.transpose() << endl;

    //                     cv::Point2f norm_pt;
    //                     norm_pt.x = t_B2_c(0)/t_B2_c(2);
    //                     norm_pt.y = t_B2_c(1)/t_B2_c(2);
    //                     cout << "norm_pt " << norm_pt.x << " " << norm_pt.y << endl;
    //                     vector<cv::Point2f> cam_points_undistorted;
    //                     cam_points_undistorted.push_back(norm_pt);

    //                     vector<cv::Point2f> cam_points = distortPoints(cam_points_undistorted, cam_intrinsics, cam_distortion_model, cam_distortion_coeffs);

    //                     cv::Point2f pt = cam_points.at(0);
    //                     cout << "pt " << pt.x << " " << pt.y << endl;
    //                     cv::Point2f pt_top = cv::Point2f(pt.x - 10, pt.y - 10);
    //                     cv::Point2f pt_bot = cv::Point2f(pt.x + 10, pt.y + 10);
    //                     cv::cvtColor(img, img, CV_GRAY2BGR);
    //                     cv::rectangle(img, pt_top, pt_bot, cv::Scalar(0, 0, 255), 1);

    //                     cv::imshow("img", img);
    //                     cv::waitKey(5);
    //                     break;
    //                 }
    //             }
    //         }
    //     }        
    // }

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
                    cv_ptr = cv_bridge::toCvShare(cam_Msg, sensor_msgs::image_encodings::MONO8);
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

                Eigen::Matrix<double, 3, 1> t_B2_c = p_BinC + R_BtoC * t_B2_B1;
                // Eigen::Matrix<double, 3, 1> t_B2_c = rotationMatrix * (p_BinC + R_BtoC * t_B2_B1);
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
                cv::cvtColor(img, img, CV_GRAY2BGR);
                cv::rectangle(img, pt_top, pt_bot, cv::Scalar(0, 0, 255), 1);

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
    printf("valid_img_cnt %d\n", valid_img_cnt);

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