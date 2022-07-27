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
    std::map<double, Eigen::Matrix<double, 17, 1>> gt1_states;
    std::map<double, Eigen::Matrix<double, 17, 1>> gt2_states;
    
    if (nh->hasParam("path_gt1") && nh->hasParam("path_gt2"))
    {
        std::string path_to_gt;
        nh->param<std::string>("path_gt1", path_to_gt, "");
        std::cout << path_to_gt << std::endl;
        if (!path_to_gt.empty()) {
            ov_core::DatasetReader::load_gt_file(path_to_gt, gt1_states);
            PRINT_DEBUG("gt file path is: %s\n", path_to_gt.c_str());
        }
        nh->param<std::string>("path_gt2", path_to_gt, "");
        std::cout << path_to_gt << std::endl;
        if (!path_to_gt.empty()) {
            ov_core::DatasetReader::load_gt_file(path_to_gt, gt2_states);
            PRINT_DEBUG("gt file path is: %s\n", path_to_gt.c_str());
        }
    }
    printf("dataset1 %d %f %f\n", gt1_states.size(), gt1_states.begin()->first, gt1_states.rbegin()->first);
    printf("dataset2 %d %f %f\n", gt2_states.size(), gt2_states.begin()->first, gt2_states.rbegin()->first);

    std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_cov_buffer;
    int cnt = -1;
    for (auto const& x : gt1_states)
    {
        cnt++;
        if (cnt % 10 != 0)
        {
            continue;
        }
        geometry_msgs::PoseWithCovarianceStamped vio_pose_Msg;
        double t = x.first;
        vio_pose_Msg.header.stamp = ros::Time().fromSec(t);
        Eigen::Matrix<double, 4, 1> q_I1_W;        
        vio_pose_Msg.pose.pose.orientation.x = x.second(5, 0); // quat
        vio_pose_Msg.pose.pose.orientation.y = x.second(6, 0);
        vio_pose_Msg.pose.pose.orientation.z = x.second(7, 0);
        vio_pose_Msg.pose.pose.orientation.w = x.second(4, 0);
        
        vio_pose_Msg.pose.pose.position.x = x.second(1, 0); // pos
        vio_pose_Msg.pose.pose.position.y = x.second(2, 0);
        vio_pose_Msg.pose.pose.position.z = x.second(3, 0);

        Eigen::Matrix<double, 6, 6> covariance_vio_pose = Eigen::Matrix<double, 6, 6>::Zero();
        covariance_vio_pose.block(0,0,3,3) = 1.0e-4 * Eigen::Matrix<double, 3, 3>::Identity();
        covariance_vio_pose.block(3,3,3,3) = 1.0e-6 * Eigen::Matrix<double, 3, 3>::Identity();
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                vio_pose_Msg.pose.covariance[6 * r + c] = covariance_vio_pose(r, c);
            }
        }

        poses_cov_buffer.push_back(vio_pose_Msg);
    }

    // double shift_time = gt1_states.begin()->first - gt2_states.begin()->first;
    double shift_time = 0.0;
    std::map<double, Eigen::Matrix<double, 17, 1>> gt2_states_cut;
    for (auto const& x : gt2_states)
    {
        double t = x.first;
        if (t > gt1_states.rbegin()->first - shift_time)
        {
            break;
        }
        gt2_states_cut.insert({t, x.second});
    }
    printf("gt2_states_cut %d %f %f\n", gt2_states_cut.size(), gt2_states_cut.begin()->first, gt2_states_cut.rbegin()->first);

    std::vector<geometry_msgs::PointStamped> relative_position_buffer;
    double max_dis = -1;
    for (auto const& x : gt1_states)
    {
        double t1 = x.first - shift_time;
        Eigen::Matrix<double, 4, 1> q_I1_W;        
        q_I1_W(0, 0) = x.second(5, 0); // quat
        q_I1_W(1, 0) = x.second(6, 0);
        q_I1_W(2, 0) = x.second(7, 0);
        q_I1_W(3, 0) = x.second(4, 0);
        Eigen::Matrix<double, 3, 3> R_I1_W = ov_core::quat_2_Rot(q_I1_W);

        // Eigen::Quaterniond q_I1_W;
        // q_I1_W.x() = x.second(5, 0); // quat
        // q_I1_W.y() = x.second(6, 0);
        // q_I1_W.z() = x.second(7, 0);
        // q_I1_W.w() = x.second(4, 0);
        // Eigen::Matrix<double, 3, 3> R_I1_W = q_I1_W.toRotationMatrix();
        
        Eigen::Matrix<double, 3, 1> t_I1_W;
        t_I1_W(0, 0) = x.second(1, 0); // pos
        t_I1_W(1, 0) = x.second(2, 0);
        t_I1_W(2, 0) = x.second(3, 0);

        for (auto const& y : gt2_states_cut)
        {
            double t2 = y.first;
            if (fabs(t1 - t2) < 0.002)
            {
                Eigen::Matrix<double, 3, 1> t_I2_W = y.second.block(1, 0, 3, 1);
                Eigen::Matrix<double, 3, 1> t_I2_I1 = R_I1_W * (t_I2_W - t_I1_W);
                if (t_I2_I1.norm() > max_dis)
                {
                    max_dis = t_I2_I1.norm();
                }

                // Append to our relative_position vector
                geometry_msgs::PointStamped posetemp;
                posetemp.header.stamp = ros::Time().fromSec(t2);
                posetemp.point.x = t_I2_I1(0);
                posetemp.point.y = t_I2_I1(1);
                posetemp.point.z = t_I2_I1(2);
                relative_position_buffer.push_back(posetemp);
                
                break;
            }
        }
    }
    if (relative_position_buffer.size() == 0)
    {
        printf("relative_position_buffer.size() == 0\n");
    }
    printf("relative_position_buffer %d %f %f\n", relative_position_buffer.size(), relative_position_buffer.begin()->header.stamp.toSec(), relative_position_buffer.rbegin()->header.stamp.toSec());
    printf("max_dis %f\n", max_dis);

    for (int i = 1; i < relative_position_buffer.size(); i++)
    {
        double dt = relative_position_buffer.at(i).header.stamp.toSec() - relative_position_buffer.at(i-1).header.stamp.toSec();
        if (dt < 0 || dt > 0.02)
        {
            printf("relative_position_buffer dt err %f\n", dt);
            getchar();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    string cam_distortion_model = "radtan";
    cv::Vec4d cam_intrinsics;
    cv::Vec4d cam_distortion_coeffs;

    // // default
    // cam_intrinsics[0] = 385.7544860839844;
    // cam_intrinsics[1] = 385.7544860839844;
    // cam_intrinsics[2] = 323.1204833984375;
    // cam_intrinsics[3] = 236.7432098388672;

    // cam_distortion_coeffs[0] = 0;
    // cam_distortion_coeffs[1] = 0;
    // cam_distortion_coeffs[2] = 0;
    // cam_distortion_coeffs[3] = 0;

    // Eigen::Isometry3d T;
    // Eigen::Matrix4d T_CtoI;
    // T_CtoI << 0.99993848, -0.01087644,  0.00217611, -0.00490239,
    //         0.01085902,  0.9999101,   0.00786587,  0.00131864,
    //         -0.00226147, -0.00784175,  0.9999667,  0.01165503,
    //         0.000,0.000,0.000,1.000;
    // T.linear() = T_CtoI.block(0, 0 , 3, 3);
    // T.translation() = T_CtoI.block(0, 3, 3, 1);
    // Eigen::Isometry3d T_cam_imu = T.inverse();
    // Eigen::Matrix3d R_c_i = T_cam_imu.linear();
    // Eigen::Vector3d t_i_c = T_cam_imu.translation();

    // calib r and t
    cam_intrinsics[0] = 390.423;
    cam_intrinsics[1] = 388.618;
    cam_intrinsics[2] = 320.599;
    cam_intrinsics[3] = 240.203;

    cam_distortion_coeffs[0] = 0.010;
    cam_distortion_coeffs[1] = -0.010;
    cam_distortion_coeffs[2] = 0.002;
    cam_distortion_coeffs[3] = -0.005;

    Eigen::Matrix3d R_c_i;
    R_c_i << 0.999903,   0.0138569, -0.00162008,
            -0.0138701,    0.999868, -0.00845159,
            0.00150276,  0.00847324,    0.999963;
    Eigen::Vector3d t_i_c;
    t_i_c << 0.00461925, -0.00339613,  -0.0350073;

    // double roll = 0.0;
    // double yaw = 0.05;
    // double pitch = -0.05;
    // Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    // Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    // Eigen::Matrix3d rotationMatrix = q.matrix();

    // // Location of the ROS bag we want to read in
    // std::string path_to_bag1;
    // nh->param<std::string>("path_bag", path_to_bag1, "");
    // PRINT_DEBUG("ros bag1 path is: %s\n", path_to_bag1.c_str());
    
    // rosbag::Bag bag1;
    // bag1.open(path_to_bag1);  // BagMode is Read by default
    // std::string cam_topic_name = "/camera/infra1/image_rect_raw";
    // std::vector<std::string> img_topics;
    // img_topics.push_back(cam_topic_name);
    // rosbag::View view_bag1(bag1, rosbag::TopicQuery(img_topics));
    // std::vector<double> cam_msg_buffer;
    // int valid_img_cnt = 0;
    // cv::VideoWriter video("/home/junlin/auto_label.mp4",CV_FOURCC('D','I','V','X'),30, cv::Size(640,480));
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
    //                     Eigen::Matrix<double, 3, 1> t_I2_I1;
    //                     t_I2_I1(0) = y.point.x;
    //                     t_I2_I1(1) = y.point.y;
    //                     t_I2_I1(2) = y.point.z;
    //                     // Eigen::Matrix<double, 3, 1> t_I2_c = t_i_c + R_c_i * t_I2_I1;
    //                     Eigen::Matrix<double, 3, 1> t_I2_c = t_i_c + R_c_i * rotationMatrix * t_I2_I1;
    //                     cout << t_I2_c.transpose() << endl;

    //                     cv::Point2f norm_pt;
    //                     norm_pt.x = t_I2_c(0)/t_I2_c(2);
    //                     norm_pt.y = t_I2_c(1)/t_I2_c(2);
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

    //                     video.write(img);
    //                     cv::imshow("img", img);
    //                     cv::waitKey(5);
    //                     break;
    //                 }
    //             }
    //         }
    //     }        
    // }
    // bag1.close();
    // printf("valid_img_cnt %d\n", valid_img_cnt);
    // getchar();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::ofstream outFile_pose;
    outFile_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

    std::vector<geometry_msgs::PoseWithCovarianceStamped> position_buffer1;
    std::vector<geometry_msgs::PointStamped> position_buffer2;

    // int cnt = -1;
    // for (auto const& x : gt1_states)
    // {
    //     cnt++;
    //     if (cnt % 10 != 0)
    //     {
    //         continue;
    //     }
    //     double t1 = x.first - shift_time;
    //     Eigen::Matrix<double, 4, 1> q_I1_W;        
    //     q_I1_W(0, 0) = x.second(5, 0); // quat
    //     q_I1_W(1, 0) = x.second(6, 0);
    //     q_I1_W(2, 0) = x.second(7, 0);
    //     q_I1_W(3, 0) = x.second(4, 0);
    //     Eigen::Matrix<double, 3, 3> R_I1_W = ov_core::quat_2_Rot(q_I1_W);

    //     // Eigen::Quaterniond q_I1_W;
    //     // q_I1_W.x() = x.second(5, 0); // quat
    //     // q_I1_W.y() = x.second(6, 0);
    //     // q_I1_W.z() = x.second(7, 0);
    //     // q_I1_W.w() = x.second(4, 0);
    //     // Eigen::Matrix<double, 3, 3> R_I1_W = q_I1_W.toRotationMatrix();
        
    //     Eigen::Matrix<double, 3, 1> t_I1_W;
    //     t_I1_W(0, 0) = x.second(1, 0); // pos
    //     t_I1_W(1, 0) = x.second(2, 0);
    //     t_I1_W(2, 0) = x.second(3, 0);
        
    //     for (auto const& y : relative_position_buffer)
    //     {
    //         double t2 = y.header.stamp.toSec();
    //         if (fabs(t1 - t2) < 0.005)
    //         {
    //             Eigen::Matrix<double, 3, 1> t_I2_I1;
    //             t_I2_I1(0) = y.point.x;
    //             t_I2_I1(1) = y.point.y;
    //             t_I2_I1(2) = y.point.z;
    //             Eigen::Matrix<double, 3, 1> t_I2_W = t_I1_W + R_I1_W.transpose() * t_I2_I1;

    //             double time_stamp = t2;
    //             outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
    //                             << t_I2_W(0) << " " << t_I2_W(1) << " " << t_I2_W(2) << " "
    //                             << std::endl;
                
    //             geometry_msgs::PoseWithCovarianceStamped p1;
    //             p1.header.stamp = ros::Time().fromSec(x.first - shift_time);
    //             p1.pose.pose.orientation.x = x.second(5, 0);
    //             p1.pose.pose.orientation.y = x.second(6, 0);
    //             p1.pose.pose.orientation.z = x.second(7, 0);
    //             p1.pose.pose.orientation.w = x.second(4, 0);
    //             p1.pose.pose.position.x = x.second(1, 0);
    //             p1.pose.pose.position.y = x.second(2, 0);
    //             p1.pose.pose.position.z = x.second(3, 0);
    //             Eigen::Matrix<double, 6, 6> covariance_posori = Eigen::Matrix<double, 6, 6>::Zero();
    //             covariance_posori.block(0, 0, 3, 3) = 1.0e-4 * Eigen::Matrix<double, 3, 3>::Identity();
    //             covariance_posori.block(3, 3, 3, 3) = 1.0e-6 * Eigen::Matrix<double, 3, 3>::Identity();
    //             for (int r = 0; r < 6; r++) {
    //                 for (int c = 0; c < 6; c++) {
    //                 p1.pose.covariance[6 * r + c] = covariance_posori(r, c);
    //                 }
    //             }
    //             position_buffer1.push_back(p1);
    //             position_buffer2.push_back(y);

    //             break;
    //         }
    //     }
    // }

    // from optitrack to imu
    // Eigen::Matrix<double, 4, 1> q1_test;
    // q1_test(0, 0) = -0.4841; // quat
    // q1_test(1, 0) = 0.4734;
    // q1_test(2, 0) = -0.4324;
    // q1_test(3, 0) = 0.5954;
    // Eigen::Matrix<double, 3, 3> R1_test = ov_core::quat_2_Rot(q1_test);
    // from vio world to imu
    // Eigen::Matrix<double, 4, 1> q2_test;
    // q2_test(0, 0) = -0.72301974196; // quat
    // q2_test(1, 0) = -0.00597789482798;
    // q2_test(2, 0) = -0.00590941341005;
    // q2_test(3, 0) = 0.690776227401;
    // Eigen::Matrix<double, 3, 3> R2_test = ov_core::quat_2_Rot(q2_test);

    // cout << R2_test.transpose() * R1_test << endl;
    // getchar();

    for (auto & x : poses_cov_buffer)
    {
        double t1 = x.header.stamp.toSec() - shift_time;
        Eigen::Matrix<double, 4, 1> q_I1_W;
        Eigen::Matrix<double, 3, 1> t_I1_W;
        q_I1_W(0, 0) = x.pose.pose.orientation.x; // quat
        q_I1_W(1, 0) = x.pose.pose.orientation.y;
        q_I1_W(2, 0) = x.pose.pose.orientation.z;
        q_I1_W(3, 0) = x.pose.pose.orientation.w;
        t_I1_W(0, 0) = x.pose.pose.position.x; // pos
        t_I1_W(1, 0) = x.pose.pose.position.y;
        t_I1_W(2, 0) = x.pose.pose.position.z;
        Eigen::Matrix<double, 3, 3> R_I1_W = ov_core::quat_2_Rot(q_I1_W);

        double time_stamp = x.header.stamp.toSec();
        outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
                        << x.pose.pose.position.x << " " << x.pose.pose.position.y << " " << x.pose.pose.position.z << " "
                        << std::endl;

        for (auto const& y : relative_position_buffer)
        {
            double t2 = y.header.stamp.toSec();
            if (fabs(t1 - t2) < 0.005)
            {
                Eigen::Matrix<double, 3, 1> t_I2_I1;
                t_I2_I1(0) = y.point.x;
                t_I2_I1(1) = y.point.y;
                t_I2_I1(2) = y.point.z;
                Eigen::Matrix<double, 3, 1> t_I2_W = t_I1_W + R_I1_W.transpose() * t_I2_I1;

                // double time_stamp = t2;
                // outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
                //                 << t_I2_W(0) << " " << t_I2_W(1) << " " << t_I2_W(2) << " "
                //                 << std::endl;

                x.header.stamp = ros::Time().fromSec(x.header.stamp.toSec() - shift_time);
                position_buffer1.push_back(x);
                position_buffer2.push_back(y);
                
                break;
            }
        }
    }

    for (int i = 1; i < position_buffer1.size(); i++)
    {
        double dt = position_buffer1.at(i).header.stamp.toSec() - position_buffer1.at(i-1).header.stamp.toSec();
        if (dt < 0 || dt > 0.08)
        {
            printf("position_buffer dt err %f\n", dt);
            getchar();
        }
    }

    printf("position_buffer %d %f %f\n", position_buffer1.size(), position_buffer1.begin()->header.stamp.toSec(), position_buffer1.rbegin()->header.stamp.toSec());

    std::vector<geometry_msgs::PoseStamped> traget_position_buffer;
    for (auto const& x : gt2_states_cut)
    {
        double t = x.first;
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time().fromSec(t); 
        msg.pose.orientation.x = x.second(5, 0); // quat
        msg.pose.orientation.y = x.second(6, 0);
        msg.pose.orientation.z = x.second(7, 0);
        msg.pose.orientation.w = x.second(4, 0);

        msg.pose.position.x = x.second(1, 0); // pos
        msg.pose.position.y = x.second(2, 0);
        msg.pose.position.z = x.second(3, 0);

        traget_position_buffer.push_back(msg);

        // outFile_pose << std::fixed << std::setprecision(6) << t << " "
        //                 << msg.pose.position.x << " " << msg.pose.position.y << " " << msg.pose.position.z << " "
        //                 << std::endl;
    }
    printf("target_position_buffer %d %f %f\n", traget_position_buffer.size(), traget_position_buffer.begin()->header.stamp.toSec(), traget_position_buffer.rbegin()->header.stamp.toSec());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Location of the ROS bag we want to read in
    std::string path_to_bag2;
    nh->param<std::string>("path_bag", path_to_bag2, "");
    PRINT_DEBUG("ros bag2 path is: %s\n", path_to_bag2.c_str());
    
    rosbag::Bag bag2;
    bag2.open(path_to_bag2);  // BagMode is Read by default
    std::string imu_topic_name = "/uav_1/mavros/imu/data";
    std::vector<std::string> topics;
    topics.push_back(imu_topic_name);
    rosbag::View view_bag2(bag2, rosbag::TopicQuery(topics));
    std::vector<sensor_msgs::Imu> imu_msg_buffer;
    for(rosbag::View::iterator it = view_bag2.begin();it != view_bag2.end() && ros::ok();it++)
    {
        if(it->getTopic() == imu_topic_name)
        {
            sensor_msgs::Imu::ConstPtr imu_Msg = it->instantiate<sensor_msgs::Imu>();
            if (imu_Msg != NULL)
            {
                sensor_msgs::Imu message = *imu_Msg;
                imu_msg_buffer.push_back(message);
            }
        }        
    }
    bag2.close();

    for (int i = 1; i < imu_msg_buffer.size(); i++)
    {
        double dt = imu_msg_buffer.at(i).header.stamp.toSec() - imu_msg_buffer.at(i-1).header.stamp.toSec();
        if (dt < 0)
        {
            printf("imu_msg_buffer dt err %f\n", dt);
            getchar();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    int used_imu_msg_cntr = 0;
    for (auto imu_data : imu_msg_buffer)
    {
        if (imu_data.header.stamp.toSec() < position_buffer1[0].header.stamp.toSec())
        {
            ++used_imu_msg_cntr;
        }
    }
    imu_msg_buffer.erase(imu_msg_buffer.begin(), imu_msg_buffer.begin()+used_imu_msg_cntr);

    // printf("first pos %f first imu %f\n", position_buffer1[0].header.stamp.toSec(), imu_msg_buffer[0].header.stamp.toSec());

    rosbag::Bag bag_data;
    bag_data.open("/home/junlin/CSE/data.bag", rosbag::bagmode::Write);

    for (int i = 0 ; i < traget_position_buffer.size(); i++)
    {
        bag_data.write("/target_pose", traget_position_buffer[i].header.stamp, traget_position_buffer[i]);
    }

    int pos_cnt = 0;
    while(pos_cnt < position_buffer1.size())
    {
        int used_imu_msg_cntr = 0;
        for (auto imu_data : imu_msg_buffer)
        {
            if (imu_data.header.stamp.toSec() <= position_buffer1[pos_cnt].header.stamp.toSec())
            {
                // printf("imu time %f\n", imu_data.header.stamp.toSec());
                bag_data.write("/imu", imu_data.header.stamp, imu_data);
                ++used_imu_msg_cntr;
            }
        }
        imu_msg_buffer.erase(imu_msg_buffer.begin(), imu_msg_buffer.begin()+used_imu_msg_cntr);

        // printf("pos %d time %f, imu_cnt %d\n", pos_cnt, position_buffer1[pos_cnt].header.stamp.toSec(), used_imu_msg_cntr);
        if ((used_imu_msg_cntr > 0) && (used_imu_msg_cntr > 20))
        {
            printf("imu_cnt err %d\n", used_imu_msg_cntr);
            break;
        }

        bag_data.write("/vio_pose", position_buffer1[pos_cnt].header.stamp, position_buffer1[pos_cnt]);
        bag_data.write("/relative_position", position_buffer2[pos_cnt].header.stamp, position_buffer2[pos_cnt]);

        ++pos_cnt;
    }
    bag_data.close();
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