#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

#include "utils/dataset_reader.h"
#include "utils/quat_ops.h"

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "create_bag");
    auto nh = std::make_shared<ros::NodeHandle>("~");
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher pub_pathimu;
    pub_pathimu = nh->advertise<nav_msgs::Path>("/ov_msckf/pathimu", 1000);
    ros::Rate loop_rate(1000);

    rosbag::Bag bag_vio_traj;
    bag_vio_traj.open("/home/junlin/CSE/vio_result/bag_vio_traj.bag");  // BagMode is Read by default
    std::string vio_topic_name = "/vio_odo";
    std::vector<std::string> vio_topics;
    vio_topics.push_back(vio_topic_name);
    rosbag::View view_bag_vio(bag_vio_traj, rosbag::TopicQuery(vio_topics));
    std::vector<geometry_msgs::PoseStamped> poses_imu;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_cov_buffer;

    for(rosbag::View::iterator it = view_bag_vio.begin();it != view_bag_vio.end() && ros::ok();it++)
    {
        if(it->getTopic() == vio_topic_name)
        {
            geometry_msgs::PoseWithCovarianceStamped::ConstPtr vio_pose_Msg = it->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
            if (vio_pose_Msg != nullptr)
            {
                // printf("%d %f\n", vio_pose_Msg->header.seq, vio_pose_Msg->header.stamp.toSec());
                // getchar();

                poses_cov_buffer.push_back(*vio_pose_Msg);

                // Append to our pose vector
                geometry_msgs::PoseStamped posetemp;
                posetemp.header = vio_pose_Msg->header;
                posetemp.pose = vio_pose_Msg->pose.pose;
                poses_imu.push_back(posetemp);

                // Create our path (imu)
                // NOTE: We downsample the number of poses as needed to prevent rviz crashes
                // NOTE: https://github.com/ros-visualization/rviz/issues/1107
                nav_msgs::Path arrIMU;
                arrIMU.header.stamp = vio_pose_Msg->header.stamp;
                arrIMU.header.seq = vio_pose_Msg->header.seq;
                arrIMU.header.frame_id = "global";
                for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
                    arrIMU.poses.push_back(poses_imu.at(i));
                }
                pub_pathimu.publish(arrIMU);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }        
    } 
    bag_vio_traj.close();

    for (int i = 1; i < poses_imu.size(); i++)
    {
        double dt = poses_imu.at(i).header.stamp.toSec() - poses_imu.at(i-1).header.stamp.toSec();
        if (dt < 0 || dt > 0.06)
        {
            printf("poses_imu dt err %f\n", dt);
            getchar();
        }
    }

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

    double shift_time = gt1_states.begin()->first - gt2_states.begin()->first;
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
            if (fabs(t1 - t2) < 0.001)
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
    printf("relative_position_buffer %d %f %f\n", relative_position_buffer.size(), relative_position_buffer.begin()->header.stamp.toSec(), relative_position_buffer.rbegin()->header.stamp.toSec());
    printf("max_dis %f\n", max_dis);

    for (int i = 1; i < relative_position_buffer.size(); i++)
    {
        double dt = relative_position_buffer.at(i).header.stamp.toSec() - relative_position_buffer.at(i-1).header.stamp.toSec();
        if (dt < 0 || dt > 0.01)
        {
            printf("relative_position_buffer dt err %f\n", dt);
            getchar();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::ofstream outFile_pose;
    outFile_pose.open("/home/junlin/GNSS/eval/stamped_traj_estimate.txt");

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
    //         if (fabs(t1 - t2) < 0.001)
    //         {
    //             Eigen::Matrix<double, 3, 1> t_I2_I1;
    //             t_I2_I1(0) = y.pose.position.x;
    //             t_I2_I1(1) = y.pose.position.y;
    //             t_I2_I1(2) = y.pose.position.z;
    //             Eigen::Matrix<double, 3, 1> t_I2_W = t_I1_W + R_I1_W.transpose() * t_I2_I1;

    //             double time_stamp = t2;
    //             outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
    //                             << t_I2_W(0) << " " << t_I2_W(1) << " " << t_I2_W(2) << " "
    //                             << std::endl;
                
    //             break;
    //         }
    //     }
    // }

    std::vector<geometry_msgs::PoseWithCovarianceStamped> position_buffer1;
    std::vector<geometry_msgs::PointStamped> position_buffer2;
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

        // double time_stamp = x.header.stamp.toSec();
        // outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
        //                 << x.pose.pose.position.x << " " << x.pose.pose.position.y << " " << x.pose.pose.position.z << " "
        //                 << std::endl;

        for (auto const& y : relative_position_buffer)
        {
            double t2 = y.header.stamp.toSec();
            if (fabs(t1 - t2) < 0.01)
            {
                // Eigen::Matrix<double, 3, 1> t_I2_I1;
                // t_I2_I1(0, 0) = y.point.x;
                // t_I2_I1(1, 0) = y.point.y;
                // t_I2_I1(2, 0) = y.point.z;

                // Eigen::Matrix<double, 3, 1> t_I2_W = t_I1_W + R_I1_W.transpose() * t_I2_I1;

                // Eigen::Matrix<double, 6, 6> covariance_vio_pose = Eigen::Matrix<double, 6, 6>::Zero();
                // for (int r = 0; r < 6; r++) {
                //     for (int c = 0; c < 6; c++) {
                //         covariance_vio_pose(r, c) = x.pose.covariance[6 * r + c];
                //     }
                // }

                // Eigen::Matrix<double, 3, 6> Jacob = Eigen::Matrix<double, 3, 6>::Zero();
                // Jacob.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
                // Jacob.block(0, 3, 3, 3) = -R_I1_W.transpose() * ov_core::skew_x(t_I2_I1);

                // Eigen::Matrix<double, 6, 6> covariance_position = Eigen::Matrix<double, 6, 6>::Zero();
                // covariance_position.block(0, 0, 3, 3) = Jacob * covariance_vio_pose * Jacob.transpose();

                // geometry_msgs::PoseWithCovarianceStamped poseIinM;
                // poseIinM.header.stamp = y.header.stamp;
                // poseIinM.pose.pose.orientation.x = 0;
                // poseIinM.pose.pose.orientation.y = 0;
                // poseIinM.pose.pose.orientation.z = 0;
                // poseIinM.pose.pose.orientation.w = 0;
                // poseIinM.pose.pose.position.x = t_I2_W(0);
                // poseIinM.pose.pose.position.y = t_I2_W(1);
                // poseIinM.pose.pose.position.z = t_I2_W(2);

                // // Finally set the covariance in the message (in the order position then orientation as per ros convention)
                // for (int r = 0; r < 6; r++) {
                //     for (int c = 0; c < 6; c++) {
                //     poseIinM.pose.covariance[6 * r + c] = covariance_position(r, c);
                //     }
                // }
                // position_buffer.push_back(poseIinM);

                // double time_stamp = poseIinM.header.stamp.toSec();
                // outFile_pose << std::fixed << std::setprecision(6) << time_stamp << " "
                //                 << poseIinM.pose.pose.position.x << " " << poseIinM.pose.pose.position.y << " " << poseIinM.pose.pose.position.z << " "
                //                 // << poseIinM.pose.pose.orientation.x << " " << poseIinM.pose.pose.orientation.y << " " << poseIinM.pose.pose.orientation.z << " " << poseIinM.pose.pose.orientation.w << " "
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
        if (dt < 0 || dt > 0.06)
        {
            printf("position_buffer dt err %f\n", dt);
            getchar();
        }
    }

    printf("position_buffer %d %f %f\n", position_buffer1.size(), position_buffer1.begin()->header.stamp.toSec(), position_buffer1.rbegin()->header.stamp.toSec());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Location of the ROS bag we want to read in
    std::string path_to_bag2;
    nh->param<std::string>("path_bag", path_to_bag2, "");
    PRINT_DEBUG("ros bag2 path is: %s\n", path_to_bag2.c_str());
    
    rosbag::Bag bag2;
    bag2.open(path_to_bag2);  // BagMode is Read by default
    std::string imu_topic_name = "/imu0";
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

    ros::spin();
    return 0;
}