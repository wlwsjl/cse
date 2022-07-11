#include "parameters.h"

double ACC_N, ACC_W;
double GYR_N, GYR_W;

Eigen::Vector3d G{0.0, 0.0, 9.81};

double SOLVER_TIME;
int NUM_ITERATIONS;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    
    fsSettings.release();
}

void readParameters()
{
    SOLVER_TIME = 0.02;
    NUM_ITERATIONS = 5; 

    // euroc
    // ACC_N = 1.0e-1;
    // ACC_W = 1.0e-1;
    // GYR_N = 1.0e-2;
    // GYR_W = 1.0e-2;

    // lab
    ACC_N = 10.0;
    ACC_W = 0.01;
    GYR_N = 0.1;
    GYR_W = 0.01;
}
