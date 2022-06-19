#pragma once

#include "utility/utility.h"
#include "utility/tic_toc.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <nav_msgs/Odometry.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/p_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

#include <time.h>
#include <random>

using namespace std;

class Estimator
{
  public:
    // interface
    double current_time = -1;
    void process_input(vector<Input> collective_input);
    void processIMU(double dt, Input &input_);
    void processMeas(MeasPose &meas_pose);
    
    // internal
    void clearState();
    void slideWindow();
    void solveOdometry();
    void optimization_init(int frame_count);
    void optimization();
    void vector2double();
    void vector2double_init(int frame_count);
    void double2vector();
    void double2vector_init(int frame_count);
    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)]; 

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    MeasPose meas[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
   
    int frame_count;

    bool first_input;
    bool failure_occur;

    double initial_timestamp;

    double para_Position[WINDOW_SIZE + 1][SIZE_POSITION];
    double para_Attitude[WINDOW_SIZE + 1][SIZE_ATTITUDE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    double solve_time_ms;
};
