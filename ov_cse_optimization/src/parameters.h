#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#define MARG 0

struct Input
{
  // inputs for prediction step
  Eigen::Matrix<double,3,1> a;
  Eigen::Matrix<double,3,1> w;
  double t;
};

struct MeasPose
{
  // measurements for pose measurement update
  Eigen::Vector3d r;
  Eigen::Matrix<double,3,3> cov;
  double t;
};

// const int WINDOW_SIZE = 10;
const int WINDOW_SIZE = 5;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;

void readParameters(ros::NodeHandle &n);
void readParameters();

enum SIZE_PARAMETERIZATION
{
    SIZE_POSITION = 3,
    SIZE_ATTITUDE = 4, // quaternion
    SIZE_SPEEDBIAS = 9
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};