#ifndef DynamicEstimator_H
#define DynamicEstimator_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <Eigen/Cholesky>
#include <strstream>
#include <string.h>
#include <vector>
#include "v_tools.h"
#include <math.h>
#include <cmath>
#include <string.h>
#include <queue>
#include <time.h>
#include <fstream>

using namespace std;

// define state & measurement sizes
#define NUM_STATES         int(16)
#define NUM_STATES_TANGENT int(15)
#define Qx_Dim             int(12)
#define Aug_Dim            int(3)
#define NUM_SIGMAS         int(2*(NUM_STATES_TANGENT)+1)
#define NUM_SIGMAS_Q       int(2*(Qx_Dim)+1)
#define RT_PI  3.14159265358979323846
#define RT_PIF 3.1415927F

int q_index = 0;
int q_dim = 3;
int bg_index = 3;
int bg_dim = 3;
int v_index = 6;
int v_dim = 3;
int ba_index = 9;
int ba_dim = 3;
int r_index = 12;
int r_dim = 3;

struct Input
{
  // inputs for prediction step
  vec3 a;
  vec3 w;
  ros::Time t;
  // flt t;
};

struct MeasPose
{
  // measurements for pose measurement update
  vec3 r;
  mat3 cov;
};

struct MeasTargetPose
{
  // measurements for pose measurement update
  vec3 r;
  quat q;
  mat6 cov;
};

// structs/classes to cleanly save sates, inputs, measurements, and initialization values
struct InitVars
{
  // variables needed for filter initialization
  // flags
  bool readyToInitialize = false;
  bool rInitialized = false;
  bool qrInitialized = false;
  bool inputInitialized = false;
  // time
  ros::Time t;
  // flt t;
  // state variables
  quat q;
  vec3 bg;
  vec3 v;
  vec3 ba;
  vec3 r;  
  
  // uncertainty covariance
  // Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P0;
  Eigen::MatrixXf P0;
};

class State
{
public:
  // state variables
  quat q;
  vec3 bg;
  vec3 v;
  vec3 ba;
  vec3 r;

  // state manifold operations to "add" & "subtract" states
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> boxminus(const State& other) const;
  void boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta);
};

class Clone_State
{
public:
  // state variables
  vec3 r;

  MeasPose meas;

  ros::Time t;
  // flt t;
};

struct StateWithCov
{
  // complete state with covariance and timestamp
  State X;
  long long int id;
  std::map<long long int, Clone_State> clone_states;
  // Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P;
  Eigen::MatrixXf P;
  ros::Time t;
  // flt t;
};

struct Consts
{
  // often used constants

  // process model covariance
  Eigen::Matrix<flt,Qx_Dim,Qx_Dim> Qx;

  // UKF tuning parameters
  flt alpha, kappa, beta;
  flt lambda, lambda_q;
  // UKF weight matrices
  Eigen::Matrix<flt,NUM_SIGMAS,1> Wm,Wc;
  Eigen::Matrix<flt,NUM_SIGMAS_Q,1> Wmq, Wcq;

  // generally often used constants
  vec3 e_z;
  flt g;
  // matrices to speed up calculation
  mat3 zero3;
  mat3 eye3;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> eye_NST;
};


// estimator class
class DynamicEstimator
{
public:
  // node initialization function
  void onInit(const ros::NodeHandle &nh);
  void Init();
  void resetFilter();

  std::string file;
  // subscribers and publisher
  ros::Subscriber sub_imu_,
                  sub_pos_;
  ros::Publisher pub_pathimu;
  std::vector<geometry_msgs::PoseStamped> poses_imu;
  void input_callback(Input &input_);
  void r_callback(MeasPose &meas_pose);
  void qr_callback(MeasTargetPose &meas_pose);

  // member variables
  Consts consts_;
  InitVars init_vars_;
  StateWithCov state_;
  Input input_;

  // status flags
  bool initialized_ = false;
  int useMethod_ = 3;

  std::ofstream outFile_pose;
  
private:
  // estimator member function to initialize, predict and update
  void initializeFilter(const InitVars& init);

  // models
  void processModel(State& X, Input& U, flt &dt);
  void processModel_noise(State &X, Input &U, Eigen::Matrix<flt,Qx_Dim,1> &n, flt &dt);

  void predictEKF(StateWithCov& oldState,Input& input);
  void measUpdatePoseEKF(StateWithCov &oldState, MeasPose &meas);
  void measUpdateTargetPoseEKF(StateWithCov &oldState, MeasTargetPose &meas);
  void stateAugmentation(MeasPose &meas_pose);
  void measUpdatePoseMSCKF();
  void pruneCloneStateBuffer();

  // UKF functions
  void getSigmaQ(Eigen::Matrix<flt,Qx_Dim,NUM_SIGMAS_Q> &sigmaQ);
  void getSigmaPoints(State *sigmaPoints, StateWithCov &state);
  void calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS]);
  void predictUKF(StateWithCov &state, Input &input);
  void measUpdatePoseUKF(StateWithCov &state, MeasPose &meas);

  // publisher function
  void publishEstimates(const StateWithCov &estimate);
};

#endif // DynamicEstimator_H
