#include "v_tools.h"

void print(const string &s) {
  if (doPrint) {
    ROS_INFO(s.c_str());
  }
}

void print()
{
  static int i;
  i++;
  if (doPrint) {
    ROS_INFO("position: %d", i);
  }
}

void warn(const string &s) {
  if (1) {
    ROS_WARN(s.c_str());
  }
}

void warn(const char* ch) {
  if (1) {
    ROS_WARN(ch);
  }
}

template <typename T>
vec3 msg2vec(T const &t) {
  return vec3(t.x,t.y,t.z);
}

geometry_msgs::Vector3 vec2msg(vec3 const &t) {
  geometry_msgs::Vector3 msg;
  msg.x = t(0);
  msg.y = t(1);
  msg.z = t(2);
  return msg;
}

geometry_msgs::Vector3 ivec2msg(vec3 const &t) {
  geometry_msgs::Vector3 msg;
  msg.x = t(0);
  msg.y = -t(1);
  msg.z = -t(2);
  return msg;
}

quat msg2quat(const geometry_msgs::Quaternion &q) {
  return quat(q.w,q.x,q.y,q.z);
}

geometry_msgs::Quaternion quat2msg(quat const &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::Quaternion iquat2msg(quat const &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = -q.y();
  msg.z = -q.z();
  return msg;
}

quat qExp(const angleA &aA)
{
  quat q;
  flt phi(aA.norm());
  if (phi == 0)
    q = quat(1.0,0.0,0.0,0.0);
  else
  {
    vec3 u(aA.normalized());
    q.w() = cos(phi);
    q.x() = sin(phi)*u[0];
    q.y() = sin(phi)*u[1];
    q.z() = sin(phi)*u[2];
  }
  return q;
}

angleA qLog(const quat &q)
{
  vec3 qv(q.x(),q.y(),q.z());
  flt qvNorm = qv.norm();
  flt phi(atan2(qvNorm,q.w()));
  vec3 u;
  if (phi == 0)
    u = vec3(0.0,0.0,1.0);
  else if (phi < 1e-6)
    u = qv/q.w()*(1-qvNorm*qvNorm/(3*q.w()*q.w()));
  else
    u = qv.normalized();

  return angleA(phi*u);
}

void print(const string &s, const quat &q)
{
  if (doPrint) {
    stringstream ss;
    ss << s << "\n" << q.w() << "  " << q.x() << "  " << q.y() << "  " << q.z();
    ROS_INFO(ss.str().c_str());
  }
}

quat qBoxPlus(const quat &q, const angleA &delta)
{
  return quat(q*qExp(delta/2.0)).normalized();
}

angleA qBoxMinus(const quat &q1, const quat &q2)
{
  return angleA(2.0*qLog(q2.inverse()*q1));
}

quat qMean(const quat *qList, const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights)
{
  const int n = weights.rows();
  quat mu = qList[0];
  angleA error[n];
  int k = 1;
  angleA meanError(0.0,0.0,0.0);
  while (true) {
    meanError = angleA(0.0,0.0,0.0);
    for (int i = 0; i < n; i++)
    {
      error[i] = qBoxMinus(qList[i],mu);
    }
    for (int i = 0; i < n; i++)
    {
      meanError += error[i];
    }
    meanError = meanError/flt(n);
    mu = qBoxPlus(mu,meanError);
    if (k>2) {
      //print("mean quat calc too long, done steps",k);
      break;
    }
    if (meanError.norm() < numeric_limits<flt>::epsilon()*1.0e3)
      break;
    k++;
  }
  return mu;
}

vec4 qMean(const Eigen::Matrix<flt,4,Eigen::Dynamic> &qMatrix, const Eigen::Matrix<flt,Eigen::Dynamic,1> &weights)
{
  quat qList[qMatrix.cols()];
  for (int i=0; i<qMatrix.cols(); i++)
  {
    qList[i] = quat(qMatrix(0,i),qMatrix(1,i),qMatrix(2,i),qMatrix(3,i));
  }
  quat qMu = qMean(qList,weights);
  return vec4(qMu.w(),qMu.x(),qMu.y(),qMu.z());
}

vec3 qRotateVec(const quat &q, const vec3 &vec)
{
  quat qvec;
  qvec.x() = vec(0);
  qvec.y() = vec(1);
  qvec.z() = vec(2);
  qvec.w() = 0;
  qvec = q*qvec*q.inverse();
  return vec3(qvec.x(),qvec.y(),qvec.z());
}

Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& w) {
  Eigen::Matrix3f w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

Eigen::Matrix3d skewSymmetric_d(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
  Eigen::Matrix<double, 3, 3> q_x = skewSymmetric_d(q.block(0, 0, 3, 1));
  Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                        2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
  return Rot;
}

// // refer to "Quaternion kinematics for the error-state Kalman filter", but it doesn't work
// Eigen::Matrix<double,3,3> Jr(Eigen::Matrix<double,3,1> &theta)
// {
//   double theta_norm = theta.norm();
//   if (theta_norm < 1e-12) {
//     return Eigen::MatrixXd::Identity(3, 3);
//   } else {
//     double theta_norm_square = theta_norm * theta_norm;
//     double theta_norm_cube = theta_norm_square * theta_norm;
//     Eigen::Matrix<double, 3, 3> theta_skew = skewSymmetric_d(theta);
//     Eigen::Matrix<double, 3, 3> theta_skew_square = theta_skew * theta_skew;
//     Eigen::Matrix<double, 3, 3> J = Eigen::MatrixXd::Identity(3, 3) -
//                                     ((1 - cos(theta_norm)) / theta_norm_square) * theta_skew +
//                                     (theta_norm - sin(theta_norm) / theta_norm_cube) * theta_skew_square;
//     return J;
//   }
// }

// refer to open_vins, it works
Eigen::Matrix<double, 3, 3> Jl_so3(Eigen::Matrix<double, 3, 1> w) {
  double theta = w.norm();
  if (theta < 1e-12) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) + (1 - sin(theta) / theta) * a * a.transpose() +
                                    ((1 - cos(theta)) / theta) * skewSymmetric_d(a);
    return J;
  }
}

Eigen::Matrix<double, 3, 3> Jr_so3(Eigen::Matrix<double, 3, 1> w) { return Jl_so3(-w); }

vec3 R2ypr(const mat3 &R)
{
    vec3 n = R.col(0);
    vec3 o = R.col(1);
    vec3 a = R.col(2);

    vec3 ypr(3);
    flt y = atan2(n(1), n(0));
    flt p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    flt r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

mat3 ypr2R(const vec3 &ypr)
{
    flt y = ypr(0) / 180.0 * M_PI;
    flt p = ypr(1) / 180.0 * M_PI;
    flt r = ypr(2) / 180.0 * M_PI;

    mat3 Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    mat3 Ry;
    Ry << cos(p), 0., sin(p),
        0., 1., 0.,
        -sin(p), 0., cos(p);

    mat3 Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    return Rz * Ry * Rx;
}
