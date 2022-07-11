#include "dynamic_estimator.h"

////////////////////  MAIN FUNCTIONS  ////////////////////

void DynamicEstimator::onInit(const ros::NodeHandle &nh)
{
  // initialize often used variables as consts
  consts_.zero3 = mat3::Zero();
  consts_.eye3 = mat3::Identity();
  consts_.eye_NST = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity();

  consts_.e_z << 0.0f,0.0f,1.0f;
  consts_.g = 9.807f;

  init_vars_.P0 = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Zero();

  // read flag to see whether to use EKF or UKF
  int temp_method;
  nh.param<int>("useMethod", temp_method, 1.0);
  useMethod_ = temp_method;

  // imu noise covariance
  double gyro_noise = 1.0e-3;
  double gyro_bias_noise = 1.0e-4;
  double acc_noise = 1.0e-2;
  double acc_bias_noise = 1.0e-3;
  nh.param("imu/gyroscope_noise_density",      gyro_noise, 1.0e-3);
  nh.param("imu/gyroscope_random_walk",      gyro_bias_noise, 1.0e-4);
  nh.param("imu/accelerometer_noise_density",      acc_noise, 1.0e-2);
  nh.param("imu/accelerometer_random_walk",      acc_bias_noise, 1.0e-3);
  consts_.Qx = Eigen::Matrix<flt, Qx_Dim, Qx_Dim>::Zero();
  consts_.Qx.block<3, 3>(0, 0) = consts_.eye3*std::pow(gyro_noise, 2);
  consts_.Qx.block<3, 3>(3, 3) = consts_.eye3*std::pow(gyro_bias_noise, 2);
  consts_.Qx.block<3, 3>(6, 6) = consts_.eye3*std::pow(acc_noise, 2);
  consts_.Qx.block<3, 3>(9, 9) = consts_.eye3*std::pow(acc_bias_noise, 2);

  // init_vars_.q   = quat::Identity();
  init_vars_.q.w() = 0.591301;
  init_vars_.q.x() = 0.021611;
  init_vars_.q.y() = -0.805788;
  init_vars_.q.z() = 0.024535;
  init_vars_.q.inverse();
  init_vars_.bg = vec3::Zero();
  init_vars_.v = vec3::Zero();
  init_vars_.ba = vec3::Zero();
  
  init_vars_.P0.block(bg_index, bg_index, bg_dim, bg_dim) = 1.0e-4 * consts_.eye3;
  init_vars_.P0.block(v_index, v_index, v_dim, v_dim) = 1.0e-4 * consts_.eye3;
  init_vars_.P0.block(ba_index, ba_index, ba_dim, ba_dim) = 1.0e-4 * consts_.eye3;

  // read UKF tuning parameters
  double temp;
  nh.param("UKF/alpha", temp, 0.4);
  consts_.alpha = flt(temp);
  nh.param("UKF/beta", temp, 2.0);
  consts_.beta = flt(temp);
  nh.param("UKF/kappa", temp, 0.0);
  consts_.kappa = flt(temp);

  // calculate the lambda and weights for unscented transform
  consts_.lambda = consts_.alpha*consts_.alpha*(NUM_STATES_TANGENT+consts_.kappa) - (NUM_STATES_TANGENT);
  consts_.Wm << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*((NUM_STATES_TANGENT) + consts_.lambda)));
  consts_.Wc << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda) + (1-consts_.alpha*consts_.alpha+consts_.beta), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*(NUM_STATES_TANGENT + consts_.lambda)));
  consts_.lambda_q = consts_.alpha*consts_.alpha*(Qx_Dim+consts_.kappa) - (Qx_Dim);
  consts_.Wmq << consts_.lambda_q/(Qx_Dim + consts_.lambda_q), Eigen::Matrix<flt,2*(Qx_Dim),1>::Constant(2*(Qx_Dim),1,1.0/(2.0*((Qx_Dim) + consts_.lambda_q)));
  consts_.Wcq << consts_.lambda_q/(Qx_Dim + consts_.lambda_q) + (1-consts_.alpha*consts_.alpha+consts_.beta), Eigen::Matrix<flt,2*(Qx_Dim),1>::Constant(2*(Qx_Dim),1,1.0/(2.0*(Qx_Dim + consts_.lambda_q)));

}

void DynamicEstimator::Init()
{
  // initialize often used variables as consts
  consts_.zero3 = mat3::Zero();
  consts_.eye3 = mat3::Identity();
  consts_.eye_NST = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity();

  consts_.e_z << 0.0f,0.0f,1.0f;
  consts_.g = 9.807f;

  init_vars_.P0 = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Zero();

  // imu noise covariance
  // euroc
  // flt gyro_noise = 1.0e-3;
  // flt gyro_bias_noise = 1.0e-4;
  // flt acc_noise = 1.0e-2;
  // flt acc_bias_noise = 1.0e-3;

  // lab
  flt gyro_noise = 0.1;
  flt gyro_bias_noise = 0.01;
  flt acc_noise = 10.0;
  flt acc_bias_noise = 0.01;

  consts_.Qx = Eigen::Matrix<flt, Qx_Dim, Qx_Dim>::Zero();
  consts_.Qx.block<3, 3>(0, 0) = consts_.eye3*std::pow(gyro_noise, 2);
  consts_.Qx.block<3, 3>(3, 3) = consts_.eye3*std::pow(gyro_bias_noise, 2);
  consts_.Qx.block<3, 3>(6, 6) = consts_.eye3*std::pow(acc_noise, 2);
  consts_.Qx.block<3, 3>(9, 9) = consts_.eye3*std::pow(acc_bias_noise, 2);

  // init_vars_.q   = quat::Identity();

  // euroc
  // init_vars_.q.w() = 0.591301;
  // init_vars_.q.x() = 0.021611;
  // init_vars_.q.y() = -0.805788;
  // init_vars_.q.z() = 0.024535;

  // lab
  init_vars_.q.w() = 0.999836;
  init_vars_.q.x() = 0.00206999;
  init_vars_.q.y() = 0.00626433;
  init_vars_.q.z() = -0.0168877;

  init_vars_.q.inverse();
  init_vars_.bg = vec3::Zero();
  init_vars_.v = vec3::Zero();
  init_vars_.ba = vec3::Zero();
  
  init_vars_.P0.block(bg_index, bg_index, bg_dim, bg_dim) = 1.0e-4 * consts_.eye3;
  init_vars_.P0.block(v_index, v_index, v_dim, v_dim) = 1.0e-4 * consts_.eye3;
  init_vars_.P0.block(ba_index, ba_index, ba_dim, ba_dim) = 1.0e-4 * consts_.eye3;

  // set UKF tuning parameters
  consts_.alpha = 0.4;
  consts_.beta = 2.0;
  consts_.kappa = 0.0;

  // calculate the lambda and weights for unscented transform
  consts_.lambda = consts_.alpha*consts_.alpha*(NUM_STATES_TANGENT+consts_.kappa) - (NUM_STATES_TANGENT);
  consts_.Wm << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*((NUM_STATES_TANGENT) + consts_.lambda)));
  consts_.Wc << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda) + (1-consts_.alpha*consts_.alpha+consts_.beta), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*(NUM_STATES_TANGENT + consts_.lambda)));
  consts_.lambda_q = consts_.alpha*consts_.alpha*(Qx_Dim+consts_.kappa) - (Qx_Dim);
  consts_.Wmq << consts_.lambda_q/(Qx_Dim + consts_.lambda_q), Eigen::Matrix<flt,2*(Qx_Dim),1>::Constant(2*(Qx_Dim),1,1.0/(2.0*((Qx_Dim) + consts_.lambda_q)));
  consts_.Wcq << consts_.lambda_q/(Qx_Dim + consts_.lambda_q) + (1-consts_.alpha*consts_.alpha+consts_.beta), Eigen::Matrix<flt,2*(Qx_Dim),1>::Constant(2*(Qx_Dim),1,1.0/(2.0*(Qx_Dim + consts_.lambda_q)));

}

void DynamicEstimator::initializeFilter(const InitVars &init)
{
  // set state value, state covariance values and time stamp
  state_.X.q = init.q;
  state_.X.bg = init.bg;
  state_.X.v = init.v;
  state_.X.ba = init.ba;
  state_.X.r = init.r;

  state_.t = init.t;

  state_.P = init.P0;
  
  state_.id = 0;
  state_.clone_states.clear();

  init_vars_ = InitVars();
  initialized_ = true;

  publishEstimates(state_);

  // print("filter initialized");
}

void DynamicEstimator::resetFilter()
{
  init_vars_ = InitVars();
  initialized_ = false;
  warn("reset",int(init_vars_.inputInitialized || init_vars_.rInitialized));
}

void DynamicEstimator::stateAugmentation(MeasPose &meas_pose) {

  state_.clone_states[state_.id] = Clone_State();
  Clone_State& clone_state = state_.clone_states[state_.id];

  clone_state.t = state_.t;
  clone_state.r = state_.X.r;

  clone_state.meas = meas_pose;
  state_.id++;

  Eigen::Matrix<flt, Aug_Dim, NUM_STATES_TANGENT> J = Eigen::Matrix<flt, Aug_Dim, NUM_STATES_TANGENT>::Zero();
  J.block<Aug_Dim, Aug_Dim>(0, r_index) = Eigen::Matrix<flt, Aug_Dim, Aug_Dim>::Identity();

  // Resize the state covariance matrix.
  size_t old_rows = state_.P.rows();
  size_t old_cols = state_.P.cols();
  state_.P.conservativeResize(old_rows+Aug_Dim, old_cols+Aug_Dim);

  // Rename some matrix blocks for convenience.
  const Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT>& P11 =
    state_.P.block<NUM_STATES_TANGENT, NUM_STATES_TANGENT>(0, 0);
  const Eigen::MatrixXf& P12 =
    state_.P.block(0, NUM_STATES_TANGENT, NUM_STATES_TANGENT, old_cols-NUM_STATES_TANGENT);

  // Fill in the augmented state covariance.
  state_.P.block(old_rows, 0, Aug_Dim, old_cols) << J*P11, J*P12;
  state_.P.block(0, old_cols, old_rows, Aug_Dim) =
    state_.P.block(old_rows, 0, Aug_Dim, old_cols).transpose();
  state_.P.block<Aug_Dim, Aug_Dim>(old_rows, old_cols) =
    J * P11 * J.transpose();

  // Fix the covariance to be symmetric
  Eigen::MatrixXf state_cov_fixed = (state_.P +
      state_.P.transpose()) / 2.0;
  state_.P = state_cov_fixed;

  return;
}

void DynamicEstimator::pruneCloneStateBuffer() {

  if (state_.clone_states.size() < 5)
    return;

  // Find clone states to be removed.
  vector<long long int> rm_clone_state_ids(0);
  auto first_clone_state_iter = state_.clone_states.begin();
  rm_clone_state_ids.push_back(first_clone_state_iter->first);

  for (const auto& id : rm_clone_state_ids) {
    int clone_sequence = std::distance(state_.clone_states.begin(),
        state_.clone_states.find(id));
    int clone_state_start = NUM_STATES_TANGENT + Aug_Dim*clone_sequence;
    int clone_state_end = clone_state_start + Aug_Dim;

    // Remove the corresponding rows and columns in the state
    // covariance matrix.
    if (clone_state_end < state_.P.rows()) {
      state_.P.block(clone_state_start, 0,
          state_.P.rows()-clone_state_end,
          state_.P.cols()) =
        state_.P.block(clone_state_end, 0,
            state_.P.rows()-clone_state_end,
            state_.P.cols());

      state_.P.block(0, clone_state_start,
          state_.P.rows(),
          state_.P.cols()-clone_state_end) =
        state_.P.block(0, clone_state_end,
            state_.P.rows(),
            state_.P.cols()-clone_state_end);

      state_.P.conservativeResize(
          state_.P.rows()-Aug_Dim, state_.P.cols()-Aug_Dim);
    } else {
      state_.P.conservativeResize(
          state_.P.rows()-Aug_Dim, state_.P.cols()-Aug_Dim);
    }

    // Remove this clone state in the state vector.
    state_.clone_states.erase(id);
  }

  return;
}

void DynamicEstimator::predictEKF(StateWithCov &state, Input &input)
{
  // calculate time diff to predicted state
  flt dt = input.t.toSec()-state.t.toSec();
  if (dt > 1.0e-5)
  {
    // Remove the bias from the measured gyro and acceleration
    vec3 gyro = input.w - state.X.bg;
    vec3 acc = input.a - state.X.ba;
    
    mat3 R = state.X.q.toRotationMatrix();
    mat3 skew_gyro = skewSymmetric(gyro);
    mat3 skew_acc = skewSymmetric(acc);

    // Compute discrete transition and noise covariance matrix
    Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT> F = Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT>::Zero();
    Eigen::Matrix<flt, NUM_STATES_TANGENT, Qx_Dim> G = Eigen::Matrix<flt, NUM_STATES_TANGENT, Qx_Dim>::Zero();

    F.block<3, 3>(0, 0) = -skew_gyro;
    F.block<3, 3>(0, 3) = -consts_.eye3;
    F.block<3, 3>(6, 0) = -R * skew_acc;
    F.block<3, 3>(6, 9) = -R;
    F.block<3, 3>(12, 6) = consts_.eye3;

    G.block<3, 3>(0, 0) = -consts_.eye3;
    G.block<3, 3>(3, 3) = consts_.eye3;
    G.block<3, 3>(6, 6) = -R;
    G.block<3, 3>(9, 9) = consts_.eye3;

    // Approximate matrix exponential to the 3rd order,
    // which can be considered to be accurate enough assuming
    // dtime is within 0.01s.
    Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT> Fdt = F * dt;
    Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT> Fdt_square = Fdt * Fdt;
    Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT> Fdt_cube = Fdt_square * Fdt;
    Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT> Phi = Eigen::Matrix<flt, NUM_STATES_TANGENT, NUM_STATES_TANGENT>::Identity() + Fdt + 0.5*Fdt_square + (1.0/6.0)*Fdt_cube;

    // update time stamp
    state.t = input.t;
    // predicted mean state with non-linear model
    processModel(state.X,input,dt);
    // predicted state covariance with linearized model
    // state.P = F*state.P*F.transpose() + consts_.Qx*dt;

    // Propogate the state covariance matrix.
    state_.P.block<NUM_STATES_TANGENT, NUM_STATES_TANGENT>(0, 0) =
    Phi*state_.P.block<NUM_STATES_TANGENT, NUM_STATES_TANGENT>(0, 0)*Phi.transpose() + Phi*G*consts_.Qx*G.transpose()*Phi.transpose()*dt;

    if (state_.clone_states.size() > 0) {
      state_.P.block(
          0, NUM_STATES_TANGENT, NUM_STATES_TANGENT, state_.P.cols()-NUM_STATES_TANGENT) =
        Phi * state_.P.block(
          0, NUM_STATES_TANGENT, NUM_STATES_TANGENT, state_.P.cols()-NUM_STATES_TANGENT);
      state_.P.block(
          NUM_STATES_TANGENT, 0, state_.P.rows()-NUM_STATES_TANGENT, NUM_STATES_TANGENT) =
        state_.P.block(
          NUM_STATES_TANGENT, 0, state_.P.rows()-NUM_STATES_TANGENT, NUM_STATES_TANGENT) * Phi.transpose();
    }

    Eigen::MatrixXf state_cov_fixed = (state_.P +
        state_.P.transpose()) / 2.0;
    state_.P = state_cov_fixed;
  }
}

void DynamicEstimator::predictUKF(StateWithCov &state, Input &input)
{
  // calculate time diff to predicted state
  flt dt = input.t.toSec()-state.t.toSec();

  if (dt > 1.0e-5) {
    Eigen::Matrix<flt,Qx_Dim,NUM_SIGMAS_Q> sigmaQ;
    getSigmaQ(sigmaQ);
    State sigmaPoints_Q[NUM_SIGMAS_Q];
    for (int i = 0; i < NUM_SIGMAS_Q; i++)
    {
      sigmaPoints_Q[i] = state.X;
      Eigen::Matrix<flt,Qx_Dim,1> n = sigmaQ.col(i);
      processModel_noise(sigmaPoints_Q[i],input,n,dt);
    }

    // calculate the noise covariance
    Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_SIGMAS_Q-1> diff_Q;
    for (int i=0; i<NUM_SIGMAS_Q-1; i++) {
      // diff_Q.col(i) = sigmaPoints_Q[i+1].boxminus(sigmaPoints_Q[0]);
      diff_Q.col(i) = sigmaPoints_Q[i+1].boxminus(state.X);
    }


    // create and propagate sigma points
    State sigmaPoints[NUM_SIGMAS];
    getSigmaPoints(sigmaPoints,state);
    for (int i = 0; i<NUM_SIGMAS; i++)
      processModel(sigmaPoints[i],input,dt);
  
    // calculate the mean    
    calculateMeanStateSigma(state.X, sigmaPoints);
  
    // calculate the covariance
    Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_SIGMAS> diff;
    for (int i=0; i<NUM_SIGMAS; i++) {
      diff.col(i) = sigmaPoints[i].boxminus(state.X);
    }

    state.P = diff*consts_.Wc.asDiagonal()*diff.transpose() + diff_Q*consts_.Wcq.tail(NUM_SIGMAS_Q-1).asDiagonal()*diff_Q.transpose();

    // update time stamp and
    state.t = input.t;
  }
}

void DynamicEstimator::measUpdatePoseEKF(StateWithCov &state, MeasPose &meas)
{
  // allocate matrices
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H.block(0,r_index,3,r_dim) = consts_.eye3;
  // innovation
  Eigen::Matrix<flt,3,1> Xhat;
  Xhat.block(0,0,3,1) = meas.r-state.X.r;
  // innovation covariance
  Eigen::Matrix<flt,3,3> S_KF = meas.cov.block(0,0,3,3) + H*state.P*H.transpose();
  // optimal kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H.transpose() * S_KF.inverse();
  // updated state estimate and state covariance estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;
  state.X.boxplus(dX);
  state.P = (consts_.eye_NST-K_KF*H)*state.P*(consts_.eye_NST-K_KF*H).transpose() + K_KF*meas.cov.block(0,0,3,3)*K_KF.transpose();
}

void DynamicEstimator::measUpdatePoseMSCKF() {

  int window_size = state_.clone_states.size();

  auto& P = state_.P;
  int D = Aug_Dim * window_size;
  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(D, P.cols());
  Eigen::VectorXf r = Eigen::VectorXf::Zero(D);
  Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(D, D);
  int stack_cntr = 0;
  auto clone_state_iter = state_.clone_states.begin();
  for (int i = 0; i < window_size; ++i, ++clone_state_iter)
  {
      Eigen::MatrixXf H_xj = Eigen::MatrixXf::Zero(Aug_Dim, P.cols());
      Eigen::VectorXf r_j = Eigen::VectorXf::Zero(Aug_Dim);
      Eigen::MatrixXf Q_j = Eigen::MatrixXf::Zero(Aug_Dim, Aug_Dim);

      MeasPose meas = clone_state_iter->second.meas;
      r_j.block(0,0,3,1) = meas.r-clone_state_iter->second.r;

      Q_j = meas.cov.block(0,0,3,3);

      H_xj.block(0, NUM_STATES_TANGENT + i*Aug_Dim, Aug_Dim, Aug_Dim) = Eigen::Matrix<flt, Aug_Dim, Aug_Dim>::Identity();

      H.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
      r.segment(stack_cntr, r_j.rows()) = r_j;
      Q.block(stack_cntr, stack_cntr, Q_j.rows(), Q_j.cols()) = Q_j;
      stack_cntr += H_xj.rows();
  }

  H.conservativeResize(stack_cntr, H.cols());
  r.conservativeResize(stack_cntr);
  Q.conservativeResize(stack_cntr, stack_cntr);

  // Compute the Kalman gain.
  Eigen::MatrixXf S = H * P * H.transpose() + Q;
  Eigen::MatrixXf K_transpose = S.ldlt().solve(H * P);
  Eigen::MatrixXf K = K_transpose.transpose();

  // Compute the error of the state.
  Eigen::VectorXf delta_x = K * r;

  // Update the head state.
  const Eigen::VectorXf& delta_x_head = delta_x.head<NUM_STATES_TANGENT>();
  state_.X.boxplus(delta_x_head);

  // Update the clone states.
  clone_state_iter = state_.clone_states.begin();
  for (int i = 0; i < state_.clone_states.size();
      ++i, ++clone_state_iter) {
    const Eigen::VectorXf& delta_x_clone = delta_x.segment<Aug_Dim>(NUM_STATES_TANGENT+i*Aug_Dim);

    clone_state_iter->second.r       += delta_x_clone.segment(0,3);
  }

  // Update state covariance.
  Eigen::MatrixXf I_KH = Eigen::MatrixXf::Identity(K.rows(), H.cols()) - K*H;
  state_.P = I_KH*state_.P;

  // Fix the covariance to be symmetric
  Eigen::MatrixXf state_cov_fixed = (state_.P +
      state_.P.transpose()) / 2.0;
  state_.P = state_cov_fixed;

  return;
}

void DynamicEstimator::measUpdatePoseUKF(StateWithCov &state, MeasPose &meas)
{
  // Unscented Kalman Filter update step for position measurement
  // allocating sigma points
  const int L = 2*(NUM_STATES_TANGENT)+1;
  State sigmaPoints[L];
  Eigen::Matrix<flt,3,L> propagatedSigmaPointsMat;
  // calculate sigma points
  getSigmaPoints(sigmaPoints,state);
  // sigmapoint transformation through non-linear measurement model
  for (int i = 0; i<L; i++)
  {
    propagatedSigmaPointsMat.block(0,i,3,1) = sigmaPoints[i].r;
  }
  // mean of transformed sigma points
  vec3 zhat = propagatedSigmaPointsMat*consts_.Wm;
  // innovation
  Eigen::Matrix<flt,3,L> diffZ;
  diffZ = propagatedSigmaPointsMat - zhat.replicate(1,L);
  // calculate innovation covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,L> diffX;
  for (int i=0; i<L; i++) {
    diffX.col(i) = sigmaPoints[i].boxminus(state.X);
  }
  mat3 S_UKF  = diffZ * consts_.Wc.asDiagonal() * diffZ.transpose() + meas.cov.block(0,0,3,3);
  // kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> Pcross = diffX * consts_.Wc.asDiagonal() * diffZ.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_UKF = Pcross*S_UKF.inverse();
  // updated covariance estimate (a posteriori)
  state.P = state.P - K_UKF*S_UKF*K_UKF.transpose();
  // updated estimate (a posteriori)
  Eigen::Matrix<flt,3,1> meas_r;
  meas_r.block(0,0,3,1) = meas.r;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_UKF*(meas_r - zhat);
  state.X.boxplus(del);
}

////////////////////  IMPLEMENTATION FUNCTIONS  ////////////////////

////////////////////  process and measurement models  ////////////////////

void DynamicEstimator::processModel(State &X, Input &U, flt &dt)
{
  vec3 gyro = U.w - X.bg;
  vec3 acc = U.a - X.ba;

  // calculate delta: delta = xdot*dt
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> delta;
  
  delta.setZero();
  // delta on SO(3)
  delta.segment(q_index,q_dim)  = gyro*dt;
  // delta on the Euclidean Space
  delta.segment(r_index,r_dim)  = X.v*dt;

  vec3 dv1 = X.q.toRotationMatrix() * acc;
  vec3 dv2 = consts_.g*consts_.e_z;
  vec3 dv = dv1 - dv2;
  delta.segment(v_index,v_dim)  = dv*dt;
  
  // add delta: x_new = x_old + delta
  X.boxplus(delta);
}

void DynamicEstimator::processModel_noise(State &X, Input &U, Eigen::Matrix<flt,Qx_Dim,1> &n, flt &dt)
{
  vec3 n_bg = n.segment(0, 3);
  vec3 n_bg_w = n.segment(3, 3);
  vec3 n_ba = n.segment(6, 3);
  vec3 n_ba_w = n.segment(9, 3);

  vec3 gyro = U.w - X.bg - n_bg;
  vec3 acc = U.a - X.ba - n_ba;

  // calculate delta: delta = xdot*dt
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> delta;
  
  delta.setZero();
  // delta on SO(3)
  delta.segment(q_index,q_dim)  = gyro*dt;
  // delta on the Euclidean Space
  delta.segment(r_index,r_dim)  = X.v*dt;

  vec3 dv1 = X.q.toRotationMatrix() * acc;
  vec3 dv2 = consts_.g*consts_.e_z;
  vec3 dv = dv1 - dv2;
  delta.segment(v_index,v_dim)  = dv*dt;

  delta.segment(ba_index,ba_dim)  = n_ba_w*dt;
  delta.segment(bg_index,bg_dim)  = n_bg_w*dt;
  
  // add delta: x_new = x_old + delta
  X.boxplus(delta);
}

////////////////////  UKF functions  ////////////////////

void DynamicEstimator::getSigmaQ(Eigen::Matrix<flt,Qx_Dim,NUM_SIGMAS_Q> &sigmaQ)
{
  // Cholesky factorization
  Eigen::LLT<Eigen::Matrix<flt,Qx_Dim,Qx_Dim>> factorization(consts_.Qx);
  Eigen::Matrix<flt,Qx_Dim,Qx_Dim> sqrt = factorization.matrixL();
  flt c = std::sqrt(consts_.lambda_q + Qx_Dim);
  sqrt *= c;

  // calculate sigma Q
  sigmaQ.setZero();
  for (int i = 1; i <= Qx_Dim; i++){
    sigmaQ.col(i) = sqrt.col(i-1);
    sigmaQ.col(i+Qx_Dim) = -sqrt.col(i-1);
  }
}

void DynamicEstimator::getSigmaPoints(State *sigmaPoints, StateWithCov &state)
{
  // Cholesky factorization
  Eigen::LLT<Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>> factorization(state.P);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> sqrt = factorization.matrixL();
  flt c = std::sqrt(consts_.lambda + NUM_STATES_TANGENT);
  sqrt *= c;

  // calculate sigma points
  for (int i = 0; i <= 2*NUM_STATES_TANGENT; i++)
    sigmaPoints[i] = state.X;
  for (int i = 1; i <= NUM_STATES_TANGENT; i++){
    sigmaPoints[i].boxplus(sqrt.col(i-1));
    sigmaPoints[i+NUM_STATES_TANGENT].boxplus(-sqrt.col(i-1));
  }
}

void DynamicEstimator::calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS])
{
  // function to calculate the mean of sigma points
  // calculate mean on SO(3)
  quat qList[NUM_SIGMAS];
  for (int i = 0; i < NUM_SIGMAS; i++)
    qList[i] = sigmaPoints[i].q;
  mean.q = qMean(qList,consts_.Wm);

  // calulate mean on Euclidean Space variables
  mean.bg      = consts_.Wm[0] * sigmaPoints[0].bg;
  mean.v       = consts_.Wm[0] * sigmaPoints[0].v;
  mean.ba      = consts_.Wm[0] * sigmaPoints[0].ba;
  mean.r       = consts_.Wm[0] * sigmaPoints[0].r;

  for (int i = 1; i < NUM_SIGMAS; i++)
  {
    mean.bg      += consts_.Wm[i] * sigmaPoints[i].bg;
    mean.v       += consts_.Wm[i] * sigmaPoints[i].v;
    mean.ba      += consts_.Wm[i] * sigmaPoints[i].ba;
    mean.r       += consts_.Wm[i] * sigmaPoints[i].r;
  }
}

////////////////////  message callback functions  ////////////////////

void DynamicEstimator::input_callback(Input &input_)
{
  if (initialized_)
  {
    // predict state with either EKF or UKF
    if (useMethod_ == 1 || useMethod_ == 2)
    {
      predictEKF(state_, input_);
    }
    else
    {
      predictUKF(state_, input_);
    }
    // publish updated estimates
    // publishEstimates(state_);
  }
  else
  {
    // if not initialized, save msg, set flag that pose initialized
    init_vars_.inputInitialized = true;
    init_vars_.t = input_.t;
    // check whether ready to initialize or not
    init_vars_.readyToInitialize = init_vars_.inputInitialized &&
                                   init_vars_.rInitialized;
    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void DynamicEstimator::r_callback(MeasPose &meas_pose)
{
  if (initialized_)
  {
    // update state with either EKF or UKF
    if (useMethod_ == 1)
    {
      measUpdatePoseEKF(state_, meas_pose);
    }
    else if (useMethod_ == 2)
    {
      stateAugmentation(meas_pose);
      measUpdatePoseMSCKF();
      pruneCloneStateBuffer();
    }
    else
    {
      measUpdatePoseUKF(state_, meas_pose);
    }  
    // publish updated estimates
    publishEstimates(state_);
  }
  else
  {
    // if not initialized, save msg, set flag that pose initialized
    init_vars_.r   = meas_pose.r;
    init_vars_.P0.block(r_index,r_index,r_dim,r_dim) = meas_pose.cov.block(0,0,3,3);
    init_vars_.rInitialized = true;
    // check whether ready to initialize or not
    init_vars_.readyToInitialize = init_vars_.inputInitialized &&
                                   init_vars_.rInitialized;
    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void DynamicEstimator::publishEstimates(const StateWithCov &estimate)
{
  // Append to our pose vector
  geometry_msgs::PoseStamped posetemp;
  posetemp.header.stamp = estimate.t;
  posetemp.pose.position.x = estimate.X.r(0);
  posetemp.pose.position.y = estimate.X.r(1);
  posetemp.pose.position.z = estimate.X.r(2);
  posetemp.pose.orientation.x = estimate.X.q.x();
  posetemp.pose.orientation.y = estimate.X.q.y();
  posetemp.pose.orientation.z = estimate.X.q.z();
  posetemp.pose.orientation.w = estimate.X.q.w();
  poses_imu.push_back(posetemp);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = posetemp.header.stamp;
  arrIMU.header.frame_id = "global";
  for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
      arrIMU.poses.push_back(poses_imu.at(i));
  }
  pub_pathimu.publish(arrIMU);

  double t = posetemp.header.stamp.toSec();
  // static double init_time = -1.0;
  // if (init_time < 0)
  // {
  //   init_time = posetemp.header.stamp.toSec();
  // }
  // double t = posetemp.header.stamp.toSec() - init_time;

  // timestamp
  outFile_pose.precision(5);
  outFile_pose.setf(std::ios::fixed, std::ios::floatfield);
  outFile_pose << t << " ";

  // save mass
  outFile_pose.precision(6);
  outFile_pose << estimate.X.r(0) << " " <<estimate.X.r(1) << " " <<estimate.X.r(2) << " "
          << estimate.X.q.x() << " " <<estimate.X.q.y() << " " << estimate.X.q.z() << " " << estimate.X.q.w() << " "
          << std::endl;
}

////////////////////  state addition/subraction functions  ////////////////////

void State::boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta)
{
  // adding a delta (element of the tangent space of the state space) to a state (element of the state space)
  // "addition" on SO(3)
  this->q       = qBoxPlus(this->q,delta.segment(q_index,q_dim));
  // addition in the Euclidean Space
  this->bg      += delta.segment(bg_index, bg_dim);
  this->v       += delta.segment(v_index,v_dim);
  this->ba      += delta.segment(ba_index, ba_dim);
  this->r       += delta.segment(r_index,r_dim);  
}

Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> State::boxminus(const State &other) const
{
  // calculating the delta (lying on the tangent space of the state space) of two state
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> delta;
  // delta on SO(3)
  delta.segment(q_index,q_dim)  = qBoxMinus(this->q,other.q);
  // delta on the Euclidean Space
  delta.segment(bg_index,bg_dim) = this->bg - other.bg;
  delta.segment(v_index,v_dim)  = this->v - other.v;
  delta.segment(ba_index,ba_dim) = this->ba - other.ba;
  delta.segment(r_index,r_dim)  = this->r - other.r;  
  return delta;
}
