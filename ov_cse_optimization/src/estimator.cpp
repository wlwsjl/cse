#include "estimator.h"

void Estimator::clearState()
{
    current_time = -1;
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    solver_flag = INITIAL;
    first_input = false,
    frame_count = 0;
    initial_timestamp = 0;

    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    failure_occur = 0;
}

void Estimator::process_input(vector<Input> collective_input)
{
    for (int i = 0; i < collective_input.size(); i++)
    {
        Input cur_input = collective_input.at(i);
        double t = cur_input.t;

        if (current_time < 0)
                current_time = t;

        double dt = t - current_time;
        ROS_ASSERT(dt >= 0);
        current_time = t;

        processIMU(dt, cur_input);
    }
}

void Estimator::processIMU(double dt, Input &input_)
{
    Vector3d linear_acceleration = input_.a;
    Vector3d angular_velocity = input_.w;

    if (!first_input)
    {
        first_input = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]}; 
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // propagate state with Input measurements
        if(solver_flag == NON_LINEAR)
        {
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - G;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - G;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
        }
    }

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

void Estimator::processMeas(MeasPose &meas_pose)
{
    marginalization_flag = MARGIN_OLD;

    std_msgs::Header header;
    header.stamp = ros::Time().fromSec(meas_pose.t);
    Headers[frame_count] = header;

    meas[frame_count] = meas_pose;

    if (solver_flag == INITIAL)
    {
        // use meas_pose to initialize
        Ps[frame_count] = meas_pose.r;
        Quaterniond q;
        q.w() = 0.591301;
        q.x() = 0.021611;
        q.y() = -0.805788;
        q.z() = 0.024535;
        Rs[frame_count] = q.toRotationMatrix().transpose();
        Vs[frame_count] = Vector3d::Zero();
        Bas[frame_count] = Vector3d::Zero();
        Bgs[frame_count] = Vector3d::Zero();
        
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                // result = initialStructure();
                result = true;

                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                // ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            else
                slideWindow();
        }
        else
        {
            if (frame_count > 1)
            {
                optimization_init(frame_count);
            }
            frame_count++;
        }
            
    }
    else
    {
        TicToc t_solve;
        solveOdometry();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        // if (failureDetection())
        // {
        //     ROS_WARN("failure detection!");
        //     getchar();
        //     failure_occur = 1;
        //     clearState();
        //     ROS_WARN("system reboot!");
        //     return;
        // }

        TicToc t_margin;
        slideWindow();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        optimization();
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Position[i][0] = Ps[i].x();   //propagation value for init value of optimization
        para_Position[i][1] = Ps[i].y();
        para_Position[i][2] = Ps[i].z();

        Quaterniond q{Rs[i]};
        para_Attitude[i][0] = q.x();
        para_Attitude[i][1] = q.y();
        para_Attitude[i][2] = q.z();
        para_Attitude[i][3] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
}

void Estimator::vector2double_init(int frame_count)
{
    for (int i = 0; i <= frame_count; i++)
    {
        para_Position[i][0] = Ps[i].x();   //propagation value for init value of optimization
        para_Position[i][1] = Ps[i].y();
        para_Position[i][2] = Ps[i].z();

        Quaterniond q{Rs[i]};
        para_Attitude[i][0] = q.x();
        para_Attitude[i][1] = q.y();
        para_Attitude[i][2] = q.z();
        para_Attitude[i][3] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
}

// void Estimator::double2vector()
// {
//     Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
//     Vector3d origin_P0 = Ps[0];

//     if (failure_occur)
//     {
//         origin_R0 = Utility::R2ypr(last_R0);
//         origin_P0 = last_P0;
//         failure_occur = 0;
//     }
//     Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Attitude[0][3],
//                                                       para_Attitude[0][0],
//                                                       para_Attitude[0][1],
//                                                       para_Attitude[0][2]).toRotationMatrix());
//     double y_diff = origin_R0.x() - origin_R00.x();
//     //TODO
//     Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
//     if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
//     {
//         ROS_DEBUG("euler singular point!");
//         rot_diff = Rs[0] * Quaterniond(para_Attitude[0][3],
//                                        para_Attitude[0][0],
//                                        para_Attitude[0][1],
//                                        para_Attitude[0][2]).toRotationMatrix().transpose();
//     }

//     for (int i = 0; i <= WINDOW_SIZE; i++)
//     {

//         Rs[i] = rot_diff * Quaterniond(para_Attitude[i][3], para_Attitude[i][0], para_Attitude[i][1], para_Attitude[i][2]).normalized().toRotationMatrix();

//         Ps[i] = rot_diff * Vector3d(para_Position[i][0] - para_Position[0][0],
//                                 para_Position[i][1] - para_Position[0][1],
//                                 para_Position[i][2] - para_Position[0][2]) + origin_P0;

//         Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
//                                     para_SpeedBias[i][1],
//                                     para_SpeedBias[i][2]);

//         Bas[i] = Vector3d(para_SpeedBias[i][3],
//                             para_SpeedBias[i][4],
//                             para_SpeedBias[i][5]);

//         Bgs[i] = Vector3d(para_SpeedBias[i][6],
//                             para_SpeedBias[i][7],
//                             para_SpeedBias[i][8]);
//     }
// }

void Estimator::double2vector()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = Quaterniond(para_Attitude[i][3], para_Attitude[i][0], para_Attitude[i][1], para_Attitude[i][2]).normalized().toRotationMatrix();

        Ps[i] = Vector3d(para_Position[i][0],
                                para_Position[i][1],
                                para_Position[i][2]);

        Vs[i] = Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                            para_SpeedBias[i][4],
                            para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                            para_SpeedBias[i][7],
                            para_SpeedBias[i][8]);
    }
}

void Estimator::double2vector_init(int frame_count)
{
    for (int i = 0; i <= frame_count; i++)
    {

        Rs[i] = Quaterniond(para_Attitude[i][3], para_Attitude[i][0], para_Attitude[i][1], para_Attitude[i][2]).normalized().toRotationMatrix();

        Ps[i] = Vector3d(para_Position[i][0],
                                para_Position[i][1],
                                para_Position[i][2]);

        Vs[i] = Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                            para_SpeedBias[i][4],
                            para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                            para_SpeedBias[i][7],
                            para_SpeedBias[i][8]);
    }
}

bool Estimator::failureDetection()
{
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }

    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        return true;
    }
    return false;
}

void Estimator::optimization_init(int frame_count)
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)  
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Position[i], SIZE_POSITION);
        problem.AddParameterBlock(para_Attitude[i], SIZE_ATTITUDE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);

        problem.SetParameterBlockConstant(para_Position[i]);
    }
    problem.SetParameterBlockConstant(para_Attitude[0]);
    problem.SetParameterBlockConstant(para_SpeedBias[0]);

    TicToc t_whole, t_prepare;

    vector2double_init(frame_count);

    for (int i = 0; i < frame_count; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Position[i], para_Attitude[i], para_SpeedBias[i], para_Position[j], para_Attitude[j], para_SpeedBias[j]);
         
    }

    for(int i = 0; i < (frame_count + 1); i++)
    {
        PFactor* p_factor=new PFactor(meas[i]);
        problem.AddResidualBlock(p_factor, NULL, para_Position[i]);
    }

    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    solve_time_ms = t_solver.toc();
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", solve_time_ms);

    double2vector_init(frame_count);
}

void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)  
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Position[i], SIZE_POSITION);
        problem.AddParameterBlock(para_Attitude[i], SIZE_ATTITUDE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

    TicToc t_whole, t_prepare;

    vector2double();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Position[i], para_Attitude[i], para_SpeedBias[i], para_Position[j], para_Attitude[j], para_SpeedBias[j]);
         
    }

    for(int i = 0; i < (WINDOW_SIZE + 1); i++)
    {
        PFactor* p_factor=new PFactor(meas[i]);
        problem.AddResidualBlock(p_factor, NULL, para_Position[i]);
    }

    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    solve_time_ms = t_solver.toc();
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", solve_time_ms);

    double2vector();

    /*PREPARE marginalization info to be added as prior in next window*/
#if(MARG == 1)
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Position[0] ||
                    last_marginalization_parameter_blocks[i] == para_Attitude[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);

            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Position[0], para_Attitude[0], para_SpeedBias[0], para_Position[1], para_Attitude[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1, 2});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            PFactor* p_factor = new PFactor(meas[0]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(p_factor, NULL,
                                                                    vector<double *>{para_Position[0]},
                                                                    vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info); 
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i - 1];
            addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            (std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Position[WINDOW_SIZE - 1]) ||
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Attitude[WINDOW_SIZE - 1])))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_Speed[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Position[WINDOW_SIZE - 1] ||
                        last_marginalization_parameter_blocks[i] == para_Attitude[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
  
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i - 1];
                    addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Position[i])] = para_Position[i];
                    addr_shift[reinterpret_cast<long>(para_Attitude[i])] = para_Attitude[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;      
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
#endif
    
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD) 
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
                std::swap(meas[i], meas[i + 1]);   
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            meas[WINDOW_SIZE] = meas[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            meas[frame_count - 1] = meas[frame_count];
            // std::cout << "slidingnew..." << std::endl;

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
        }
    }
}