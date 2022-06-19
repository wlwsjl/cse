#ifndef P_FACTOR_H
#define P_FACTOR_H

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../parameters.h"

#include <ceres/ceres.h>

class PFactor : public ceres::SizedCostFunction<3, 3>
{
  public:
    PFactor()=delete;
    PFactor(MeasPose _pdata):pdata(_pdata){}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
 
        Eigen::Vector3d P = pdata.r;

        Eigen::Matrix<double, 3, 3> p_covariance = pdata.cov;

        Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);

        residual.block<3, 1>(O_P, 0) = Pi - P;

        Eigen::Matrix<double, 3, 3> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 3, 3>>(p_covariance.inverse()).matrixL().transpose();
        residual = sqrt_info * residual;

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_position_i(jacobians[0]);
                jacobian_position_i.setZero();

                jacobian_position_i.block<3, 3>(O_P, O_P) = Eigen::Matrix3d::Identity();

                jacobian_position_i = sqrt_info * jacobian_position_i;

                if (jacobian_position_i.maxCoeff() > 1e8 || jacobian_position_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of IMU residual wrt position_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
        }

        return true;
    }

    MeasPose pdata;
};

#endif //P_FACTOR_H
