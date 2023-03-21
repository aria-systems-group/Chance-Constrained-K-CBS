/* Author: Qi Heng */

#include "StatePropogators/UncertainUnicycleSP.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;


UncertainUnicycleStatePropagator::UncertainUnicycleStatePropagator(const oc::SpaceInformationPtr &si): 
    oc::StatePropagator(si), duration_(si->getPropagationStepSize())
{
    //=========================================================================
    // Open loop system definition
    //=========================================================================
    Eigen::Matrix4d A_ol;
    A_ol <<  1.0, 0.2, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.2,
             0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 4, 2> B_ol;
    B_ol << 0.02, 0.0,
            0.2, 0.0,
            0.0, 0.02,
            0.0, 0.2;

    //=========================================================================
    // PD controller coefficients (from LQR)
    //=========================================================================
    K_ << 1, 1, 0.0, 0.0,
         0.0, 0.0, 1, 1;

    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_ = A_ol - B_ol * K_;

    // std::cout << A_cl_ << std::endl;
    // exit(-1);

    // A_cl_d_.resize(4, 4);
    A_cl_d_ = Eigen::MatrixXd::Identity(4, 4) - A_cl_ * duration_;
    A_cl_d_ = A_cl_d_.inverse().eval();

    // set a 2 by 2 matrix for uncertainty propagation
    A_cl_d_22_(0, 0) = 0.7; //A_cl_d_(0, 0);
    A_cl_d_22_(0, 1) = A_cl_d_(0, 2);
    A_cl_d_22_(1, 0) = A_cl_d_(2, 0);
    A_cl_d_22_(1, 1) = 0.7; //A_cl_d_(2, 2);

    double processNoise = 0.1;
    Q_ = pow(processNoise, 2) * I_;
    double measurementNoise = 0.1;
    R_ = pow(measurementNoise, 2) * I_;
    // Q_ << 0.05, 0.0,
           // 0.0, 0.1;
    // R_ = Q_;
}

double saturate(double value, const double min_value, const double max_value) {
    if(value < min_value)
        return min_value;
    else if(value > max_value) 
        return max_value;
    else
        return value;
}

double wrap(double angle) 
{
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) 
        angle -= 2 * M_PI;
    return angle;
}

void UncertainUnicycleStatePropagator::propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const 
{
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    const double x_pose = start->as<RealVectorBeliefSpace::StateType>()->values[0];
    const double y_pose = start->as<RealVectorBeliefSpace::StateType>()->values[1];
    const double yaw    = start->as<RealVectorBeliefSpace::StateType>()->values[2];
    const double surge  = start->as<RealVectorBeliefSpace::StateType>()->values[3];

    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    const double x_pose_reference   = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    const double y_pose_reference   = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    const double yaw_reference      = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];
    const double surge_reference    = control->as<oc::RealVectorControlSpace::ControlType>()->values[3];

    // std::cout << x_pose_reference

    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y))) with PD controller
    //=========================================================================
    double u_0 = K_(0, 0) * (x_pose_reference - x_pose) + K_(0, 1) * (surge_reference * cos(yaw_reference) - surge * cos(yaw));
    double u_1 = K_(1, 3) * (y_pose_reference - y_pose) + K_(1, 2) * (surge_reference * sin(yaw_reference) - surge * sin(yaw));

    //=========================================================================
    // Get dot(v) and dot(yaw) from dot(dot(x)) dot(dot(y))
    //=========================================================================
    double u_bar_0 = cos(yaw) * u_0 + sin(yaw) * u_1;
    double u_bar_1 = (-sin(yaw) * u_0 + cos(yaw) * u_1) / surge;

    //=========================================================================
    // Bound controller outputs (dot(v) and dot(yaw))
    //=========================================================================
    u_bar_0 = saturate(u_bar_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    u_bar_1 = saturate(u_bar_1, turning_rate_bounds_[0], turning_rate_bounds_[1]);
   
    //=========================================================================
    // Bound surge
    //=========================================================================
    double surge_final = surge + duration * u_bar_0;
    surge_final = saturate(surge_final, surge_bounds_[0], surge_bounds_[1]);
   
    //=========================================================================
    // Propagate mean
    //=========================================================================
    result->as<RealVectorBeliefSpace::StateType>()->values[0] = x_pose + duration * cos(yaw) * surge_final;
    result->as<RealVectorBeliefSpace::StateType>()->values[1] = y_pose + duration * sin(yaw) * surge_final;
    result->as<RealVectorBeliefSpace::StateType>()->values[2] = wrap(yaw + duration * u_bar_1);
    result->as<RealVectorBeliefSpace::StateType>()->values[3] = surge_final;

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    Eigen::Matrix2d sigma_from  = start->as<RealVectorBeliefSpace::StateType>()->sigma_.block<2, 2>(0, 0);
    Eigen::Matrix2d lambda_from = start->as<RealVectorBeliefSpace::StateType>()->lambda_.block<2, 2>(0, 0);;
    Eigen::Matrix2d sigma_pred  = F_ * sigma_from * F_ + Q_;

    Eigen::Matrix2d S = (H_ * sigma_pred * H_.transpose()) + R_;
    Eigen::Matrix2d K = (sigma_pred * H_.transpose()) * S.inverse();
    Eigen::Matrix2d lambda_pred = A_cl_d_22_ * lambda_from * A_cl_d_22_;
    Eigen::MatrixXd sigma_to = (I_ - (K * H_)) * sigma_pred;
    Eigen::MatrixXd lambda_to = lambda_pred + K * H_ * sigma_pred;

    result->as<RealVectorBeliefSpace::StateType>()->sigma_ = 0.00001 * Eigen::MatrixXd::Identity(4, 4);;
    result->as<RealVectorBeliefSpace::StateType>()->sigma_.block<2, 2>(0, 0) = sigma_to;
    
    result->as<RealVectorBeliefSpace::StateType>()->lambda_ = 0.00001 * Eigen::MatrixXd::Identity(4, 4);;
    result->as<RealVectorBeliefSpace::StateType>()->lambda_.block<2, 2>(0, 0) = lambda_to;

    // std::cout << "here"<< std::endl;
    // si_->printState(start, std::cout);
    // si_->printState(result, std::cout);
}

bool UncertainUnicycleStatePropagator::canPropagateBackward(void) const
{
    return false;
}