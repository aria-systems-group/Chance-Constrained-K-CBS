#include "StatePropogators/UncertainLinearSP.h"


R2_UncertainLinearStatePropagator::R2_UncertainLinearStatePropagator(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si),
    duration_(si->getPropagationStepSize())
{
    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_.resize(2, 2);
    A_ol_ << 1.0, 0.0,
             0.0, 1.0;

    B_ol_.resize(2, 2);
    B_ol_ << 1.0, 0.0,
             0.0, 1.0;

    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_.resize(2, 2);
    A_cl_ = A_ol_ - B_ol_ * K_;

    B_cl_.resize(2, 2);
    B_cl_ = B_ol_ * K_;

    //=========================================================================
    // Discrete close loop system definition
    //=========================================================================
    A_cl_d_.resize(2, 2);
    A_cl_d_ = I_ + A_cl_ * duration_;

    B_cl_d_.resize(2, 2);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - I_) * B_cl_;

    double processNoise = 1;
    Q_ = pow(processNoise, 2) * I_;
    double measurementNoise = 1;
    R_ = pow(measurementNoise, 2) * I_;
}

// void saturate(double &value, const double &min_value, const double &max_value) {
//     if(value < min_value) value = min_value;
//     if(value > max_value) value = max_value;
// }

// double wrap(double angle) {
//     angle = fmod(angle, 2 * M_PI);
//     if (angle > M_PI) angle -= 2 * M_PI;
//     return angle;
// }

void R2_UncertainLinearStatePropagator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
{
    const double* all_st_values = state->as<RealVectorBeliefSpace::StateType>()->values;
    const double* all_cntrl_values = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const Eigen::MatrixXd all_sigmas = state->as<RealVectorBeliefSpace::StateType>()->sigma_;
    const Eigen::MatrixXd all_lambdas = state->as<RealVectorBeliefSpace::StateType>()->lambda_;

    // propogate the individual agent
    Eigen::Vector2d nxt_mu_a = Eigen::Vector2d::Zero();
    Eigen::Matrix2d nxt_sigma_a = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d nxt_lambda_a = Eigen::Matrix2d::Zero();

    // extract agent-specific state information
    Eigen::Vector2d mu_a{all_st_values[0], all_st_values[1]};
    Eigen::Vector2d cntrl_a{all_cntrl_values[0], all_cntrl_values[1]};
    Eigen::Matrix2d sigma_a = all_sigmas.block<2, 2>(0, 0);
    Eigen::Matrix2d lambda_a = all_lambdas.block<2, 2>(0, 0);

    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    const double x_pose = mu_a[0];
    const double y_pose = mu_a[1];
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    const double x_pose_reference = cntrl_a[0];
    const double y_pose_reference = cntrl_a[1];
    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    // const double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    // const double u_1 = B_ol_(0, 0) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);

    nxt_mu_a[0] = x_pose + duration_ * x_pose_reference;
    nxt_mu_a[1] = y_pose + duration_ * y_pose_reference;

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    const Eigen::Matrix2d sigma_pred = F_ * sigma_a * F_ + Q_;

    const Eigen::Matrix2d S = (H_ * sigma_pred * H_.transpose()) + R_;
    const Eigen::Matrix2d K = (sigma_pred * H_.transpose()) * S.inverse();
    const Eigen::Matrix2d lambda_pred = A_cl_ * lambda_a * A_cl_;
    nxt_sigma_a = (I_ - (K * H_)) * sigma_pred;
    nxt_lambda_a = lambda_pred + K * H_ * sigma_pred;

    // update the resulting states
    result->as<RealVectorBeliefSpace::StateType>()->values[0] = nxt_mu_a[0];
    result->as<RealVectorBeliefSpace::StateType>()->values[1] = nxt_mu_a[1];
    result->as<RealVectorBeliefSpace::StateType>()->sigma_.block<2, 2>(0, 0) = nxt_sigma_a;
    result->as<RealVectorBeliefSpace::StateType>()->lambda_.block<2, 2>(0, 0) = nxt_lambda_a;
}

bool R2_UncertainLinearStatePropagator::canPropagateBackward(void) const
{
    return false;
}
