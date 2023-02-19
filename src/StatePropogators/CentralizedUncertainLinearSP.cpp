#include "StatePropogators/CentralizedUncertainLinearSP.h"


CentralizedUncertainLinearStatePropagator::CentralizedUncertainLinearStatePropagator(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
{
    // duration_ = si->getPropagationStepSize();

    // //=========================================================================
    // // Open loop system definition
    // //=========================================================================
    // A_ol_.resize(2, 2);
    // A_ol_ << 1.0, 0.0,
    //          0.0, 1.0;

    // B_ol_.resize(2, 2);
    // B_ol_ << 1.0, 0.0,
    //          0.0, 1.0;

    // //=========================================================================
    // // Close loop system definition
    // //=========================================================================
    // A_cl_.resize(2, 2);
    // A_cl_ = A_ol_ - B_ol_ * K_;

    // B_cl_.resize(2, 2);
    // B_cl_ = B_ol_ * K_;

    // //=========================================================================
    // // Discrete close loop system definition
    // //=========================================================================
    // A_cl_d_.resize(2, 2);
    // A_cl_d_ = Eigen::MatrixXd::Identity(2, 2) + A_cl_ * duration_;

    // B_cl_d_.resize(2, 2);
    // B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(2, 2)) * B_cl_;

    // double processNoise = 0.1;
    // Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    // double measurementNoise = 0.1;
    // R = pow(measurementNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
}

void CentralizedUncertainLinearStatePropagator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const
{
    // // use the motionmodel to apply the controls
    // // motionModel_->Evolve(state, control, motionModel_->getZeroNoise(), to);
    // //=========================================================================
    // // Get CX vector (RRT near vertex)
    // //=========================================================================
    // start_css = state->as<R2BeliefSpace::StateType >();
    // x_pose = start_css->getX();
    // y_pose = start_css->getY();
    // //=========================================================================
    // // Get CX vector (RRT random vertex)
    // //=========================================================================
    // x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    // y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    // //=========================================================================
    // // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    // //=========================================================================
    // double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    // double u_1 = B_ol_(0, 0) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);

    // result->as<R2BeliefSpace::StateType>()->setX(x_pose + duration * x_pose_reference);
    // result->as<R2BeliefSpace::StateType>()->setY(y_pose + duration * y_pose_reference);

    // //=========================================================================
    // // Propagate covariance in the equivalent closed loop system
    // //=========================================================================
    // Eigen::Matrix2d sigma_from = state->as<R2BeliefSpace::StateType>()->getSigma();
    // Eigen::Matrix2d lambda_from = state->as<R2BeliefSpace::StateType>()->getLambda();
    // Eigen::Matrix2d sigma_pred = F*sigma_from*F + Q;

    // Mat lambda_pred, K;
    // Mat S = (H * sigma_pred * H.transpose())+ R;
    // K = (sigma_pred * H.transpose()) * S.inverse();
    // lambda_pred = A_cl_*lambda_from*A_cl_;
    // Mat sigma_to = (I - (K*H)) * sigma_pred;
    // Mat lambda_to = lambda_pred + K*H*sigma_pred;
    // result->as<R2BeliefSpace::StateType>()->setSigma(sigma_to);
    // result->as<R2BeliefSpace::StateType>()->setLambda(lambda_to);
}

bool CentralizedUncertainLinearStatePropagator::canPropagateBackward(void) const
{
    return false;
}
