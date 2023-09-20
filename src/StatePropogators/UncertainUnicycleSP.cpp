/* Author: Qi Heng */

#include "StatePropogators/UncertainUnicycleSP.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;




DynUnicycleControlSpace::DynUnicycleControlSpace(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si) {

    // controller_parameters_ = controller_parameters;
    // forward_acceleration_bounds_ = forward_acceleration_bounds;
    // turning_rate_bounds_ = turning_rate_bounds;
    // surge_bounds_ = surge_bounds;
    // system_noise_ = system_noise;
    // std::cout << "initialized my man1" << std::endl;
    duration_ = si->getPropagationStepSize();
    // std::cout << "initialized my man1" << std::endl;

    forward_acceleration_bounds_.push_back(-0.5);
    forward_acceleration_bounds_.push_back(0.5);
    turning_rate_bounds_.push_back(-0.5); //-2.0
    turning_rate_bounds_.push_back(0.5); //2.0
    surge_bounds_.push_back(0.05);
    surge_bounds_.push_back(5.0);
    system_noise_.push_back(0.1);
    system_noise_.push_back(0.0);
    system_noise_.push_back(0.0);
    system_noise_.push_back(0.1);
    //=========================================================================
    // Open loop system definition
    //=========================================================================
    // A_ol_.resize(4, 4);
    A_ol_ << 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, 0.0;
    std::cout << "initialized my man2" << std::endl;
    // B_ol_.resize(4, 2);
    B_ol_ << 0.0, 0.0,
             1.0, 0.0,
             0.0, 0.0,
             0.0, 1.0;

    // set system noise in the state space
    // (we only care about Pxx, Pxy, Pyx, Pyy)
    // // Pw_22_.resize(2, 2);
    // Pw_22_ << 0.0, 0.0,
    //           0.0, 0.0;
    // Pw_22_ *= duration_ * duration_;
    // std::cout << "initialized my man3" << std::endl;
    //=========================================================================
    // PD controller coefficients (from LQR)
    //=========================================================================
    // K_.resize(2, 4);
    K_(0, 0) = 2.5857;
    K_(0, 1) = 3.4434;
    K_(0, 2) = 0.0;
    K_(0, 3) = 0.0;
    K_(1, 0) = 0.0;
    K_(1, 1) = 0.0;
    K_(1, 2) = 2.5857;
    K_(1, 3) = 3.4434;

    // controller_parameters_.push_back(0.316);
    // controller_parameters_.push_back(1.054);
    // controller_parameters_.push_back(0.0);
    // controller_parameters_.push_back(0.0);
    // controller_parameters_.push_back(0.0);
    // controller_parameters_.push_back(0.0);
    // controller_parameters_.push_back(1.054);
    // controller_parameters_.push_back(0.316);

    controller_parameters_.push_back(1.0);
    controller_parameters_.push_back(1.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(0.0);
    controller_parameters_.push_back(1.0);
    controller_parameters_.push_back(1.0);
    //=========================================================================
    // Close loop system definition
    //=========================================================================
    // A_cl_.resize(4, 4);
    Eigen::Matrix4d A_ol_d_dt = A_ol_ * duration_;
    // std::cout << duration_ << std::endl;
    // std::cout << A_ol_d_dt << std::endl;
    Eigen::Matrix4d A_ol_d = A_ol_d_dt.exp();
    // std::cout << A_ol_d << std::endl;


    Eigen::MatrixXd AB(A_ol_.rows(), A_ol_.cols()+B_ol_.cols());
    Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(A_ol_.cols()+B_ol_.cols()-A_ol_.rows(), A_ol_.cols()+B_ol_.cols());
    AB.leftCols(A_ol_.cols()) = A_ol_;
    AB.rightCols(B_ol_.cols()) = B_ol_;
    Eigen::MatrixXd AB0(AB.rows()+zeroMat.rows(), AB.cols());
    AB0.topRows(AB.rows()) = AB;
    AB0.bottomRows(zeroMat.rows()) = zeroMat;
    Eigen::MatrixXd AB0dt = duration_*AB0;
    Eigen::MatrixXd AB0exp = AB0dt.exp();
    Eigen::MatrixXd B_ol_d = AB0exp.rightCols(2).topRows(4);

    A_cl_d_ = A_ol_d - B_ol_d * K_;

    // std::cout << A_ol_d << std::endl;
    // std::cout << B_ol_d << std::endl;
    // std::cout << A_cl_d_ << std::endl;
    // exit(1);


    F = A_ol_d;
    

    // set a 2 by 2 matrix for uncertainty propagation
    // A_cl_d_22_ = A_cl_d_.block<2, 2>(0, 0);


    // std::cout << A_cl_d_22_(0,0) << " " << A_cl_d_22_(1,1) << std::endl;

    double processNoise = 0.1; //process noise is 0.0 for scenario
    Q = pow(processNoise, 2) * Eigen::Matrix4d::Identity();

    double measurementNoise = 0.1;
    R = pow(measurementNoise, 2) * Eigen::Matrix4d::Identity();

    std::cout << "initialized my man" << std::endl;
}

void saturate(double &value, const double min_value, const double max_value) {
    if(value < min_value) value = min_value;
    if(value > max_value) value = max_value;
}

double wrap(double angle) 
{
    double k = std::floor((M_PI - angle) / (2*M_PI));
    double alpha = angle + (2 * k * M_PI);
    if (abs(alpha) > M_PI)
        std::cout << "WRONG wrapToPi!!" << std::endl;
    return alpha;
}

void DynUnicycleControlSpace::propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const {
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    
    double x_pose, y_pose, yaw, surge, Pxx_init, Pyy_init;
    x_pose = start->as<RealVectorBeliefSpace::StateType>()->values[0];
    y_pose = start->as<RealVectorBeliefSpace::StateType>()->values[1];
    yaw = start->as<RealVectorBeliefSpace::StateType>()->values[2];
    surge = start->as<RealVectorBeliefSpace::StateType>()->values[3];
    double cos_y = cos(yaw);
    double sin_y = sin(yaw);
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    double x_pose_reference, y_pose_reference, yaw_reference, surge_reference;

    // control_css = control->as<oc::CompoundControlSpace::ControlType>();
    // const oc::RealVectorControlSpace::ControlType *control_css_rvs_pose = control_css->as<oc::RealVectorControlSpace::ControlType>(0);

    x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    yaw_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];
    surge_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[3];
    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y))) with PD controller
    //=========================================================================
    double u_0 = controller_parameters_[0] * (x_pose_reference - x_pose) + controller_parameters_[1] * (surge_reference * cos(yaw_reference) - surge * cos_y);
    double u_1 = controller_parameters_[6] * (y_pose_reference - y_pose) + controller_parameters_[7] * (surge_reference * sin(yaw_reference) - surge * sin_y);
    // std::cout << i++ << std::endl;
    //=========================================================================
    // Get dot(v) and dot(yaw) from dot(dot(x)) dot(dot(y))
    //=========================================================================
    u_bar_0 = cos_y * u_0 + sin_y * u_1;
    u_bar_1 = (-sin_y * u_0 + cos_y * u_1) / surge;

    //=========================================================================
    // Bound controller outputs (dot(v) and dot(yaw))
    //=========================================================================
    saturate(u_bar_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    saturate(u_bar_1, turning_rate_bounds_[0], turning_rate_bounds_[1]);
    //=========================================================================
    // Bound surge
    //=========================================================================
    surge_final = surge + duration * u_bar_0;
    // saturate(surge_final, surge_bounds_[0], surge_bounds_[1]);
    //=========================================================================
    // Propagate mean
    //=========================================================================
    // ob::CompoundStateSpace::StateType *result_css = result->as<ob::CompoundStateSpace::StateType>();
    // RealVectorBeliefSpace::StateType *result_css_rvs_pose = result_css->as<RealVectorBeliefSpace::StateType>(0);

    // result_css_rvs_pose->setX();
    // result_css_rvs_pose->setY();
    // result_css->as<ob::SO2StateSpace::StateType>(1)->value = ;
    // result_css->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = surge_final;
    result->as<RealVectorBeliefSpace::StateType>()->values[0] = x_pose + duration * cos_y * surge_final;
    result->as<RealVectorBeliefSpace::StateType>()->values[1] = y_pose + duration * sin_y * surge_final;
    result->as<RealVectorBeliefSpace::StateType>()->values[2] = wrap(yaw + duration * u_bar_1);
    result->as<RealVectorBeliefSpace::StateType>()->values[3] = surge_final;
    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    // std::cout << "-----------" << std::endl;
    Eigen::Matrix4d sigma_from = start->as<RealVectorBeliefSpace::StateType>()->sigma_;
    Eigen::Matrix4d lambda_from = start->as<RealVectorBeliefSpace::StateType>()->lambda_;
    // std::cout << lambda_from << std::endl;
    Eigen::Matrix4d sigma_pred = (F * sigma_from * F) + Q;

    Eigen::Matrix4d S = (H * sigma_pred * H.transpose())+ R;
    auto K = (sigma_pred * H.transpose()) * S.inverse();
    auto lambda_pred = A_cl_d_ * lambda_from * A_cl_d_.transpose();
    // std::cout << lambda_pred << std::endl;
    Eigen::Matrix4d sigma_to = (I - (K * H)) * sigma_pred;
    Eigen::Matrix4d lambda_to = lambda_pred + K * H * sigma_pred;

    result->as<RealVectorBeliefSpace::StateType>()->sigma_ = sigma_to;
    result->as<RealVectorBeliefSpace::StateType>()->lambda_ = lambda_to;

    // std::cout << lambda_to << std::endl;
    // std::cout << "-----------" << std::endl;
}

bool DynUnicycleControlSpace::canPropagateBackward(void) const
{
    return false;
}