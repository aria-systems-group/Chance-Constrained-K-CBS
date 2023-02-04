#pragma once
#include "common.h"
#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "../Spaces/R2BeliefSpace.h"


typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;

/** \brief State propagation for a 2D point motion model. */
class R2_UncertainLinearStatePropagator : public oc::StatePropagator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** \brief Construct representation of a unicycle state propagator.
    */
    R2_UncertainLinearStatePropagator(const oc::SpaceInformationPtr &si);

    virtual ~R2_UncertainLinearStatePropagator(void){}

    /** \brief Will always return false, as the simulation can only proceed forward in time */
    virtual bool canPropagateBackward(void) const;

    /** \brief Propagate from a state, under a given control, for some specified amount of time.
        We use the motion model to do the actual number crunching.
    */
    virtual void propagate(const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result) const;

private:

    Eigen::Matrix2d A_ol_, B_ol_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;

    double duration_;

    mutable double x_pose, x_pose_reference, y_pose, y_pose_reference;

    mutable const R2BeliefSpace::StateType *start_css;

    
    mutable oc::RealVectorControlSpace::ControlType *control_css;

    const Eigen::Vector2d start_css_rvs_pose;
    const Eigen::Matrix2d start_css_rvs_cov;

    Eigen::Matrix2d I = Eigen::MatrixXd::Identity(2, 2);
    Eigen::Matrix2d H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::Matrix2d F = Eigen::MatrixXd::Identity(2, 2);

    Eigen::Matrix2d sigma_pred, lambda_pred, K, Q, Ak;

    double K_ = 0.3;

    mutable const R2BeliefSpace::StateType *result_css;
    ob::RealVectorStateSpace::StateType *result_css_rvs_pose;
    Eigen::Matrix2d result_css_rvs_cov;

protected:

    int dimensions_ = 2;


    // Belief::Belief beliefModel_;

    // MotionModelMethod::MotionModelPointer motionModel_;

    // firm::SpaceInformation::SpaceInformationPtr siF_;
    /**
    You can add a simulated environment here where the controls can get applied, useful for
    showing the graphics, very similar to the concept of ActuationSystem in PMPL.
    */
};
