#pragma once
#include "Spaces/RealVectorBeliefSpace.h"
// #include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


// typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;

/** \brief State propagation for a 2D point motion model. */
class CentralizedUncertainLinearStatePropagator : public oc::StatePropagator
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** \brief Construct representation of a unicycle state propagator.
    */
    CentralizedUncertainLinearStatePropagator(const oc::SpaceInformationPtr &si);

    ~CentralizedUncertainLinearStatePropagator(void) override {}

    /** \brief Will always return false, as the simulation can only proceed forward in time */
    bool canPropagateBackward(void) const override;

    /** \brief Propagate from a state, under a given control, for some specified amount of time.
        We use the motion model to do the actual number crunching.
    */
    void propagate(const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result) const override;

private:

    Eigen::Matrix2d A_ol_, B_ol_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;

    const double duration_;

    const double num_agents_;

    // mutable double x_pose, x_pose_reference, y_pose, y_pose_reference;

    // mutable const R2BeliefSpace::StateType *start_css;

    
    // mutable oc::RealVectorControlSpace::ControlType *control_css;

    // const Eigen::Vector2d start_css_rvs_pose;
    // const Eigen::Matrix2d start_css_rvs_cov;

    // single agent matrix definitions
    Eigen::Matrix2d I_ = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d H_ = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d F_ = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
    double K_ = 0.3;
    void singleAgentPropogate_(Eigen::Vector2d& mu, Eigen::Matrix2d& sigma, Eigen::Matrix2d& lambda, Eigen::Vector2d& cntrl, Eigen::Vector2d& nxt_mu, Eigen::Matrix2d& nxt_sigma, Eigen::Matrix2d& nxt_lambda) const;
};
