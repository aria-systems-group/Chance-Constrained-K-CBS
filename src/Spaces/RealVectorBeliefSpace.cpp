#include "Spaces/RealVectorBeliefSpace.h"

// double RealVectorBeliefSpace::StateType::meanNormWeight_  = -1;
// double RealVectorBeliefSpace::StateType::covNormWeight_   = -1;
// double RealVectorBeliefSpace::StateType::reachDist_   = -1;

ompl::base::State* RealVectorBeliefSpace::allocState(void) const
{
    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];
    rstate->sigma_ = Eigen::MatrixXd::Zero(dimension_, dimension_);
    rstate->lambda_ = Eigen::MatrixXd::Zero(dimension_, dimension_);
    return rstate;
}

double RealVectorBeliefSpace::distance(const State* state1, const State *state2) const
{
    // returns the wasserstein distance
    Eigen::VectorXd mu_diff(dimension_);
    for (int i = 0; i < dimension_; i++) {
        mu_diff[i] = state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i];
    }
    Eigen::MatrixXd cov1 = state1->as<StateType>()->getCovariance();
    Eigen::MatrixXd cov2 = state2->as<StateType>()->getCovariance();
    return mu_diff.squaredNorm() + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace();
}

void RealVectorBeliefSpace::copyState(State *destination, const State *source) const
{
    RealVectorStateSpace::copyState(destination, source);
    destination->as<StateType>()->sigma_ = source->as<StateType>()->sigma_;
    destination->as<StateType>()->lambda_ = source->as<StateType>()->lambda_;
}

void RealVectorBeliefSpace::freeState(State *state) const
{
    state->as<StateType>()->sigma_.resize(0, 0);
    state->as<StateType>()->lambda_.resize(0, 0);
    RealVectorStateSpace::freeState(state);
}

void RealVectorBeliefSpace::printState(const State *state, std::ostream &out) const
{
    RealVectorStateSpace::printState(state, out);
    out << "Sigma: " << std::endl <<  state->as<StateType>()->sigma_ << std::endl;
    out << "Lambda: " << std::endl <<  state->as<StateType>()->lambda_ << std::endl;
}
