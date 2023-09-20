#include "Spaces/RealVectorBeliefSpace.h"

// double RealVectorBeliefSpace::StateType::meanNormWeight_  = -1;
// double RealVectorBeliefSpace::StateType::covNormWeight_   = -1;
// double RealVectorBeliefSpace::StateType::reachDist_   = -1;

ompl::base::State* RealVectorBeliefSpace::allocState(void) const
{
    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];
    rstate->sigma_  = 0.01 * Eigen::MatrixXd::Identity(dimension_, dimension_);
    rstate->lambda_ = 0.01 * Eigen::MatrixXd::Identity(dimension_, dimension_);
    return rstate;
}

double RealVectorBeliefSpace::distance(const State* state1, const State *state2) const
{
    // std::cout << "distance" << std::endl;
    // printState(state1, std::cout);
    // printState(state2, std::cout);
    // returns the wasserstein distance
    Eigen::VectorXd mu_diff(dimension_);
    for (int i = 0; i < dimension_; i++) {
        mu_diff[i] = state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i];
    }

    if (mu_diff.squaredNorm() == 0)
        return 0.0;

    // std::cout << mu_diff << std::endl;
    Eigen::MatrixXd cov1 = state1->as<StateType>()->getCovariance();
    Eigen::MatrixXd cov2 = state2->as<StateType>()->getCovariance();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es1(cov1);
    auto cov1_sqrt = es1.operatorSqrt();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es2(cov2);
    auto cov2_sqrt = es2.operatorSqrt();
    auto cov3 = (cov2_sqrt * cov1 * cov2_sqrt);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es3(cov3);
    auto cov3_sqrt = es3.operatorSqrt();

    double t = mu_diff.squaredNorm() + (cov1 + cov2 - (2 * cov3_sqrt)).trace();
    // if (t < 0) {
    //     // printState(state1, std::cout);
    //     // printState(state2, std::cout);
    //     // std::cout << mu_diff.squaredNorm() << std::endl;
    //     // std::cout << cov1_sqrt << std::endl;
    //     // std::cout << cov2_sqrt << std::endl;
    //     // std::cout << (cov2_sqrt * cov1 * cov2_sqrt) << std::endl;
    //     // auto t1 = (cov2_sqrt * cov1 * cov2_sqrt);
    //     // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es3(t1);
    //     // auto cov3_sqrt = es3.operatorSqrt();
    //     // std::cout << cov3_sqrt << std::endl;
    //     // std::cout << 2 * cov3_sqrt << std::endl;
    //     // std::cout << cov1 + cov2 - 2*(cov3_sqrt) << std::endl;
    //     std::cout << "ERROR in DISTANCE" << std::endl;
    //     std::cout << t << std::endl;
    //     exit(-1);
    // }
    // std::cout << "out " << t  << std::endl;
    return abs(t);
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
