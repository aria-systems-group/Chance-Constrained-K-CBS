#include "Spaces/R2BeliefSpaceEuclidean.h"


double R2BeliefSpaceEuclidean::StateType::meanNormWeight_  = -1;
double R2BeliefSpaceEuclidean::StateType::covNormWeight_   = -1;
double R2BeliefSpaceEuclidean::StateType::reachDist_   = -1;
// arma::colvec R2BeliefSpaceEuclidean::StateType::normWeights_ = arma::zeros<arma::colvec>(3);

bool R2BeliefSpaceEuclidean::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
{

    Eigen::Vector2d stateDiff = this->getMatrixData() - state->as<R2BeliefSpaceEuclidean::StateType>()->getMatrixData();
    double meanNorm = stateDiff.norm();

    double reachConstraint  = reachDist_;

    if(relaxedConstraint)
        reachConstraint *= 4;

    if(meanNorm <= reachConstraint)
    {
        return true;
    }

    return false;
    
}

ompl::base::State* R2BeliefSpaceEuclidean::allocState(void) const
{

    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];

    return rstate;
}

void R2BeliefSpaceEuclidean::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setSigma(source->as<StateType>()->getSigma());
    destination->as<StateType>()->setLambda(source->as<StateType>()->getLambda());
}

void R2BeliefSpaceEuclidean::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

double R2BeliefSpaceEuclidean::distance(const State* state1, const State *state2) const 
{

    //returns the euclidean distance
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    return pow(dx*dx+dy*dy, 0.5);

}

void R2BeliefSpaceEuclidean::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Yaw]: ";
    std::cout<<"["<<state->as<R2BeliefSpaceEuclidean::StateType>()->getX()<<", "<<state->as<R2BeliefSpaceEuclidean::StateType>()->getY()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance()<<std::endl;
    std::cout<<"Sigma is" << std::endl;
    std::cout<<state->as<R2BeliefSpaceEuclidean::StateType>()->getSigma()<<std::endl;
    std::cout<<"Lambda is" << std::endl;
    std::cout<<state->as<R2BeliefSpaceEuclidean::StateType>()->getLambda()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}