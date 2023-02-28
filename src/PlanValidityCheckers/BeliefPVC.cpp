#include "PlanValidityCheckers/BeliefPVC.h"


BeliefPVC::BeliefPVC(MultiRobotProblemDefinitionPtr pdef, const std::string name, const double p_safe_agnts):
    PlanValidityChecker(pdef, name), p_safe_agnts_(p_safe_agnts), p_coll_agnts_(-1)
{
    p_coll_agnts_ = 1 - p_safe_agnts;
}

ConstraintPtr BeliefPVC::createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int constrained_robot)
{
    // interpolate all trajectories to the same discretization and get max size
    for (auto itr = p.begin(); itr != p.end(); itr++) {
        itr->interpolate();
    }

    int constraining_robot = (constrained_robot == conflicts.front()->agent1Idx_) ? conflicts.front()->agent2Idx_ : conflicts.front()->agent1Idx_;
    assert(constrained_robot != constraining_robot);
    const double step_duration = mrmp_pdef_->getSystemStepSize();
    std::vector<double> times;
    std::vector<ob::State*> states;
    for (auto itr = conflicts.begin(); itr != conflicts.end(); itr++) {
        times.push_back(((*itr)->timeStep_ * step_duration));
        if ( (*itr)->timeStep_ <  p[constraining_robot].getStates().size()) {
            ob::State* st = mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->allocState();
            mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->copyState(st, p[constraining_robot].getState((*itr)->timeStep_));
            states.push_back(st);
            // mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->printState(st);

        }
        else {
            ob::State* st = mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->allocState();
            mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->copyState(st, p[constraining_robot].getStates().back());
            states.push_back(st);
            // mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->printState(st);
        }
    }
    ConstraintPtr c = std::make_shared<BeliefConstraint>(constrained_robot, constraining_robot, times, states);
    return c;
}

std::map<std::string, Belief> BeliefPVC::getActiveRobots_(Plan p, const int step, const int a1, const int a2)
{
    std::map<std::string, Belief> activeRobots;

    // using no pre-defined agent pair, return vector of all robots
    if (a1 == -1 || a2 == -1) {
        int idx = 0;
        auto itr = p.begin();
        for(; itr < p.end(); itr++, idx++ )
        {
            if (step < itr->getStateCount()){
                Belief b = getDistribution_(itr->getState(step));
                activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[idx]->getName(), b});
            }
            else{
                Belief b = getDistribution_(itr->getStates().back());
                activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[idx]->getName(), b});
            }
        }
    }
    else {
        // provided agent pair, return vector with only those two robots
        if (step < p[a1].getStateCount()) {
            Belief b = getDistribution_(p[a1].getState(step));
            activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[a1]->getName(), b});
        }
        else {
            Belief b = getDistribution_(p[a1].getStates().back());
            activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[a1]->getName(), b});
        }
        if (step < p[a2].getStateCount()) {
            Belief b = getDistribution_(p[a2].getState(step));
            activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[a2]->getName(), b});
        }
        else {
            Belief b = getDistribution_(p[a2].getStates().back());
            activeRobots.insert({mrmp_pdef_->getInstance()->getRobots()[a2]->getName(), b});
        }
    }
    return activeRobots;
}

Belief BeliefPVC::getDistribution_(const ob::State* st)
{
    double* all_vals = st->as<RealVectorBeliefSpace::StateType>()->values;
    Eigen::VectorXd mu(2);
    mu[0] = all_vals[0];
    mu[1] = all_vals[1];
    Eigen::Matrix2d Sigma = st->as<RealVectorBeliefSpace::StateType>()->getCovariance().block<2, 2>(0, 0);
    Belief b(mu, Sigma);
    return b;
}
