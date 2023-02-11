#include "PlanValidityCheckers/BeliefPVC.h"


BeliefPVC::BeliefPVC(MultiRobotProblemDefinitionPtr pdef, const std::string name, const double p_safe):
    PlanValidityChecker(pdef, name), p_safe_(p_safe){}

ConstraintPtr BeliefPVC::createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int constrained_robot)
{
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
        }
        else {
            ob::State* st = mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->allocState();
            mrmp_pdef_->getRobotSpaceInformationPtr(constraining_robot)->copyState(st, p[constraining_robot].getStates().back());
            states.push_back(st);
        }
    }
    ConstraintPtr c = std::make_shared<BeliefConstraint>(constrained_robot, constraining_robot, times, states);
    return c;
}

std::unordered_map<std::string, Belief> BeliefPVC::getActiveRobots_(Plan p, const int step, const int a1, const int a2)
{
    std::unordered_map<std::string, Belief> activeRobots;

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
    Eigen::VectorXd mu = st->as<R2BeliefSpace::StateType>()->getXY();
    Eigen::MatrixXd Sigma = st->as<R2BeliefSpace::StateType>()->getCovariance();
    Belief b(mu, Sigma);
    return b;
}
