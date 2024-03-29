#pragma once
#include "Spaces/RealVectorBeliefSpace.h"
#include "Constraints/BeliefConstraint.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"

typedef std::pair<Eigen::VectorXd, Eigen::MatrixXd> Belief;

OMPL_CLASS_FORWARD(BeliefPVC);
class BeliefPVC: public PlanValidityChecker
{
public:
    BeliefPVC(MultiRobotProblemDefinitionPtr pdef, const std::string name, const double p_safe_agnts);

    ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx) override;

protected:
    std::map<std::string, Belief> getActiveRobots_(Plan p, const int step, const int a1 = -1, const int a2 = -1);
    Belief getDistribution_(const ob::State* st);
    const double p_safe_agnts_;
    double p_coll_agnts_;
};
