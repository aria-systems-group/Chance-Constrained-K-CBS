#pragma once
#include "Spaces/R2BeliefSpace.h"
#include "Constraints/BeliefConstraint.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"

typedef std::pair<Eigen::VectorXd, Eigen::MatrixXd> Belief;

OMPL_CLASS_FORWARD(BeliefPVC);
class BeliefPVC: public PlanValidityChecker
{
public:
    BeliefPVC(MultiRobotProblemDefinitionPtr pdef, const std::string name, const double p_safe);

    ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx) override;

protected:
    std::unordered_map<std::string, Belief> getActiveRobots_(Plan p, const int step, const int a1 = -1, const int a2 = -1);
    Belief getDistribution_(const ob::State* st);
    const double p_safe_;
};
