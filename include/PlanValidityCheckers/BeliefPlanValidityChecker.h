#pragma once
#include "common.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"
#include "Constraints/BeliefConstraint.h"
#include "Spaces/R2BeliefSpace.h"

OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
OMPL_CLASS_FORWARD(BeliefPlanValidityChecker);
class BeliefPlanValidityChecker: public PlanValidityChecker
{
public:
	BeliefPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef);

	std::vector<ConflictPtr> validatePlan(Plan p);

	ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx);
};
