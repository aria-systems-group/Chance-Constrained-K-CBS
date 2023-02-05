#include "PlanValidityCheckers/BeliefPlanValidityChecker.h"

BeliefPlanValidityChecker::BeliefPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef):
	PlanValidityChecker(pdef, "BeliefPlanValidityChecker") {};

std::vector<ConflictPtr> BeliefPlanValidityChecker::validatePlan(Plan p)
{
	OMPL_ERROR("%s: validatePlan not yet implemented.", name_.c_str());
	return {};
}

ConstraintPtr BeliefPlanValidityChecker::createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx)
{
	OMPL_ERROR("%s: createConstraint not yet implemented.", name_.c_str());
	return nullptr;
}

