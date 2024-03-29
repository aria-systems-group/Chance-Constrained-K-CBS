#pragma once
#include "utils/Conflict.h"
#include "utils/MultiRobotProblemDefinition.h"
#include "Constraints/Constraint.h"
#include <ompl/control/PathControl.h>

namespace oc = ompl::control;
typedef std::vector<oc::PathControl> Plan;


OMPL_CLASS_FORWARD(PlanValidityChecker);
class PlanValidityChecker
{
public:
	PlanValidityChecker(MultiRobotProblemDefinitionPtr pdef, const std::string name):
		mrmp_pdef_(pdef), name_(name) {}

	virtual std::vector<ConflictPtr> validatePlan(Plan p) = 0;

	virtual ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int agent) = 0;

	virtual bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) = 0;

    std::string getName() const {return name_;};

protected:
	MultiRobotProblemDefinitionPtr mrmp_pdef_;
	std::string name_;
};
