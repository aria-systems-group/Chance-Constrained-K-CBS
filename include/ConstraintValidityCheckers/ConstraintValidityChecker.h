#pragma once
#include "common.h"
#include "Constraints/Constraint.h"
#include <ompl/control/PathControl.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
typedef std::vector<oc::PathControl> Plan;


OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
OMPL_CLASS_FORWARD(ConstraintValidityChecker);
class ConstraintValidityChecker
{
public:
	ConstraintValidityChecker(Robot* robot, std::string name = "ConstraintValidityChecker"):
		robot_(robot), name_(name) {}

	virtual bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) = 0;

protected:
	Robot* robot_;
	std::string name_;
};
