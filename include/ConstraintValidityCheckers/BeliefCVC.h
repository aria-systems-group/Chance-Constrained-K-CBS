#pragma once
#include "ConstraintValidityCheckers/ConstraintValidityChecker.h"
#include "Constraints/BeliefConstraint.h"
#include "Spaces/R2BeliefSpace.h"

class BeliefCVC: public ConstraintValidityChecker
{
public:
	BeliefCVC(Robot *robot):
		ConstraintValidityChecker(robot, "BeliefCVC") {}

	virtual bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;
};

