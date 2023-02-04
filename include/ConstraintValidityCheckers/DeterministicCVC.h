#pragma once
#include "ConstraintValidityCheckers/ConstraintValidityChecker.h"
#include "Constraints/DeterministicConstraint.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>


class DeterministicCVC: public ConstraintValidityChecker
{
public:
	DeterministicCVC(Robot *robot):
		ConstraintValidityChecker(robot, "DeterministicCVC") {}

	virtual bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;
private:
	Polygon getShapeFromState_(ob::State *st);
};
