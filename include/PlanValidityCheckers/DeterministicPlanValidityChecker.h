#pragma once
#include "common.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"
#include "Constraints/DeterministicConstraint.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>


OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
OMPL_CLASS_FORWARD(DeterministicPlanValidityChecker);
class DeterministicPlanValidityChecker: public PlanValidityChecker
{
public:
	DeterministicPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef);

	std::vector<ConflictPtr> validatePlan(Plan p) override;

	ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx) override;

	bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override
	{
		OMPL_ERROR("Not yet implemented.");
		return false;
	}
	
private:
	std::vector<std::pair<int, Polygon>> getActiveRobots_(Plan p, const int step, const int a1 = -1, const int a2 = -2);
	Polygon getShapeFromState_(ob::State *st, const int robotIdx);
	ConflictPtr checkForConflicts_(std::vector<std::pair<int, Polygon>> shapes, const int step);
};
