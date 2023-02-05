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
	~BeliefPlanValidityChecker();
	std::vector<ConflictPtr> validatePlan(Plan p);

	ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx);
private:
	std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> getActiveRobots_(Plan p, const int step, const int a1 = -1, const int a2 = -1);
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> getDistFromState_(ob::State* st);
	ConflictPtr checkForConflicts_(std::vector<std::pair<int, std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> states, const int step);
	bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab);
	std::vector<std::vector<double>> HalfPlanes_;
	const double p_safe_;
	const double p_coll_dist_;
};
