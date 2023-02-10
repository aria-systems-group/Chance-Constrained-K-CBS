#pragma once
// #include "common.h"
// #include "Conflict.h"
#include "Spaces/R2BeliefSpace.h"
#include "Constraints/BeliefConstraint.h"
#include "PlanValidityCheckers/PlanValidityChecker.h"
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <unordered_map>

namespace bm = boost::math;


OMPL_CLASS_FORWARD(BeliefPlanValidityChecker);
class BeliefPlanValidityChecker: public PlanValidityChecker
{
public:
	BeliefPlanValidityChecker(MultiRobotProblemDefinitionPtr pdef);
	~BeliefPlanValidityChecker();

	std::vector<ConflictPtr> validatePlan(Plan p) override;

	ConstraintPtr createConstraint(Plan p, std::vector<ConflictPtr> conflicts, const int robotIdx) override;

	bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

private:
	std::unordered_map<std::string, std::pair<Eigen::Vector2d, Eigen::Matrix2d>> getActiveRobots_(Plan p, const int step, const int a1 = -1, const int a2 = -1);
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> getDistFromState_(ob::State* st);
	ConflictPtr checkForConflicts_(std::unordered_map<std::string, std::pair<Eigen::Vector2d, Eigen::Matrix2d>> states_map, const int step);
	bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes, const bool replanning = false);
	Polygon getMinkowskiSumOfRobots_(Robot* r1, Robot* r2);
	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
	Point addPoints_(const Point &a, const Point &b);
	Point subtractPoints_(const Point &a, const Point &b);
	double crossProduct_(const Point &a, const Point &b);
	std::unordered_map<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> halfPlane_map_;
	const double p_plan_safe_;
	const double p_plan_coll_dist_;
	const double p_constraint_safe_;
	const double p_constraint_coll_dist_;
};
