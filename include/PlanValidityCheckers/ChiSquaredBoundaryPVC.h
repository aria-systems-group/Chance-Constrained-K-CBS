#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <Eigen/Eigenvalues> 
#include <unordered_map>

namespace bm = boost::math;


class ChiSquaredBoundaryPVC: public BeliefPVC
{
public:
    ChiSquaredBoundaryPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

private:
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Belief belief_a, const double rad_a, const Belief belief_b, const double rad_b);
    std::unordered_map<std::string, double> boundingRadii_map;
    double sc_;
};
