#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Eigenvalues> 
#include <unordered_map>

namespace bm = boost::math;


class ChiSquaredBoundaryPVC: public BeliefPVC
{
public:
    ChiSquaredBoundaryPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

    bool independentCheck(ob::State* state1, ob::State* state2);

private:
    double chi_squared_quantile_(double v, double p)
    {
        return quantile(bm::chi_squared(v), p);
    }
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Belief belief_a, const double rad_a, const Belief belief_b, const double rad_b);
    std::unordered_map<std::string, double> boundingRadii_map;
    double sc_;
};
