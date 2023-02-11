#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <Eigen/Eigenvalues> 
#include <unordered_map>

namespace bm = boost::math;


class DiskBoundedPVC: public BeliefPVC
{
public:
    DiskBoundedPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;
    
private:
    ConflictPtr checkForConflicts_(std::unordered_map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Belief belief_a, const double rad_a, const Belief belief_b, const double rad_b);
    double findBoundingRadius_(const Robot* r);
    // Polygon getMinkowskiSumOfRobots_(Robot* r1, Robot* r2);
    // std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
    // Point addPoints_(const Point &a, const Point &b);
    // Point subtractPoints_(const Point &a, const Point &b);
    // double crossProduct_(const Point &a, const Point &b);
    std::unordered_map<std::string, double> boundingRadii_map;
    double sc_;
};
