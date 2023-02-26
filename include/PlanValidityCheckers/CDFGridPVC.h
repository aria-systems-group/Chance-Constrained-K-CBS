#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Eigenvalues> 
#include <unordered_map>

namespace bm = boost::math;


class CDFGridPVC: public BeliefPVC
{
public:
    CDFGridPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe, int nSteps);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

    bool independentCheck(ob::State* state1, ob::State* state2);

private:
    double chi_squared_quantile_(double v, double p)
    {
        return quantile(bm::chi_squared(v), p);
    }
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Belief belief_a, const Belief belief_b, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> HP, Polygon V);
    Polygon getBoundingBox_(const double r_1, const double r_2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
    std::vector<double> linspace_(double start_in, double end_in, int num_in);
    double cdfRect_(std::vector<Point> B);
    std::unordered_map<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> halfPlane_map_;
    std::unordered_map<std::string, Polygon> polygon_map_;
    double sc_;
    const int disk_steps_;
};