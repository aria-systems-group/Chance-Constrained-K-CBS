#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/special_functions/erf.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <unordered_map>

namespace bm = boost::math;


class AdaptiveRiskBlackmorePVC: public BeliefPVC
{
public:
    AdaptiveRiskBlackmorePVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

    bool independentCheck(ob::State* state1, ob::State* state2);
    
private:
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    std::unordered_map<std::string, double> createEtaMap_(const std::string r_a_name, std::map<std::string, Belief> states_map);
    bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes, const double eta_i);
    Polygon getMinkowskiSumOfRobots_(Robot* r1, Robot* r2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
    Point addPoints_(const Point &a, const Point &b);
    Point subtractPoints_(const Point &a, const Point &b);
    double crossProduct_(const Point &a, const Point &b);
    std::unordered_map<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> halfPlane_map_;
};