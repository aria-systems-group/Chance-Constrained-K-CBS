#pragma once
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/math/special_functions/erf.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <unordered_map>

namespace bm = boost::math;


class Blackmore2PVC: public BeliefPVC
{
public:
    Blackmore2PVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;

    bool independentCheck(ob::State* state1, ob::State* state2);
    
private:
    double ImprovedHyperplaneCCValidityChecker(const Eigen::MatrixXd A, const Eigen::MatrixXd B, const double x_pose, const double y_pose, const Eigen::Matrix2d &PX) const;
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes);
    Polygon getMinkowskiSumOfRobots_(Robot* r1, Robot* r2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
    Point addPoints_(const Point &a, const Point &b);
    Point subtractPoints_(const Point &a, const Point &b);
    double crossProduct_(const Point &a, const Point &b);
    std::unordered_map<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> halfPlane_map_;
    double p_coll_dist_;
};
