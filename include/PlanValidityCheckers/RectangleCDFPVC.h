#pragma once
#include "common.h"
#include "PlanValidityCheckers/BeliefPVC.h"
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/math/special_functions/erf.hpp>

namespace bm = boost::math;


class RectangleCDFPVC: public BeliefPVC
{
public:
    RectangleCDFPVC(MultiRobotProblemDefinitionPtr pdef, const double p_safe);

    std::vector<ConflictPtr> validatePlan(Plan p) override;

    bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;
    
private:
    ConflictPtr checkForConflicts_(std::map<std::string, Belief> states_map, const int step);
    bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab, const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> halfPlanes);
    double findBoundingRadius_(const Robot* r);
    Polygon getBoundingBox_(const double r_1, const double r_2);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getHalfPlanes_(Polygon combined_poly);
    std::unordered_map<std::string, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> halfPlane_map_;
};