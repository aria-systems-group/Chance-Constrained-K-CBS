#pragma once
#include "ConstraintValidityCheckers/ConstraintValidityChecker.h"
#include "Constraints/BeliefConstraint.h"
#include "Spaces/R2BeliefSpace.h"
#include <algorithm>

class BeliefCVC: public ConstraintValidityChecker
{
public:
	BeliefCVC(Robot *robot, const int num_obs);
	virtual bool satisfiesConstraints(oc::PathControl path, std::vector<ConstraintPtr> constraints) override;
private:
    std::pair<Eigen::Vector2d, Eigen::Matrix2d> getDistFromState_(ob::State* st);
    bool isSafe_(const Eigen::Vector2d mu_ab, const Eigen::Matrix2d Sigma_ab);
    std::vector<std::vector<double>> HalfPlanes_;
    const double p_safe_;
    const double p_coll_dist_;
};

