#pragma once
#include "common.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/base/Goal.h>
#include <ompl/control/SpaceInformation.h>
#include <boost/math/special_functions/erf.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bm = boost::math;


class ChanceConstrainedGoal : public ob::Goal
{
public:
    ChanceConstrainedGoal(const oc::SpaceInformationPtr &si, const Location goal, const double toll, const double p_safe);

    bool isSatisfied(const ob::State *st) const override;
  
    bool isSatisfied(const ob::State *st, double *distance) const override;

private:
	std::pair<Eigen::Vector2d, Eigen::Matrix2d> getDistFromState_(const ob::State* st) const;
	void setHalfPlanes_(Polygon combined_poly);
	bool isSafe_(const ob::State* st, double *distance = nullptr) const;

	Eigen::MatrixXd A_;
	Eigen::MatrixXd B_;
	Eigen::VectorXd goal_;
	const double p_safe_;
	const double threshold_;
};
