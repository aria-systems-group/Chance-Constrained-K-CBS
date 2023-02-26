#pragma once
#include "utils/common.h"
#include "Spaces/RealVectorBeliefSpace.h"
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

class CentralizedCCGoal : public ob::Goal
{
public:
    CentralizedCCGoal(const oc::SpaceInformationPtr &si, const ob::State* goal, const double toll, const double p_safe);

    bool isSatisfied(const ob::State *st) const override;
  
    bool isSatisfied(const ob::State *st, double *distance) const override;

    bool isSingleAgentSatisfied(const ob::State *st, const int idx) const;

private:
    bool isSafe_(const Eigen::Vector2d mu, const Eigen::Matrix2d sigma, const Eigen::Vector2d goal, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> hp, double *distance) const;
    // std::pair<Eigen::Vector2d, Eigen::Matrix2d> getDistFromState_(const ob::State* st) const;
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> createHalfPlanes_(Polygon combined_poly);

    // bool isSafe_(const ob::State* st, double *distance = nullptr) const;

    // Eigen::MatrixXd A_;
    // Eigen::MatrixXd B_;
    std::unordered_map<int, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> half_plane_map_;
    Eigen::VectorXd goal_;
    const int num_agents_;
    const double p_safe_;
    const double threshold_;
};

