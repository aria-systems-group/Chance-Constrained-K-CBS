#pragma once
#include "common.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

namespace ob = ompl::base;


class EuclideanPathLengthObjective : public ob::PathLengthOptimizationObjective
{
    public:
        EuclideanPathLengthObjective(const ob::SpaceInformationPtr& si) :
        ob::PathLengthOptimizationObjective(si)
        {
        }
 
        ob::Cost motionCost(const State *s1, const State *s2) const override
        {
            Eigen::Vector2d diff = s1->as<R2BeliefSpace::StateType>()->getXY() - s2->as<R2BeliefSpace::StateType>()->getXY();
            return Cost(diff.norm());
        }
};

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        //ob::StateCostIntegralObjective(si, true)
        ob::StateCostIntegralObjective(si, false)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const
    {
      //std::cout << "clearance: " << si_->getStateValidityChecker()->clearance(s) << std::endl;
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
// class RiskZonesObjective : public ob::StateCostIntegralObjective
// {
// public:
// 	RiskZonesObjective(const ob::SpaceInformationPtr& si, bool enableMotionCostInterpolation) :
//         ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
//     {
//     }
//
//     ob::Cost stateCost(const ob::State* s) const
//     {
//     	std::shared_ptr<StateValidityCheckerBasic> state_vality_checker = std::static_pointer_cast<StateValidityCheckerBasic> (si_->getStateValidityChecker());
//         return ob::Cost(state_vality_checker->checkRiskZones(s));
//     }
//
//     ob::Cost motionCost(const ob::State *s1,const ob::State *s2) const
//     {
//     	if (interpolateMotionCost_)
//     	{
//     		ob::Cost totalCost = this->identityCost();
//
//     		int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
//     		//nd = int(nd/10);
//
//     		ob::State *test1 = si_->cloneState(s1);
//     		ob::Cost prevStateCost = this->stateCost(test1);
//     		if (nd > 1)
//     		{
//     			ob::State *test2 = si_->allocState();
//     			for (int j = 1; j < nd; ++j)
//     			{
//     				si_->getStateSpace()->interpolate(s1, s2, (double) j / (double) nd, test2);
//     				ob::Cost nextStateCost = this->stateCost(test2);
//     				totalCost = ob::Cost(totalCost.value() + this->trapezoid(prevStateCost, nextStateCost,
//     						si_->distance(test1, test2)).value());
//     				std::swap(test1, test2);
//     				prevStateCost = nextStateCost;
//     			}
//     			si_->freeState(test2);
//     		}
//
//     		// Lastly, add s2
//     		totalCost = ob::Cost(totalCost.value() + this->trapezoid(prevStateCost, this->stateCost(s2),
//     				si_->distance(test1, s2)).value());
//
//     		si_->freeState(test1);
//
//     		return totalCost;
//     	}
//     	else
//     		return this->trapezoid(this->stateCost(s1), this->stateCost(s2),
//     				si_->distance(s1, s2));
//     }
// };
//
// class DirVectorsRiskObjective : public ob::StateCostIntegralObjective
// {
// public:
// 	DirVectorsRiskObjective(const ob::SpaceInformationPtr& si) :
//         ob::StateCostIntegralObjective(si, true)
//     {
//     }
//
//     ob::Cost stateCost(const ob::State* s) const
//     {
//     	std::shared_ptr<StateValidityCheckerBasic> state_vality_checker = std::static_pointer_cast<StateValidityCheckerBasic> (si_->getStateValidityChecker());
//         return ob::Cost(state_vality_checker->checkDirVectorsRisk(s));
//     }
// };

// ob::OptimizationObjectivePtr getDirVectorsRiskObjective(const ob::SpaceInformationPtr& si);

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
// ob::OptimizationObjectivePtr getRiskZonesObjective(const ob::SpaceInformationPtr& si, bool motion_cost_interpolation);

ob::OptimizationObjectivePtr getEuclideanPathLengthObjective(const ob::SpaceInformationPtr& si);

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

/** Create an optimization objective which attempts to optimize both
    path length and clearance. We do this by defining our individual
    objectives, then adding them to a MultiOptimizationObjective
    object. This results in an optimization objective where path cost
    is equivalent to adding up each of the individual objectives' path
    costs.
    When adding objectives, we can also optionally specify each
    objective's weighting factor to signify how important it is in
    optimal planning. If no weight is specified, the weight defaults to
    1.0.
*/
ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si);

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax.
 */
// ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);
