#pragma once
#include "common.h"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/base/goals/GoalRegion.h>


class R2BeliefSpaceGoal : public ompl::base::GoalRegion
{
public:
    R2BeliefSpaceGoal(const oc::SpaceInformationPtr &si, const ob::ScopedState<> goal) : 
    	ompl::base::GoalRegion(si), goal_(goal)
    {
        setThreshold(0.25);
    }
 
    virtual double distanceGoal(const ob::State *st) const
    {

        double dx = st->as<R2BeliefSpace::StateType>()->getX() - goal_[0];
        double dy = st->as<R2BeliefSpace::StateType>()->getY() - goal_[1];

        return (dx*dx + dy*dy);
        // perform any operations and return a double indicating the distance to the goal
    }
private:
	const ob::ScopedState<> goal_;
};
