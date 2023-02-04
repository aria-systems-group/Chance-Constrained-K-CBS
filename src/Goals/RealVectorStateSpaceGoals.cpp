#include "Goals/RealVectorStateSpaceGoals.h"


R2Goal::R2Goal(const oc::SpaceInformationPtr &si, const ob::ScopedState<> goal, const double toll) : 
    ompl::base::GoalRegion(si), goal_(goal)
{
    setThreshold(toll);
}

double R2Goal::distanceGoal(const ob::State *st) const
{
    // convert ob::state to only xy state
    auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);

    // calc distances and square the results
    const double dx_sq = pow(goal_[0] - xyState->values[0], 2);
    const double dy_sq = pow(goal_[1] - xyState->values[1], 2);

    return sqrt(dx_sq + dy_sq);
}

R3Goal::R3Goal(const ob::SpaceInformationPtr &si, const ob::ScopedState<> goal, const double toll) : 
    ompl::base::GoalRegion(si), goal_(goal)
{
    setThreshold(toll);
}

double R3Goal::distanceGoal(const ob::State *st) const
{
    // convert ob::state to only xy state
    auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyzState = compState->as<ob::RealVectorStateSpace::StateType>(0);

    // calc distances and square the results
    const double dx_sq = pow(goal_[0] - xyzState->values[0], 2);
    const double dy_sq = pow(goal_[1] - xyzState->values[1], 2);
    const double dz_sq = pow(goal_[2] - xyzState->values[2], 2);

    return sqrt(dx_sq + dy_sq + dz_sq);
}
