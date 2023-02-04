#pragma once
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Defines a circular goal region centered at goal, with radius = toll
class R2Goal : public ompl::base::GoalRegion
{
public:
    R2Goal(const oc::SpaceInformationPtr &si, const ob::ScopedState<> goal, const double toll);

    virtual double distanceGoal(const ob::State *st) const override;

private:
    const ob::ScopedState<> goal_;
};

// Defines a spherical goal region centered at goal, with radius = toll
class R3Goal : public ompl::base::GoalRegion
{
public:
    R3Goal(const ob::SpaceInformationPtr &si, const ob::ScopedState<> goal, const double toll);

    virtual double distanceGoal(const ob::State *st) const override;

private:
    const ob::ScopedState<> goal_;
};
