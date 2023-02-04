#pragma once
#include "common.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


class R2MultiRobotGoal : public ompl::base::Goal
{
public:
    R2MultiRobotGoal(const oc::SpaceInformationPtr &si, 
        std::vector<double> goal, const double toll, const std::string dynModel);

    virtual bool isSatisfied(const ob::State *st) const;

    // Is a state in a goal region, also fill "distance" with result
    virtual bool isSatisfied(const ob::State *st, double *distance) const;

    std::vector<int> isInGoal(const ob::State *st);

private:
    std::vector<double> goal_;
    double toll_;
    const std::string dyn_;
};
