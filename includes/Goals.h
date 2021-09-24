/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains Goal classes
* 
* These methods are used by OMPL to plan kinodynamically feasible 
* motion plans.
* 
* Current Capabilities Include:
*       * Circular goal region in 2D
*       * Spherical goal region in 3D
* 
* Requirements for adding new capabilities:
*       * Class that takes same inputs as existing
*********************************************************************/

/* Author: Justin Kottinger */

#pragma once
#include <ompl/control/ODESolver.h>
#include "../includes/World.h"

/********* Arbirtrary 2D Goal Class *********/
// Defines a circular goal region centered at goal, with radius toll
class ArbirtryGoal_2D : public ompl::base::Goal
{
public:
    ArbirtryGoal_2D(oc::SpaceInformationPtr &si, std::vector<double> goal, const double toll) : ompl::base::Goal(si)
    {
      goal_ = goal;
      toll_ = toll;
    }

    virtual bool isSatisfied(const ob::State *st) const
    {
        // convert ob::state to only xy state
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);

        // calc distances and square the results
        const double dx_sq = pow(goal_[0] - xyState->values[0], 2);
        const double dy_sq = pow(goal_[1] - xyState->values[1], 2);

        return (sqrt(dx_sq + dy_sq) < toll_);
    }

    // Is a state in a goal region, also fill "distance" with result
    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        // bool result = isSatisfied(st);

        // convert ob::state to only xy state
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);

        // calc distances and square the results
        const double dx_sq = pow(goal_[0] - xyState->values[0], 2);
        const double dy_sq = pow(goal_[1] - xyState->values[1], 2);

        // get Manhatten distance
        double d = sqrt(dx_sq + dy_sq);

        // const double d = getDistance2D(st);
        distance = &d;

        return (d < toll_);
    }

private:
    std::vector<double> goal_;
    double toll_;
};
/********* END 3D Goal Class *********/

/********* Arbirtrary 3D Goal Class *********/
// Defines a spherical goal region centered at goal, with radius toll
class ArbirtryGoal_3D : public ompl::base::Goal
{
public:
    ArbirtryGoal_3D(const ob::SpaceInformationPtr &si, const std::vector<double> goal, const double toll) : ompl::base::Goal(si)
    {
      goal_ = goal;
      toll_ = toll;
    }

    // Is a state in a goal region, also fill "distance" with result
    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        // bool result = isSatisfied(st);

        // convert ob::state to only xy state
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        auto xyzState = compState->as<ob::RealVectorStateSpace::StateType>(0);

        // calc distances and square the results
        const double dx_sq = pow(goal_[0] - xyzState->values[0], 2);
        const double dy_sq = pow(goal_[1] - xyzState->values[1], 2);
        const double dz_sq = pow(goal_[2] - xyzState->values[2], 2);

        // get Manhatten distance
        double d = sqrt(dx_sq + dy_sq + dz_sq);

        // const double d = getDistance2D(st);
        distance = &d;

        return (d < toll_);
    }

private:
    std::vector<double> goal_;
    double toll_;
};
/********* END 3D Goal Class *********/