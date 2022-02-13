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
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

/********* Arbirtrary 2D Goal Class *********/
// Defines a circular goal region centered at goal, with radius toll
class ArbirtryGoal_2D : public ompl::base::Goal
{
public:
    ArbirtryGoal_2D(const oc::SpaceInformationPtr &si, std::vector<double> goal, const double toll) : ompl::base::Goal(si)
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
/********* END 2D Goal Class *********/

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

/********* Arbirtrary Composed 2D Goal Class *********/
// Defines a circular goal region centered at goal, with radius toll
class ArbirtryComposedGoal_2D : public ompl::base::Goal
{
public:
    ArbirtryComposedGoal_2D(const oc::SpaceInformationPtr &si, 
        std::vector<double> goal, const double toll, const std::string dynModel) : 
            ompl::base::Goal(si), goal_{goal}, toll_{toll}, dyn_{dynModel}
    {
    }

    virtual bool isSatisfied(const ob::State *st) const
    {
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        std::vector<std::vector<double>> poses;
        int numVs = 0;
        if (dyn_ == "Two Dynamic Cars")
            numVs = 2;
        else if (dyn_ == "Three Dynamic Cars")
            numVs = 3;
        
        if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
        {
            for (int a = 0; a < numVs; a++)
            {
                auto xyState = compState->as<
                    ob::RealVectorStateSpace::StateType>(2*a + 0);
                std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
                poses.push_back(xy_pos);
            }
        }
        else
            OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
        if (poses.size() != numVs)
            OMPL_ERROR("Goal checking not working as expected.");
;
        // given a pos for each agent, calc if all in goal
        for (int a = 0; a < poses.size(); a++)
        {
            const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
            const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
            // if a single agent is out of goal, system is not in goal
            if (sqrt(dx_sq + dy_sq) > toll_)
                return false;
        }

        // survived the pos check through all agents, found goal
        return true;
    }

    // Is a state in a goal region, also fill "distance" with result
    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        std::vector<std::vector<double>> poses;
        int numVs = 0;
        if (dyn_ == "Two Dynamic Cars")
            numVs = 2;
        else if (dyn_ == "Three Dynamic Cars")
            numVs = 3;

        if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
        {
            for (int a = 0; a < numVs; a++)
            {
                auto xyState = compState->as<
                    ob::RealVectorStateSpace::StateType>(2*a + 0);
                std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
                poses.push_back(xy_pos);
            }
        }
        else
            OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
        /* Nothing passed here should change */
        if (poses.size() != numVs)
            OMPL_ERROR("Goal checking not working as expected.");

        // given a pos for each agent, calc if all in goal
        double dist = 0.0;
        bool isSolved = true;
        for (int a = 0; a < poses.size(); a++)
        {
            const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
            const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
            dist += sqrt(dx_sq + dy_sq);
            // if a single agent is out of goal, system is not in goal
            if (sqrt(dx_sq + dy_sq) > toll_)
            {
                isSolved = false;
            }
        }

        // const double d = getDistance2D(st);
        distance = &dist;

        // survived the pos check through all agents, found goal
        return isSolved;
    }

    std::vector<int> isInGoal(const ob::State *st)
    {
        auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
        std::vector<std::vector<double>> poses;
        int numVs = 0;
        if (dyn_ == "Two Dynamic Cars")
            numVs = 2;
        else if (dyn_ == "Three Dynamic Cars")
            numVs = 3;

        if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
        {
            for (int a = 0; a < numVs; a++)
            {
                auto xyState = compState->as<
                    ob::RealVectorStateSpace::StateType>(2*a + 0);
                std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
                poses.push_back(xy_pos);
            }
        }
        else
            OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
        /* Nothing passed here should change */
        if (poses.size() != numVs)
            OMPL_ERROR("Goal checking not working as expected.");

        // given a pos for each agent, calc if all in goal
        std::vector<int> idxSolved;
        for (int a = 0; a < poses.size(); a++)
        {
            const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
            const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
            double dist = sqrt(dx_sq + dy_sq);
            // if a single agent is out of goal, system is not in goal
            if (dist < toll_)
                idxSolved.push_back(a);
        }
        return idxSolved;
    }

private:
    std::vector<double> goal_;
    double toll_;
    const std::string dyn_;
};
/********* END 2D Composed Goal Class *********/
