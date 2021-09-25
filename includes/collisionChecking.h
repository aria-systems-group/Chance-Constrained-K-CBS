/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains State validity Checks
* 
* These methods are used by OMPL to plan kinodynamically feasible 
* motion plans.
* 
* Current Capabilities Include:
*       * 2D Collision checking with rectangular obstacles
*       * 3D Collision checking with rectangular obstacles
* 
* Requirements for adding new capabilities:
*       * Function that takes same inputs as existing
*********************************************************************/

/* Author: Justin Kottinger */

// #pragma once


/********* Definition of 2D State Validity Checker *********/
bool isStateValid_2D(const oc::SpaceInformation *si, const World *w, const Agent *a, const ob::State *state)
{
    if (si->satisfiesBounds(state))
    {
        // convert ob::state to only xy state
        auto compState = state->as<ompl::base::CompoundStateSpace::StateType>();
        auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);

        // TO DO COLLISION CHECKING WITH OBSTACLES
        return true;

    }
    else
        return false;
}
/********* END 2D State Validity Checker *********/

/********* Definition of 3D State Validity Checker *********/
bool isStateValid_3D(const oc::SpaceInformation *si, const World *w, const Agent *a, const ob::State *state)
{
    if (si->satisfiesBounds(state))
    {
        // convert ob::state to only xy state
        auto compState = state->as<ompl::base::CompoundStateSpace::StateType>();
        auto xyzState = compState->as<ob::RealVectorStateSpace::StateType>(0);

        // TO DO COLLISION CHECKING WITH OBSTACLES
        return true;

    }
    else
        return false;
}
/********* END 3D State Validity Checker *********/
