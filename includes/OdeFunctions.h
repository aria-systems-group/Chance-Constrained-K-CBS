/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains Dynamics models for many common robotic systems.
* 
* These methods are used by OMPL to plan kinodynamically feasible 
* motion plans.
* 
* Current Capabilities Include:
*       * Kinematic Car (KinematicCar)
* 
* Requirements for adding new capabilities:
*       * an ODE that works with ODESolver objects
*       * Post integration proceedure (if neccessary)
*       * State validity function that takes World object as an input
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include <ompl/control/ODESolver.h>
#include "../includes/World.h"

namespace ob = ompl::base;
namespace oc = ompl::control;


/********* Definition of the ODE for the Kinematic Car *********/
// the ODE
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q: [x, y, theta]
    // u: [v, steering rate]
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;
 
    // Zero out qdot
    qdot.resize (q.size (), 0);
 
    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
}
 
// callback for putting angle [0, 2pi]
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}
/********* END ODE for the Kinematic Car *********/
