/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains dynamics models for many common robotic systems.
* They are used by OMPL to plan kinodynamically feasible motion plans
* 
* Current Capabilities Include:
*       * Kinematic Car (KinematicCar)
*********************************************************************/
 
/* Author: Justin Kottinger */

#include <ompl/control/ODESolver.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

/********* Definition of the ODE for the Kinematic Car *********/

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
 
// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

bool isKinematicCarStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
 
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
 
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

/********* Definition of the ODE for the Kinematic Car *********/
