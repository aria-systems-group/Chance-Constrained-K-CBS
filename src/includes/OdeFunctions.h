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
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

/********* Definition of the ODE for the Kinematic Car *********/
// the ODE
void KinematicCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void KinematicCarPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
/********* END ODE for the Kinematic Car *********/

/********* Definition of the ODE for the Dynamic Car *********/
// the ODE
void DynamicCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void DynamicCarPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
/********* END ODE for the Dynamic Car *********/

/********* Definition of the ODE for the Dynamic Unicycle *********/
// the ODE
void DynamicUnicycleODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void DynamicUnicyclePostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
/********* END ODE for the Dynamic Unicycle *********/

/********* Definition of the ODE for Two Dynamic Cars *********/
// the ODE
void TwoDynamicCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void TwoDynamicCarsPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
/********* END ODE for Two Dynamic Cars *********/

/********* Definition of the ODE for Three Dynamic Cars *********/
// the ODE
void ThreeDynamicCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void ThreeDynamicCarsPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
/********* END ODE for Three Dynamic Cars *********/
