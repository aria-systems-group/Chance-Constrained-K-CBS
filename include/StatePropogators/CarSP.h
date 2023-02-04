#pragma once
#include "common.h"
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>


void FirstOrderCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void FirstOrderCarODEPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);

void SecondOrderCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void SecondOrderCarODEPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
