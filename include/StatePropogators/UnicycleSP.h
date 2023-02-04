#pragma once
#include "common.h"
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>


void SecondOrderUnicycleODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void SecondOrderUnicyclePostIntegration (const ob::State*, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
