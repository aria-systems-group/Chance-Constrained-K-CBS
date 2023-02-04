#pragma once
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


void SecondOrderUnicycleODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void SecondOrderUnicyclePostIntegration (const ob::State*, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
