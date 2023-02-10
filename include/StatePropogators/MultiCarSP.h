#pragma once
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


void TwoSecondOrderCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void TwoSecondOrderCarsPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);

void ThreeSecondOrderCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot);
 
// callback for putting angle [0, 2pi]
void ThreeSecondOrderCarsPostIntegration (const ob::State* /*state*/, 
    const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
