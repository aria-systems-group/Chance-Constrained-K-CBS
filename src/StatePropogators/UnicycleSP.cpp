#include "StatePropogators/UnicycleSP.h"

void SecondOrderUnicycleODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q = x, y, v, theta, omega
    // c = a, phidot
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // state params
    const double v = q[2];
    const double phi = q[3];
    const double theta = q[4];
    const double carLength = 0.2;

    // Zero out qdot
    qdot.resize (q.size (), 0);
 
    // vehicle model
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = u[0];
    qdot[3] = u[1];
    qdot[4] = (v / carLength) * tan(phi);
}
 
// callback for putting angle [0, 2pi]
void SecondOrderUnicyclePostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
}
