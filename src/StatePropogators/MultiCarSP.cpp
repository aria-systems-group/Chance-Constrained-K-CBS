#include "StatePropogators/MultiCarSP.h"

void TwoSecondOrderCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // q = x1, y1, v1, phi1, theta1, x2, y2, v2, phi2, theta2
    // c = a1, phi1, a2, phi2

    // velocities
    const double v1 = q[2];
    const double v2 = q[7];

    // phis
    const double phi1 = q[3];
    const double phi2 = q[8];

    // thetas 
    const double theta1 = q[4];
    const double theta2 = q[9];

    // tuning param
    double carLength = 0.5;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // vehicle 1
    qdot[0] = v1 * cos(theta1);
    qdot[1] = v1 * sin(theta1);
    qdot[2] = u[0];  // dv/dt == accel
    qdot[3] = u[1];  // dphi/dt == steering rate
    qdot[4] = (v1 / carLength) * tan(phi1);
    // vehicle 2
    qdot[5] = v2 * cos(theta2);
    qdot[6] = v2 * sin(theta2);
    qdot[7] = u[2];  // dv/dt == accel
    qdot[8] = u[3];  // dphi/dt == steering rate
    qdot[9] = (v2 / carLength) * tan(phi2);
}
 
// callback for putting angle [0, 2pi]
void TwoSecondOrderCarsPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    //pull the angles from both cars
    ob::CompoundState* cs = result->as<ob::CompoundState>();

    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace::StateType* angleState2 = cs->as<ob::SO2StateSpace::StateType>(3);

    //use ompl to normalize theta
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
    SO2.enforceBounds(angleState2);
}

void ThreeSecondOrderCarsODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // q = x1, y1, v1, phi1, theta1, x2, y2, v2, phi2, theta2, x3, y3, v3, phi3, theta3
    // c = a1, phi1, a2, phi2, a3, phi3

    // velocities
    const double v1 = q[2];
    const double v2 = q[7];
    const double v3 = q[12];

    // phis
    const double phi1 = q[3];
    const double phi2 = q[8];
    const double phi3 = q[13];

    // thetas 
    const double theta1 = q[4];
    const double theta2 = q[9];
    const double theta3 = q[14];

    // tuning param
    double carLength = 0.5;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // vehicle 1
    qdot[0] = v1 * cos(theta1);
    qdot[1] = v1 * sin(theta1);
    qdot[2] = u[0];  // dv/dt == accel
    qdot[3] = u[1];  // dphi/dt == steering rate
    qdot[4] = (v1 / carLength) * tan(phi1);
    // vehicle 2
    qdot[5] = v2 * cos(theta2);
    qdot[6] = v2 * sin(theta2);
    qdot[7] = u[2];  // dv/dt == accel
    qdot[8] = u[3];  // dphi/dt == steering rate
    qdot[9] = (v2 / carLength) * tan(phi2);
    // vehicle 3
    qdot[10] = v3 * cos(theta3);
    qdot[11] = v3 * sin(theta3);
    qdot[12] = u[4];  // dv/dt == accel
    qdot[13] = u[5];  // dphi/dt == steering rate
    qdot[14] = (v3 / carLength) * tan(phi3);
}
 
// callback for putting angle [0, 2pi]
void ThreeSecondOrderCarsPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    //pull the angles from both cars
    ob::CompoundState* cs = result->as<ob::CompoundState>();

    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace::StateType* angleState2 = cs->as<ob::SO2StateSpace::StateType>(3);
    ob::SO2StateSpace::StateType* angleState3 = cs->as<ob::SO2StateSpace::StateType>(5);

    //use ompl to normalize theta
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
    SO2.enforceBounds(angleState2);
    SO2.enforceBounds(angleState3);
}
