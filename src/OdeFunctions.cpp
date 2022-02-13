#include "includes/OdeFunctions.h"


/********* Definition of the ODE for the Kinematic Car *********/
// the ODE
void KinematicCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q: [x, y, theta]
    // u: [v, steering rate]
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    const double carLength = 0.2;
 
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

/********* Definition of the ODE for the Dynamic Car *********/
// the ODE
void DynamicCarODE (const oc::ODESolver::StateType& q, 
    const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    // q = x, y, v, phi, theta
    // c = a, phidot
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    // state params
    const double v = q[2];
    const double phi = q[3];
    const double theta = q[4];
    const double carLength = 0.5;

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
void DynamicCarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    //pull the angle from car
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
}
/********* END ODE for the Dynamic Car *********/

/********* Definition of the ODE for the Dynamic Unicycle *********/
// the ODE
void DynamicUnicycleODE (const oc::ODESolver::StateType& q, 
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
void DynamicUnicyclePostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    //pull the angle from car
    ob::CompoundState* cs = result->as<ob::CompoundState>();
    ob::SO2StateSpace::StateType* angleState1 = cs->as<ob::SO2StateSpace::StateType>(1);
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(angleState1);
}
/********* END ODE for the Dynamic Unicycle *********/

/********* Definition of the ODE for Two Dynamic Cars *********/
// the ODE
void TwoDynamicCarsODE (const oc::ODESolver::StateType& q, 
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
void TwoDynamicCarsPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
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
/********* END ODE for Two Dynamic Cars *********/

/********* Definition of the ODE for Three Dynamic Cars *********/
// the ODE
void ThreeDynamicCarsODE (const oc::ODESolver::StateType& q, 
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
void ThreeDynamicCarsPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
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
/********* END ODE for Three Dynamic Cars *********/