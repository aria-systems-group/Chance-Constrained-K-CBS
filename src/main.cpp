/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
*********************************************************************/
 
/* Author: Justin Kottinger */
 
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/config.h>
#include "../includes/OdeFunctions.h"
#include "../includes/World.h"
// #include <iostream>
// #include <valarray>
// #include <limits>

 
namespace ob = ompl::base;
namespace oc = ompl::control;

// standard real vector control space of any dimension
class StandardControlSpace : public oc::RealVectorControlSpace
{
public:
 
    StandardControlSpace(const ob::StateSpacePtr &stateSpace, const int numCntrls) : oc::RealVectorControlSpace(stateSpace, numCntrls)
    {
    }
};

oc::SimpleSetup* agentSimpleSetUp(const World *w)
{
    Agent *a = w->getAgents()[0];
    if (a->getDynamics() == "KinematicCar")
    {
        // Kinematic Car operates in SE2
        auto space(std::make_shared<ob::SE2StateSpace>());

        // real vector bounds of space are unique to dimension / dynamics
        ob::RealVectorBounds bounds(2);

        bounds.setLow(-1);
        bounds.setHigh(1);
 
        space->setBounds(bounds);
    }
    
 

 
    // // create a standard 2D control space 
    // const int cntlrDim = 2;
    // auto cspace(std::make_shared<StandardControlSpace>(space, cntlrDim));
 
    // // set the bounds for the control space
    // ob::RealVectorBounds cbounds(2);
    // cbounds.setLow(-0.3);
    // cbounds.setHigh(0.3);
 
    // cspace->setBounds(cbounds);
 
    // // define a simple setup class
    // oc::SimpleSetup ss(cspace);
 
    // // set state validity checking for this space
    // oc::SpaceInformation *si = ss.getSpaceInformation().get();
    // ss.setStateValidityChecker(
    //     [si](const ob::State *state) { return isKinematicCarStateValid(si, state); });
 
    // // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // // when integration has finished to normalize the orientation values.
    // auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
    // ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
 
    // ob::ScopedState<ob::SE2StateSpace> start(space);
    // start->setX(-0.5);
    // start->setY(0.0);
    // start->setYaw(0.0);
 
    // ob::ScopedState<ob::SE2StateSpace> goal(space);
    // goal->setX(0.0);
    // goal->setY(0.5);
    // goal->setYaw(0.0);
 
    // ss.setStartAndGoalStates(start, goal, 0.2);

    // auto planner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
 
    // ss.setPlanner(planner);

    // ss.setup();
 
    // ob::PlannerStatus solved = ss.solve(10.0);
 
    // if (solved)
    // {
    //     std::cout << "Found solution:" << std::endl;
 
    //     ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    // }
    // else
    //     std::cout << "No solution found" << std::endl;
    return nullptr;
}



// this follows syntax of printf see (https://www.cplusplus.com/reference/cstdio/printf/)
// OMPL_INFORM("OMPL version: %s", OMPL_VERSION);  // blue font
// OMPL_WARN("OMPL version: %s", OMPL_VERSION);  // yellow font
// OMPL_ERROR("OMPL version: %s", OMPL_VERSION); // red font
 
int main(int argc, char ** argv)
{
    std::string problem = argv[1];
    World *w = yaml2world(problem);
    
    

    // planWithSimpleSetup();
 
    return 0;
}









