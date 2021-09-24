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
#include "../includes/OdeFunctions.h"
#include "../includes/Goals.h"
#include "../includes/collisionChecking.h"

 
namespace ob = ompl::base;
namespace oc = ompl::control;

// standard real vector control space of any dimension
class StandardControlSpace : public oc::RealVectorControlSpace
{
public:
 
    StandardControlSpace(const ob::StateSpacePtr &stateSpace, int numCntrls) : oc::RealVectorControlSpace(stateSpace, numCntrls)
    {
    }
};

// this function sets-up an ompl planning problem for an arbtrary number of agents
std::vector<oc::SimpleSetup> multiAgentSimpleSetUp(const World *w)
{
    std::cout << "" << std::endl;
    OMPL_INFORM("Setting up planning problem for all agents...");
    std::vector<oc::SimpleSetup> ssVec;
    const double goalRadius = 0.25;
    for (int i=0; i < w->getAgents().size(); i++)
    {
        Agent *a = w->getAgents()[i];
        if (a->getDynamics() == "Kinematic Car")
        {
            // Kinematic Car operates in SE2
            auto space(std::make_shared<ob::SE2StateSpace>());

            // real vector bounds of space are unique to dimension / dynamics
            // get them from the World object
            ob::RealVectorBounds bounds(2);

            bounds.setLow(0, 0);
            bounds.setHigh(0, w->getWorldDimensions()[0]);
            bounds.setLow(1, 0);
            bounds.setHigh(1, w->getWorldDimensions()[1]);
 
            space->setBounds(bounds);

            // create a standard 2D control space 
            const int cntlrDim = 2;
            auto cspace(std::make_shared<StandardControlSpace>(space, cntlrDim));

            // set normalized bounds for the control space as [-1, 1]
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(-1);
            cbounds.setHigh(1);

            cspace->setBounds(cbounds);
 
            // define a simple setup class
            oc::SimpleSetup ss(cspace);

            // set state validity checking for this space
            oc::SpaceInformationPtr siPtr = ss.getSpaceInformation();
            oc::SpaceInformation *si = ss.getSpaceInformation().get();
            ss.setStateValidityChecker(
                [si, w, a](const ob::State *state) { return isStateValid_2D(si, w, a, state); });
    
            // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
            // when integration has finished to normalize the orientation values.
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
            ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
    
            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX(a->getStartLocation()[0]);
            start->setY(a->getStartLocation()[1]);
            start->setYaw(0.0);  // start yaw is set manually (TODO)

            ob::GoalPtr goal (new ArbirtryGoal_2D(siPtr, a->getGoalLocation(), goalRadius));

            ss.setStartState(start);
            ss.setGoal(goal);

            // initialize and set planner
            auto planner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
            ss.setPlanner(planner);

            ssVec.push_back(ss);
        }
        else
        {
            OMPL_ERROR("No model implemented for %s dynamics: %s", a->getName().c_str(), a->getDynamics().c_str());
            return ssVec;
        }
    }
    
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
    OMPL_INFORM("Done!");
    return ssVec;
}



// this follows syntax of printf see (https://www.cplusplus.com/reference/cstdio/printf/)
// OMPL_INFORM("OMPL version: %s", OMPL_VERSION);  // blue font
// OMPL_WARN("OMPL version: %s", OMPL_VERSION);  // yellow font
// OMPL_ERROR("OMPL version: %s", OMPL_VERSION); // red font
 
int main(int argc, char ** argv)
{
    std::string problem = argv[1];
    World *w = yaml2world(problem);
    std::vector<oc::SimpleSetup> allAgentSetUp = multiAgentSimpleSetUp(w);

    for (oc::SimpleSetup s: allAgentSetUp)
    {
        ob::PlannerStatus solved = s.solve(10.0);
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            s.getSolutionPath().asGeometric().printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }
    

    // planWithSimpleSetup();
 
    return 0;
}









