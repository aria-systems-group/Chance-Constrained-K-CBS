/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for setting up a multi-agent
* motion planning problem for OMPL. The result, is used by KD-CBS
* repeatedly in order to find valid motion plans. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include "../includes/World.h"
#include "../includes/Goals.h"
#include "../includes/OdeFunctions.h"
#include "../includes/collisionChecking.h"
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>


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
            oc::SpaceInformation *si = ss.getSpaceInformation().get();
            // ss.setStateValidityChecker(
                // [si, w, a](const ob::State *state) { return isStateValid_2D(si, w, a, state); });
            // std::vector<int> constrin;
            ss.setStateValidityChecker(std::make_shared<isStateValid_2D_Test>(ss.getSpaceInformation(), w, a));
    
            // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
            // when integration has finished to normalize the orientation values.
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
            ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
    
            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX(a->getStartLocation()[0]);
            start->setY(a->getStartLocation()[1]);
            start->setYaw(0.0);  // start yaw is set manually (TODO)

            ob::GoalPtr goal (new ArbirtryGoal_2D(ss.getSpaceInformation(), a->getGoalLocation(), goalRadius));

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
    OMPL_INFORM("Done!");
    return ssVec;
}