/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for setting up a multi-agent
* motion planning problem for OMPL. The result, is used by KD-CBS
* repeatedly in order to find valid motion plans. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include "World.h"
#include "Goals.h"
#include "OdeFunctions.h"
#include "collisionChecking.h"
#include "constraintRRT.h"
#include <ompl/control/SimpleSetup.h>
#include <utility>

namespace ob = ompl::base;
namespace oc = ompl::control;

typedef std::pair< std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition> > problem;


// standard real vector control space of any dimension
class StandardControlSpace : public oc::RealVectorControlSpace
{
public:
 
    StandardControlSpace(const ob::StateSpacePtr &stateSpace, int numCntrls) : oc::RealVectorControlSpace(stateSpace, numCntrls)
    {
    }
};

// this function sets-up an ompl planning problem for an arbtrary number of agents
// returns planners to be used
const std::vector<problem> multiAgentSetUp(const World *w)
{
    std::cout << "" << std::endl;
    OMPL_INFORM("Setting up planning problem for all agents...");
    // std::vector<shared_ptr<oc::constraintRRT> >
    std::vector<problem> probDefs;
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

            // construct an instance of  space information from this state space
            auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

            si->setStateValidityChecker(
                std::make_shared<isStateValid_2D_Test>(si, w, a));
    
            // Use the ODESolver to propagate the system.  
            // Call KinematicCarPostIntegration
            // when integration has finished to normalize the orientation values.
            auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &KinematicCarODE));
            si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 
                &KinematicCarPostIntegration));
            // assume that planner integrates dynamics at steps of 0.1 seconds
            si->setPropagationStepSize(0.1);
            si->setMinControlDuration(1);
            si->setMaxControlDuration(10);
            si->setup();

            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX(a->getStartLocation()[0]);
            start->setY(a->getStartLocation()[1]);
            start->setYaw(0.0);  // start yaw is set manually (TODO)

            ob::GoalPtr goal (new ArbirtryGoal_2D(si, a->getGoalLocation(), goalRadius));
            
            // create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // set the start and goal states
            pdef->addStartState(start);
            pdef->setGoal(goal);

            problem prob(si, pdef);

            probDefs.push_back(prob);
        }
        else
        {
            OMPL_ERROR("No model implemented for %s dynamics: %s", a->getName().c_str(), a->getDynamics().c_str());
            return probDefs;
        }
    }
    OMPL_INFORM("Done!");
    return probDefs;
}