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
// #include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
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
const std::vector<problem> multiAgentSetUp(const World *w);