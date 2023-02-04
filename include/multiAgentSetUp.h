/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all relevent code for setting up a multi-agent
* motion planning problem for OMPL. The result, is used by KD-CBS
* repeatedly in order to find valid motion plans. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "Instance.h"
#include "Spaces/R2BeliefSpace.h"
#include "StateValidityCheckers/RealVectorStateSpaceSVC.h"
#include "StateValidityCheckers/MultiRobotStateSpaceSVC.h"
#include "StateValidityCheckers/PCCBlackmoreSVC.h"
#include "StatePropogators/CarSP.h"
#include "StatePropogators/UnicycleSP.h"
#include "StatePropogators/MultiCarSP.h"
#include "StatePropogators/UncertainLinearSP.h"
#include "Goals/RealVectorStateSpaceGoals.h"
#include "Goals/MultiRobotStateSpaceGoals.h"
#include "Goals/BeliefSpaceGoals.h"
#include "Planners/ConstraintRespectingRRT.h"
#include "OptimizationObjectives/StateCostObjectives.h"
#include "ConstraintValidityCheckers/DeterministicCVC.h"
#include "MultiRobotProblemDefinition.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

// the main ompl set-up function
std::vector<MotionPlanningProblemPtr> multiAgentSetUp(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ RRT
std::vector<MotionPlanningProblemPtr> set_up_KCBS_RRT_instances(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ BSST
std::vector<MotionPlanningProblemPtr> set_up_KCBS_BSST_instances(InstancePtr mrmp_instance);

// the ompl set-up function for MR-RRT
std::vector<MotionPlanningProblemPtr> set_up_MultiRobotRRT_instances(InstancePtr mrmp_instance);

