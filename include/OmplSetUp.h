#pragma once
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
#include "Planners/ConstraintRespectingBSST.h"
#include "OptimizationObjectives/StateCostObjectives.h"
// #include "MultiRobotProblemDefinition.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

namespace po = boost::program_options;
namespace bm = boost::math;


OMPL_CLASS_FORWARD(MotionPlanningProblem);
// the main ompl set-up function
std::vector<MotionPlanningProblemPtr> set_up_all_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ RRT
std::vector<MotionPlanningProblemPtr> set_up_ConstraintRRT_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ BSST
std::vector<MotionPlanningProblemPtr> set_up_ConstraintBSST_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for MR-RRT
std::vector<MotionPlanningProblemPtr> set_up_MultiRobotRRT_MP_Problems(InstancePtr mrmp_instance);
