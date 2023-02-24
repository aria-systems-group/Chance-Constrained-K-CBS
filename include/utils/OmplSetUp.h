#pragma once
#include "StateValidityCheckers/RealVectorStateSpaceSVC.h"
#include "StateValidityCheckers/PCCBlackmoreSVC.h"
#include "StateValidityCheckers/AdaptiveRiskBlackmoreSVC.h"
#include "StateValidityCheckers/ChiSquaredBoundarySVC.h"
#include "StateValidityCheckers/CentralizedChiSquaredBoundarySVC.h"
#include "StatePropogators/CarSP.h"
#include "StatePropogators/UncertainLinearSP.h"
#include "StatePropogators/CentralizedUncertainLinearSP.h"
#include "Goals/RealVectorStateSpaceGoals.h"
#include "Goals/BeliefSpaceGoals.h"
#include "OptimizationObjectives/StateCostObjectives.h"
#include "utils/MultiRobotProblemDefinition.h"
#include "Planners/CentralizedBSST.h"
#include "Planners/BSST.h"
#include "Spaces/RealVectorBeliefSpace.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace bm = boost::math;


// the main ompl set-up function
std::vector<MotionPlanningProblemPtr> set_up_all_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ RRT
std::vector<MotionPlanningProblemPtr> set_up_ConstraintRRT_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for K-CBS w/ BSST
std::vector<MotionPlanningProblemPtr> set_up_ConstraintBSST_MP_Problems(InstancePtr mrmp_instance);

// the ompl set-up function for MR-RRT
std::vector<MotionPlanningProblemPtr> set_up_MultiRobotRRT_MP_Problem(InstancePtr mrmp_instance);

// rhw ompl awt-up function for CentralizedBSST
std::vector<MotionPlanningProblemPtr> set_up_CentralizedBSST_Problem(InstancePtr mrmp_instance);
