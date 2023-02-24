#pragma once
#include "utils/common.h"
#include "utils/MultiRobotProblemDefinition.h"
#include "Spaces/RealVectorBeliefSpace.h"
#include "PlanValidityCheckers/MinkowskiSumBlackmorePVC.h"
#include "PlanValidityCheckers/AdaptiveRiskBlackmorePVC.h"
#include "PlanValidityCheckers/ChiSquaredBoundaryPVC.h"
#include "PlanValidityCheckers/BoundingBoxBlackmorePVC.h"
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace fs = std::filesystem;
namespace oc = ompl::control;


class BeliefCollisionCheckerBenchmark
{
public:
    BeliefCollisionCheckerBenchmark(std::string file1, std::string file2);

    void runBenchmarks();

private:
    std::string beliefList1_;
    std::string beliefList2_;

    oc::SpaceInformationPtr si_{nullptr};
    MultiRobotProblemDefinitionPtr mrmp_pdef_{nullptr};
    InstancePtr instance_{nullptr};

    std::unordered_map<int, ob::State*> agent1_belief_map_;
    std::unordered_map<int, ob::State*> agent2_belief_map_;
};
