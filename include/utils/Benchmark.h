#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <ompl/util/Console.h>
#include "Mergers/DeterministicMerger.h"
#include "Mergers/BeliefMerger.h"
#include "utils/Instance.h"
#include "PlanValidityCheckers/DeterministicPVC.h"
#include "PlanValidityCheckers/MinkowskiSumBlackmorePVC.h"
#include "PlanValidityCheckers/AdaptiveRiskBlackmorePVC.h"
#include "PlanValidityCheckers/AdaptiveRiskBoundingBoxPVC.h"
#include "PlanValidityCheckers/ChiSquaredBoundaryPVC.h"
#include "PlanValidityCheckers/BoundingBoxBlackmorePVC.h"
#include "PlanValidityCheckers/CDFGridPVC.h"
#include "Planners/KCBS.h"
#include "utils/OmplSetUp.h"


void write_csv(std::string filename, std::tuple<bool, double, double> results);

void run_kcbs_benchmark(InstancePtr mrmp_instance, const double merge_bound, const double comp_time,  std::string filename);

void run_centralized_bsst_benchmark(InstancePtr mrmp_instance, const double comp_time, std::string filename);
