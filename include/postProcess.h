#pragma once
#include "Spaces/R2BeliefSpace.h"
#include <fstream>
#include <filesystem>
#include <ompl/control/PathControl.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace fs = std::filesystem;

// std::vector<oc::PathControl> decentralizeTrajectory(std::vector<oc::PathControl> compPlan, 
//     const World *w);

// void checkDisjoint(const std::vector<oc::PathControl> plan,
//     const int begin, int depth, bool &intersection, bool &done);

// std::vector<int> generateExplanation(const std::vector<oc::PathControl> plan);

// // generate date/time information to solutions or solution directories
// std::string GetCurrentTimeForFileName();

// // parent function for including date/time information to files
// fs::path appendTimeToFileName(const fs::path& fileName);

// // write solultion to the system
void exportBeliefPlan(const std::vector<oc::PathControl*> plan, const std::string problem_name);
